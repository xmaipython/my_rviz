#include "my_rviz_plugins/my_panel.h"
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>   // ros::package::getPath()：通过包名拿到包所在路径
#include <QDateTime>       // QDateTime：获取当前时间并格式化成字符串
#include <QDir>            // QDir：检查目录是否存在/创建目录
#include <QProcess>        // QProcess：在程序里执行外部命令（比 system 更稳）

#include <cmath>
#include <sstream>
#include <QMetaObject>
namespace my_rviz_plugins
{

MyPanel::MyPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  task_client_ = nh_.serviceClient<my_rviz_plugins::xju_task>("/xju_task");
  zone_client_ = nh_.serviceClient<my_rviz_plugins::keepOutZone>("/xju_zone");
  clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base_flex/clear_costmaps");
  static_map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  global_localization_client_ = nh_.serviceClient<std_srvs::Trigger>("/global_localization");
  // MBF action client
  mbf_ac_.reset(new MbfClient("move_base_flex/move_base", true));

  buildUi();
}

MyPanel::~MyPanel()
{
  nav_cancel_requested_ = true;

  if (nav_thread_.joinable()) {
    nav_thread_.join();
  }
}
static std::string trimCopy(std::string s)
{
  auto notSpace = [](int ch){ return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
  s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
  return s;
}

static std::vector<std::string> splitCsv(const std::string& csv)
{
  std::vector<std::string> out;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trimCopy(item);
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

void MyPanel::buildUi()
{
  // ========= 全局美化 =========
  this->setStyleSheet(
    "QWidget { font-size: 13px; }"
    "QTabWidget::pane { border: 1px solid #444444; border-radius: 10px; }"
    "QTabBar::tab { padding: 8px 16px; margin-right: 6px; border-radius: 10px; }"
    "QPushButton { border-radius: 12px; padding: 10px; font-weight: 700; }"
    "QGroupBox { font-weight: 800; margin-top: 8px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; }"
    "QTextEdit { border-radius: 10px; padding: 6px; }"
  );

  auto* root = new QVBoxLayout;
  root->setContentsMargins(10, 10, 10, 10);
  root->setSpacing(10);

  // ====== 上：横向分页 ======
  tabs_ = new QTabWidget(this);
  tabs_->setTabPosition(QTabWidget::North);

  // ====== 下：日志条======
  log_ = new QTextEdit(this);
  log_->setReadOnly(true);
  log_->setFixedHeight(85);         
  log_->setPlaceholderText("调试输出...");

  // ================== Tab 0：建图 ==================
  QWidget* mapping_page = new QWidget(this);
  auto* mapping_layout = new QVBoxLayout(mapping_page);
  mapping_layout->setContentsMargins(10, 28, 10, 10);
  mapping_layout->setSpacing(12);

  auto* map_group = new QGroupBox("建图", mapping_page);
  auto* map_grid = new QGridLayout(map_group);
  map_grid->setHorizontalSpacing(12);
  map_grid->setVerticalSpacing(12);

  auto* btn_start_map = new QPushButton("开始建图", map_group);
  auto* btn_stop_map  = new QPushButton("停止建图", map_group);
  auto* btn_save_map  = new QPushButton("保存地图", map_group);

  btn_start_map->setMinimumHeight(44);
  btn_stop_map->setMinimumHeight(44);
  btn_save_map->setMinimumHeight(44);

  map_grid->addWidget(btn_start_map, 0, 0);
  map_grid->addWidget(btn_stop_map,  0, 1);
  map_grid->addWidget(btn_save_map,  1, 0, 1, 2);

  mapping_layout->addWidget(map_group);
  mapping_layout->addStretch(1);

  connect(btn_start_map, SIGNAL(clicked()), this, SLOT(onStartMapping()));
  connect(btn_stop_map,  SIGNAL(clicked()), this, SLOT(onStopMapping()));
  connect(btn_save_map, SIGNAL(clicked()), this, SLOT(onSaveMap()));

  // ================== Tab 1：导航 ==================
  QWidget* nav_page = new QWidget(this);
  auto* nav_layout = new QVBoxLayout(nav_page);
  nav_layout->setContentsMargins(10, 28, 10, 10);
  nav_layout->setSpacing(12);

  auto* nav_group = new QGroupBox("输入点导航", nav_page);
  auto* nav_form = new QFormLayout(nav_group);
  nav_form->setLabelAlignment(Qt::AlignRight);
  nav_form->setFormAlignment(Qt::AlignTop);
  
  nav_form->setContentsMargins(0, 18, 0, 12);
  nav_form->setVerticalSpacing(8);
  nav_x_ = new QDoubleSpinBox(nav_group);
  nav_y_ = new QDoubleSpinBox(nav_group);
  nav_yaw_ = new QDoubleSpinBox(nav_group);

  nav_x_->setRange(-10000, 10000);
  nav_y_->setRange(-10000, 10000);
  nav_yaw_->setRange(-3.14159, 3.14159);

  nav_x_->setDecimals(3);
  nav_y_->setDecimals(3);
  nav_yaw_->setDecimals(3);

  nav_x_->setValue(0.0);
  nav_y_->setValue(0.0);
  nav_yaw_->setValue(0.0);

  nav_form->addRow("x", nav_x_);
  nav_form->addRow("y", nav_y_);
  nav_form->addRow("yaw(rad)", nav_yaw_);

  auto* nav_btn_group = new QGroupBox("操作", nav_page);
  auto* nav_btn_grid = new QGridLayout(nav_btn_group);
  nav_btn_grid->setHorizontalSpacing(12);
  nav_btn_grid->setVerticalSpacing(12);

  auto* btn_send_goal   = new QPushButton("开始导航", nav_btn_group);
  auto* btn_cancel_goal = new QPushButton("取消导航", nav_btn_group);
  auto* btn_clear_cm    = new QPushButton("清除代价地图", nav_btn_group);
  auto* btn_global_loc  = new QPushButton("重定位", nav_btn_group);

  btn_send_goal->setMinimumHeight(44);
  btn_cancel_goal->setMinimumHeight(44);
  btn_clear_cm->setMinimumHeight(44);
  btn_global_loc->setMinimumHeight(44);

  nav_btn_grid->addWidget(btn_send_goal,   0, 0);
  nav_btn_grid->addWidget(btn_cancel_goal, 0, 1);
  nav_btn_grid->addWidget(btn_clear_cm,    1, 0);
  nav_btn_grid->addWidget(btn_global_loc,  1, 1);

  nav_layout->addWidget(nav_group);
  nav_layout->addWidget(nav_btn_group);
  nav_layout->addStretch(1);

  connect(btn_send_goal,   SIGNAL(clicked()), this, SLOT(onSendNavGoal()));
  connect(btn_cancel_goal, SIGNAL(clicked()), this, SLOT(onCancelNavGoal()));
  connect(btn_clear_cm,    SIGNAL(clicked()), this, SLOT(onClearCostmaps()));
  connect(btn_global_loc,  SIGNAL(clicked()), this, SLOT(onGlobalLocalization()));

  // ================== Tab 2：绘制虚拟墙 ==================
  QWidget* wall_page = new QWidget(this);
  auto* wall_layout = new QVBoxLayout(wall_page);
  wall_layout->setContentsMargins(10, 28, 10, 10);
  wall_layout->setSpacing(12);

  auto* wall_group = new QGroupBox("虚拟墙", wall_page);
  auto* wall_grid = new QGridLayout(wall_group);
  wall_grid->setHorizontalSpacing(12);
  wall_grid->setVerticalSpacing(12);

  auto* btn_draw_wall = new QPushButton("开始绘制虚拟墙", wall_group);
  auto* btn_use_wall  = new QPushButton("停止绘制并生成", wall_group);
  auto* btn_clear_all = new QPushButton("清空全部虚拟墙", wall_group);

  btn_draw_wall->setMinimumHeight(44);
  btn_use_wall->setMinimumHeight(44);
  btn_clear_all->setMinimumHeight(44);

  auto* del_box = new QGroupBox("删除指定虚拟墙", wall_group);
  del_box->setContentsMargins(30, 10, 10, 10);
  auto* del_layout = new QHBoxLayout(del_box);
  del_layout->setContentsMargins(10, 28, 10, 10);
  del_layout->setSpacing(12);

  delete_wall_id_ = new QSpinBox(del_box);
  delete_wall_id_->setRange(0, 1000000);
  delete_wall_id_->setValue(0);

  auto* btn_del_wall = new QPushButton("删除", del_box);
  btn_del_wall->setMinimumHeight(40);

  del_layout->addWidget(new QLabel("id:", del_box));
  del_layout->addWidget(delete_wall_id_, 1);
  del_layout->addWidget(btn_del_wall);

  // auto* btn_receive = new QPushButton("（占位）", wall_group);
  // btn_receive->setMinimumHeight(10);

  wall_grid->addWidget(btn_draw_wall, 0, 0);
  wall_grid->addWidget(btn_use_wall,  0, 1);
  wall_grid->addWidget(del_box,       1, 0, 1, 2);
  wall_grid->addWidget(btn_clear_all, 2, 0, 1, 2);
  // wall_grid->addWidget(btn_receive,   3, 0, 1, 2);

  wall_layout->addWidget(wall_group);
  wall_layout->addStretch(1);

  connect(btn_draw_wall, SIGNAL(clicked()), this, SLOT(onDrawWall()));
  connect(btn_use_wall,  SIGNAL(clicked()), this, SLOT(onUseWall()));
  connect(btn_del_wall,  SIGNAL(clicked()), this, SLOT(onDeleteWall()));
  connect(btn_clear_all, SIGNAL(clicked()), this, SLOT(onClearAllWall()));
  // connect(btn_receive,   SIGNAL(clicked()), this, SLOT(onReceiveWall()));

  // ================== Tab 3：绘制清洁路径 ==================
  QWidget* path_page = new QWidget(this);
  auto* path_layout = new QVBoxLayout(path_page);
  path_layout->setContentsMargins(10, 28, 10, 10);
  path_layout->setSpacing(12);

  auto* task_group = new QGroupBox("清洁路径", path_page);
  auto* task_grid = new QGridLayout(task_group);
  task_grid->setHorizontalSpacing(12);
  task_grid->setVerticalSpacing(12);

  auto* btn_draw_zone = new QPushButton("绘制清洁区域", task_group);
  auto* btn_teach     = new QPushButton("创建示教路径", task_group);
  auto* btn_zpath     = new QPushButton("创建Z字形路径", task_group);
  auto* btn_opath     = new QPushButton("创建回字形路径", task_group);

  btn_draw_zone->setMinimumHeight(44);
  btn_teach->setMinimumHeight(44);
  btn_zpath->setMinimumHeight(44);
  btn_opath->setMinimumHeight(44);

  task_grid->addWidget(btn_draw_zone, 0, 0);
  task_grid->addWidget(btn_teach,     0, 1);
  task_grid->addWidget(btn_zpath,     1, 0);
  task_grid->addWidget(btn_opath,     1, 1);

  auto* use_group = new QGroupBox("使用路径", task_group);
  auto* use_layout = new QHBoxLayout(use_group);
  use_layout->setContentsMargins(10, 28, 10, 10);
  use_layout->setSpacing(10);
  use_path_name_ = new QLineEdit(use_group);
  use_path_name_->setPlaceholderText("输入 path_name");
  auto* btn_use_path = new QPushButton("启用路径", use_group);
  btn_use_path->setMinimumHeight(40);

  use_layout->addWidget(use_path_name_, 1);
  use_layout->addWidget(btn_use_path);
  task_grid->addWidget(use_group, 2, 0, 1, 2);

  path_layout->addWidget(task_group);
  path_layout->addStretch(1);

  connect(btn_draw_zone, SIGNAL(clicked()), this, SLOT(onDrawZone()));
  connect(btn_teach,     SIGNAL(clicked()), this, SLOT(onCreateTeachPath()));
  connect(btn_zpath,     SIGNAL(clicked()), this, SLOT(onCreateZPath()));
  connect(btn_opath,     SIGNAL(clicked()), this, SLOT(onCreateOPath()));
  connect(btn_use_path,  SIGNAL(clicked()), this, SLOT(onUsePath()));

  // ===== Tabs add =====
  tabs_->addTab(mapping_page, "建图");
  tabs_->addTab(nav_page,     "导航");
  tabs_->addTab(wall_page,    "虚拟墙");
  tabs_->addTab(path_page,    "清洁路径");

  // ===== root layout =====
  root->addWidget(tabs_, 1);
  root->addWidget(log_);

  setLayout(root);

  logInfo("UI Ready!");
}

// ---------------- 日志 ----------------
void MyPanel::logClear()
{
  if (!log_) return;
  QMetaObject::invokeMethod(log_, "clear", Qt::QueuedConnection);
}

void MyPanel::logInfo(const QString& s)
{
  if (log_) {
    QMetaObject::invokeMethod(
      log_, "append", Qt::QueuedConnection,
      Q_ARG(QString, QString("✅ %1").arg(s))
    );
  }
  ROS_INFO_STREAM(s.toStdString());
}

void MyPanel::logWarn(const QString& s)
{
  if (log_) {
    QMetaObject::invokeMethod(
      log_, "append", Qt::QueuedConnection,
      Q_ARG(QString, QString("⚠️ %1").arg(s))
    );
  }
  ROS_WARN_STREAM(s.toStdString());
}

void MyPanel::logError(const QString& s)
{
  if (log_) {
    QMetaObject::invokeMethod(
      log_, "append", Qt::QueuedConnection,
      Q_ARG(QString, QString("❌ %1").arg(s))
    );
  }
  ROS_ERROR_STREAM(s.toStdString());
}

bool MyPanel::ensureServicesReady(double timeout_sec)
{
  const bool ok_task = task_client_.waitForExistence(ros::Duration(timeout_sec));
  const bool ok_zone = zone_client_.waitForExistence(ros::Duration(timeout_sec));

  if (!ok_task) logWarn("Service task not available.");
  if (!ok_zone) logWarn("Service zone not available.");

  return ok_task && ok_zone;
}

// ---------------- srv 调用封装 ----------------
bool MyPanel::callXjuTask(uint8_t type, uint8_t command,
                          const std::string& dir,
                          const std::string& path_name,
                          const std::string& map,
                          std::string& out_message)
{
  my_rviz_plugins::xju_task srv;
  srv.request.type = type;
  srv.request.command = command;
  srv.request.dir = dir;
  srv.request.path_name = path_name;
  srv.request.map = map;

  if (!task_client_.call(srv)) {
    out_message = "task call() failed";
    return false;
  }
  out_message = srv.response.message;
  return true;
}

bool MyPanel::callXjuZone(uint8_t command, uint8_t cost,
                          const std::vector<geometry_msgs::PointStamped>& zone,
                          uint32_t id,
                          uint32_t& out_id)
{
  my_rviz_plugins::keepOutZone srv;
  srv.request.command = command;
  srv.request.cost = cost;
  srv.request.zone = zone;
  srv.request.id = id;

  if (!zone_client_.call(srv)) {
    return false;
  }
  out_id = srv.response.id;
  return true;
}

// ---------------- 虚拟墙 ----------------
void MyPanel::onDrawWall()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t command = 3;
  const uint8_t cost = 0;
  const std::vector<geometry_msgs::PointStamped> zone;
  const uint32_t id = 0;

  uint32_t resp_id = 0;
  if (callXjuZone(command, cost, zone, id, resp_id)) {
    logInfo(QString("[虚拟墙]开始绘制虚拟墙"));
  } else {
    logError("[虚拟墙]调用失败");
  }
}

void MyPanel::onUseWall()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t command = 4;
  const uint8_t cost = 0;
  const std::vector<geometry_msgs::PointStamped> zone;
  const uint32_t id = 0;

  uint32_t resp_id = 0;
  if (callXjuZone(command, cost, zone, id, resp_id)) {
    logInfo(QString("[虚拟墙]停止绘制并生成虚拟墙"));
  } else {
    logError("[虚拟墙]调用失败");
  }
}

void MyPanel::onDeleteWall()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t command = 1;
  const uint8_t cost = 0;
  const std::vector<geometry_msgs::PointStamped> zone;
  const uint32_t id = static_cast<uint32_t>(delete_wall_id_ ? delete_wall_id_->value() : 0);

  uint32_t resp_id = 0;
  if (callXjuZone(command, cost, zone, id, resp_id)) {
    logInfo(QString("[虚拟墙]删除虚拟墙请求已发送"));
  } else {
    logError("[虚拟墙]调用失败");
  }
}

void MyPanel::onClearAllWall()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t command = 2;
  const uint8_t cost = 0;
  const std::vector<geometry_msgs::PointStamped> zone;
  const uint32_t id = 0;

  uint32_t resp_id = 0;
  if (callXjuZone(command, cost, zone, id, resp_id)) {
    logInfo(QString("[虚拟墙]清空全部虚拟墙请求已发送"));
  } else {
    logError("[虚拟墙]调用失败");
  }
}

void MyPanel::onReceiveWall()
{
  logInfo("功德+1");
}

// ---------------- 清洁路径 ----------------
void MyPanel::onDrawZone()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t type_ = 1;
  const uint8_t command = 0;

  std::string message;
  if (callXjuTask(type_, command, "", "", "", message)) {
    logInfo(QString("[路径绘制]返回：%1").arg(QString::fromStdString(message)));
  } else {
    logError("[路径绘制]调用失败");
  }
}

void MyPanel::onCreateTeachPath()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t type_ = 1;
  const uint8_t command = 1;

  std::string message;
  if (callXjuTask(type_, command, "", "", "", message)) {
    logInfo(QString("[路径绘制]返回：%1").arg(QString::fromStdString(message)));
  } else {
    logError("[路径绘制]调用失败");
  }
}

void MyPanel::onCreateZPath()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t type_ = 1;
  const uint8_t command = 2;

  std::string message;
  if (callXjuTask(type_, command, "", "", "", message)) {
    logInfo(QString("[路径绘制]返回：%1").arg(QString::fromStdString(message)));
  } else {
    logError("[路径绘制]调用失败");
  }
}

void MyPanel::onCreateOPath()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t type_ = 1;
  const uint8_t command = 3;

  std::string message;
  if (callXjuTask(type_, command, "", "", "", message)) {
    logInfo(QString("[路径绘制]返回：%1").arg(QString::fromStdString(message)));
  } else {
    logError("[路径绘制]调用失败");
  }
}

void MyPanel::onUsePath()
{
  logClear();
  if (!ensureServicesReady()) return;

  const uint8_t type_ = 0;
  const uint8_t command = 0;
  const std::string path_name = use_path_name_ ? use_path_name_->text().toStdString() : std::string("");

  std::string message;
  if (callXjuTask(type_, command, "", path_name, "", message)) {
    logInfo(QString("[路径绘制]返回：%1").arg(QString::fromStdString(message)));
  } else {
    logError("[路径绘制]调用失败");
  }
}

// ---------------- 建图占位 ----------------
void MyPanel::onStartMapping()
{
  logClear();
  logInfo("开始建图");
}

void MyPanel::onStopMapping()
{
  logClear();
  logInfo("停止建图");
}

void MyPanel::onSaveMap()
{
  logClear();

  const std::string dir = getMapSaveDir(); // 获取保存目录（<pnc>/map）
  if (dir.empty()) {                       // 目录获取失败
    logError("保存目录定位失败：找不到包 pnc 或无法创建 pnc/map 目录");
    return;                                // 直接退出
  }

  const std::string stem = makeTimeStem();            // 生成 map_时间 的名字
  const std::string prefix = dir + "/" + stem;        // 生成最终前缀路径，例如 /home/robot/clean_ws/src/pnc/map/map_20260129_083012

  // 等价命令：rosrun map_server map_saver -f <prefix>
  // 说明：map_saver 会生成 <prefix>.pgm 和 <prefix>.yaml
  QString program = "rosrun";                         // 要执行的程序名：rosrun
  QStringList args;                                   // 参数列表
  args << "map_server" << "map_saver" << "-f"
       << QString::fromStdString(prefix);             // 拼出参数：map_server map_saver -f <prefix>


  // 用 QProcess 执行外部命令（比 system 更可控）
  QProcess proc;                                      // 创建进程对象
  proc.start(program, args);                          // 启动：rosrun map_server map_saver -f ...
  bool ok = proc.waitForFinished(60000);              // 等待最多60s

  if (!ok) {                                          // 超时还没结束
    proc.kill();                                      // 强制结束进程
    logError("保存地图超时，已终止 map_saver");
    return;
  }

  int exit_code = proc.exitCode();                    // 获取退出码（0 表示成功）
  QString stderr_out = proc.readAllStandardError();   // 读取错误输出（如果失败更好定位）
  QString stdout_out = proc.readAllStandardOutput();  // 读取正常输出

  if (exit_code == 0) {                               // 成功
    logInfo(QString("保存成功：%1.{pgm,yaml}").arg(QString::fromStdString(prefix)));
  } else {                                            // 失败
    logError(QString("保存失败,exit_code=%1").arg(exit_code));
    if (!stdout_out.isEmpty()) logWarn(QString("stdout: %1").arg(stdout_out));
    if (!stderr_out.isEmpty()) logWarn(QString("stderr: %1").arg(stderr_out));
  }
}

std::string MyPanel::getMapSaveDir() const
{
  const std::string pkg_path = ros::package::getPath("xju_pnc"); // 通过包名“pnc”获取包路径（例如 /home/robot/clean_ws/src/pnc）
  if (pkg_path.empty()) {                                   // 如果没找到这个包，pkg_path 会是空
    return "";                                              // 返回空表示失败
  }

  const QString dir = QString::fromStdString(pkg_path) + "/map"; // 拼出保存目录：<pnc包路径>/map
  QDir d(dir);                                                   // 用 QDir 表示这个目录
  if (!d.exists()) {                                             // 如果目录不存在
    if (!d.mkpath(".")) {                                        // mkpath(".") 表示创建这个目录（包含父目录）
      return "";                                                 // 创建失败就返回空
    }
  }
  return dir.toStdString();                                      // 返回真实可用的目录路径
}
std::string MyPanel::makeTimeStem() const
{
  const QString ts = QDateTime::currentDateTime()
                       .toString("yyyyMMdd_HHmmss"); // 取当前时间并格式化，例如 20260129_083012
  return std::string("map_") + ts.toStdString();     // 拼接成 map_20260129_083012
}

// ---------------- 导航：四元数转换 ----------------
void MyPanel::yawToQuatZW(double yaw, double& qz, double& qw)
{
  qz = std::sin(yaw * 0.5);
  qw = std::cos(yaw * 0.5);
}

// ---------------- 导航：开始/取消/清图 ----------------
void MyPanel::onSendNavGoal()
{
  logClear();

  if (nav_running_) {
    logWarn("导航正在进行中，请先取消或等待结束。");
    return;
  }

  const double x = nav_x_->value();
  const double y = nav_y_->value();
  const double z = 0.0;
  const double yaw = nav_yaw_->value();

  const std::string planner = "GlobalPlanner";
  const std::string controller = "TebLocalPlannerROS";
  const std::string recovery = ""; 

  nav_cancel_requested_ = false;
  nav_running_ = true;

  // 线程执行，避免卡 RViz
  nav_thread_ = std::thread(&MyPanel::navThreadFunc, this, x, y, z, yaw, planner, controller, recovery);
  nav_thread_.detach();  // 不阻塞 RViz（由 nav_running_ 控制）
}

void MyPanel::onCancelNavGoal()
{
  nav_cancel_requested_ = true;
  logWarn("已请求取消导航（将尽快 cancel_goal）。");

  // 尽力取消 action
  if (mbf_ac_) {
    try {
      mbf_ac_->cancelAllGoals();
    } catch (...) {}
  }
}

void MyPanel::onClearCostmaps()
{
  logClear();

  if (!clear_costmaps_client_.waitForExistence(ros::Duration(2.0))) {
    logError("clear_costmaps 服务不存在：/move_base_flex/clear_costmaps");
    return;
  }

  std_srvs::Empty srv;
  if (clear_costmaps_client_.call(srv)) {
    logInfo("已清除代价地图");
  } else {
    logError("清除代价地图失败");
  }
}

// 重定位
void MyPanel::onGlobalLocalization()
{
  logClear();

  if (!global_localization_client_.waitForExistence(ros::Duration(2.0))) {
    logError("global_localization 服务不存在");
    return;
  }

  std_srvs::Trigger srv;
  if (global_localization_client_.call(srv)) {
    if (srv.response.success) {
      logInfo(QString("全局重定位成功").arg(QString::fromStdString(srv.response.message)));
    } else {
      logWarn(QString("全局重定位返回:%1").arg(QString::fromStdString(srv.response.message)));
    }
  } else {
    logError("触发全局重定位失败:Trigger call() 返回 false");
  }
}


// ---------------- 导航线程 ----------------
void MyPanel::navThreadFunc(double x, double y, double z, double yaw,
                            const std::string& planner,
                            const std::string& controller,
                            const std::string& recovery_behaviors)
{
  auto finish = [this]() {
    nav_running_ = false;
  };

  // wait server
  if (!mbf_ac_) {
    logError("ActionClient 未初始化。");
    finish();
    return;
  }

  int max_retries = 15;
  int retry_count = 0;

  while (retry_count < max_retries && ros::ok()) {

    if (nav_cancel_requested_) {
      logWarn("导航被用户取消。");
      try { mbf_ac_->cancelAllGoals(); } catch (...) {}
      finish();
      return;
    }

    // 连接服务器（10s）
    // logInfo("等待 move_base_flex/move_base action server ...");
    if (!mbf_ac_->waitForServer(ros::Duration(10.0))) {
      logError("连接 move_base_flex action 超时（10s）。");
      finish();
      return;
    }

    // 组 goal（对齐你 py：frame_id=map）
    double qz, qw;
    yawToQuatZW(yaw, qz, qw);

    mbf_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = z;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = qz;
    goal.target_pose.pose.orientation.w = qw;

    goal.planner = planner;
    goal.controller = controller;
    goal.recovery_behaviors = splitCsv(recovery_behaviors);

    {
      std::ostringstream ss;
      ss << "发送目标: x=" << x << " y=" << y << " z=" << z
         << " yaw=" << yaw << " (qz=" << qz << ", qw=" << qw << ")"
         << " planner=" << planner << " controller=" << controller;
      // logInfo(QString::fromStdString(ss.str()));
    }

    mbf_ac_->sendGoal(goal);
    logInfo("开始导航...");

    // 等待 120s
    bool finished = mbf_ac_->waitForResult(ros::Duration(150.0));

    if (nav_cancel_requested_) {
      logWarn("导航被用户取消。");
      try { mbf_ac_->cancelAllGoals(); } catch (...) {}
      finish();
      return;
    }

    if (!finished) {
      logWarn("导航超时（120s），取消目标并重试...");
      try { mbf_ac_->cancelGoal(); } catch (...) {}
      retry_count++;
      std::ostringstream ss;
      ss << "正在尝试第 " << retry_count << " 次重试...";
      logWarn(QString::fromStdString(ss.str()));
      continue;
    }

    // 有结果
    actionlib::SimpleClientGoalState st = mbf_ac_->getState();
    if (st == actionlib::SimpleClientGoalState::SUCCEEDED) {
      logInfo("导航成功！");
      finish();
      return;
    } else {
      std::ostringstream ss;
      ss << "导航失败，状态: " << st.toString() << "，重试...";
      logError(QString::fromStdString(ss.str()));
      retry_count++;
      continue;
    }
  }

  logError("导航失败：已达到最大重试次数（15）。");
  finish();
}

}  // namespace my_rviz_plugins

PLUGINLIB_EXPORT_CLASS(my_rviz_plugins::MyPanel, rviz::Panel)
