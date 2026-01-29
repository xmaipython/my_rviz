#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

#include <my_rviz_plugins/xju_task.h>
#include <my_rviz_plugins/keepOutZone.h>

#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/GetMap.h>
#include <QTabWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QLabel>

#include <atomic>
#include <thread>

namespace my_rviz_plugins
{

class MyPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit MyPanel(QWidget* parent = 0);
  ~MyPanel() override;

private Q_SLOTS:
  // ===== 建图页=====
  void onStartMapping();
  void onStopMapping();
  void onSaveMap();
  // ===== 导航页=====
  void onSendNavGoal();
  void onCancelNavGoal();
  void onClearCostmaps();
  void onGlobalLocalization();

  // ===== 虚拟墙 keepOutZone =====
  void onDrawWall();
  void onUseWall();
  void onDeleteWall();
  void onClearAllWall();
  void onReceiveWall();

  // ===== 清洁路径 task =====
  void onDrawZone();
  void onCreateTeachPath();
  void onCreateZPath();
  void onCreateOPath();
  void onUsePath();

private:
  void buildUi();

  // 日志
  void logClear();
  void logInfo(const QString& s);
  void logWarn(const QString& s);
  void logError(const QString& s);

  bool ensureServicesReady(double timeout_sec = 2.0);

  // srv 调用封装
  bool callXjuTask(uint8_t type, uint8_t command,
                   const std::string& dir,
                   const std::string& path_name,
                   const std::string& map,
                   std::string& out_message);

  bool callXjuZone(uint8_t command, uint8_t cost,
                   const std::vector<geometry_msgs::PointStamped>& zone,
                   uint32_t id,
                   uint32_t& out_id);

  // 导航动作（线程执行，避免卡UI）
  void navThreadFunc(double x, double y, double z, double yaw,
                     const std::string& planner,
                     const std::string& controller,
                     const std::string& recovery_behaviors);

  static void yawToQuatZW(double yaw, double& qz, double& qw);

  std::string getMapSaveDir() const; // 获取保存目录
  std::string makeTimeStem() const;  // 生成 map_yyyyMMdd_HHmmss 的名字

private:
  ros::NodeHandle nh_;
  ros::ServiceClient static_map_client_; 
  // /task /zone
  ros::ServiceClient task_client_;
  ros::ServiceClient zone_client_;
  
  // clear costmaps
  ros::ServiceClient clear_costmaps_client_;

  // global_localization
  ros::ServiceClient global_localization_client_;

  // Action client（MBF）
  using MbfClient = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;
  std::unique_ptr<MbfClient> mbf_ac_;

  // ===== UI布局：上tab / 中间按钮区 / 下日志 =====
  QTabWidget* tabs_{nullptr};

  QTextEdit* log_{nullptr};  // 小日志条

  // 清洁路径输入
  QLineEdit* use_path_name_{nullptr};

  // 虚拟墙输入
  QSpinBox* delete_wall_id_{nullptr};

  // 导航输入
  QDoubleSpinBox* nav_x_{nullptr};
  QDoubleSpinBox* nav_y_{nullptr};
  QDoubleSpinBox* nav_z_{nullptr};
  QDoubleSpinBox* nav_yaw_{nullptr};

  QLineEdit* planner_{nullptr};
  QLineEdit* controller_{nullptr};
  QLineEdit* recovery_behaviors_{nullptr};

  // 导航线程控制
  std::atomic<bool> nav_running_{false};
  std::atomic<bool> nav_cancel_requested_{false};
  std::thread nav_thread_;
};

}  // namespace my_rviz_plugins
