// fuzzy_dwa_critic.hpp — 模糊DWA critic：环境感知 + fuzzylite推理 + 三项评分，融合为一个插件
#pragma once

#include <memory>
#include <string>
#include "dwb_core/trajectory_critic.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace fl { class Engine; class InputVariable; class OutputVariable; }

namespace sam_bot_new
{

class FuzzyDWACritic : public dwb_core::TrajectoryCritic
{
public:
  FuzzyDWACritic();
  ~FuzzyDWACritic() override;
  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;

protected:
  // 环境感知
  void _scan_costmap(double & od, double & dn);
  double _heading_err_deg();

  // fuzzylite 引擎（onInit构建一次）
  std::unique_ptr<fl::Engine> guide_engine_;
  std::unique_ptr<fl::Engine> safety_engine_;
  fl::InputVariable *guide_he_, *guide_od_;
  fl::OutputVariable *guide_hw_;
  fl::InputVariable *safe_dn_, *safe_od_, *safe_gd_;
  fl::OutputVariable *safe_ow_, *safe_vw_;

  // 权重监控
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fuzzy_weights_pub_;

  // 当前状态缓存
  double robot_x_{0}, robot_y_{0}, robot_yaw_{0};
  double goal_x_{0}, goal_y_{0};
  nav_2d_msgs::msg::Path2D global_plan_;
  double lookahead_x_{0}, lookahead_y_{0};  // 裁剪后的前瞻点

  // 当前周期模糊输出
  double hw_{0.5}, ow_{0.1}, vw_{0.2};

  // 参数
  double max_angle_{1.57};
  double max_vel_x_{1.0};
  double sensing_radius_{2.0};
  double lookahead_dist_{1.0};
};

}  // namespace sam_bot_new
