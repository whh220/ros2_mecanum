// fuzzy_dwa_critic.cpp — fuzzylite 驱动的模糊DWA critic
#include "sam_bot_new/fuzzy_dwa_critic.hpp"
#include <cmath>
#include <algorithm>
#include "fl/Headers.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sam_bot_new
{

// ========== 引擎构建 ==========

static std::unique_ptr<fl::Engine> _build_guide(fl::InputVariable *& he, fl::InputVariable *& od,
                                                 fl::OutputVariable *& hw)
{
  auto engine = std::make_unique<fl::Engine>("guide");

  he = new fl::InputVariable("He", 0, 180);
  he->addTerm(new fl::Triangle("S", 0, 0, 30));
  he->addTerm(new fl::Triangle("M", 15, 60, 105));
  he->addTerm(new fl::Triangle("L", 75, 120, 180));
  engine->addInputVariable(he);

  od = new fl::InputVariable("Od", 0, 3);
  od->addTerm(new fl::Triangle("N", 0, 0, 1));
  od->addTerm(new fl::Triangle("M", 0.5, 1.5, 2.5));
  od->addTerm(new fl::Triangle("F", 2, 3, 3));
  engine->addInputVariable(od);

  hw = new fl::OutputVariable("Hw", 0, 1);
  hw->setDefuzzifier(new fl::Centroid(100));
  hw->addTerm(new fl::Triangle("L", 0, 0, 0.4));
  hw->addTerm(new fl::Triangle("M", 0.25, 0.5, 0.75));
  hw->addTerm(new fl::Triangle("H", 0.6, 0.9, 1));
  hw->setDefaultValue(0.5);
  engine->addOutputVariable(hw);

  auto * rules = new fl::RuleBlock;
  rules->addRule(fl::Rule::parse("if Od is N then Hw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and He is S then Hw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and He is M then Hw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and He is L then Hw is H", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and He is S then Hw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and He is M then Hw is H", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and He is L then Hw is H", engine.get()));
  engine->addRuleBlock(rules);

  engine->configure("Minimum", "Maximum", "Minimum", "Maximum", "Centroid", "General");
  return engine;
}

static std::unique_ptr<fl::Engine> _build_safety(
    fl::InputVariable *& dn, fl::InputVariable *& od, fl::InputVariable *& gd,
    fl::OutputVariable *& ow, fl::OutputVariable *& vw)
{
  auto engine = std::make_unique<fl::Engine>("safety");

  dn = new fl::InputVariable("Dn", 0, 1);
  dn->addTerm(new fl::Triangle("L", 0, 0, 0.35));
  dn->addTerm(new fl::Triangle("M", 0.2, 0.5, 0.8));
  dn->addTerm(new fl::Triangle("H", 0.65, 1, 1));
  engine->addInputVariable(dn);

  od = new fl::InputVariable("Od", 0, 3);
  od->addTerm(new fl::Triangle("N", 0, 0, 1));
  od->addTerm(new fl::Triangle("M", 0.5, 1.5, 2.5));
  od->addTerm(new fl::Triangle("F", 2, 3, 3));
  engine->addInputVariable(od);

  gd = new fl::InputVariable("Gd", 0, 3);
  gd->addTerm(new fl::Triangle("N", 0, 0, 1));
  gd->addTerm(new fl::Triangle("M", 0.5, 1.5, 2.5));
  gd->addTerm(new fl::Triangle("F", 2, 3, 3));
  engine->addInputVariable(gd);

  ow = new fl::OutputVariable("Ow", 0, 1);
  ow->setDefuzzifier(new fl::Centroid(100));
  ow->addTerm(new fl::Triangle("L", 0, 0, 0.35));
  ow->addTerm(new fl::Triangle("M", 0.25, 0.5, 0.75));
  ow->addTerm(new fl::Triangle("H", 0.65, 0.85, 1));
  ow->setDefaultValue(0.65);
  engine->addOutputVariable(ow);

  vw = new fl::OutputVariable("Vw", 0, 1);
  vw->setDefuzzifier(new fl::Centroid(100));
  vw->addTerm(new fl::Triangle("L", 0, 0, 0.35));
  vw->addTerm(new fl::Triangle("M", 0.25, 0.5, 0.75));
  vw->addTerm(new fl::Triangle("H", 0.65, 0.85, 1));
  vw->setDefaultValue(0.5);
  engine->addOutputVariable(vw);

  auto * rules = new fl::RuleBlock;
  // Od=N (near): 距离近时优先避障
  rules->addRule(fl::Rule::parse("if Od is N and Dn is H and Gd is N then Ow is H and Vw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is N and Dn is H and Gd is M then Ow is H and Vw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is N and Dn is H and Gd is F then Ow is H and Vw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is N and Dn is L and Gd is N then Ow is M and Vw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is N and Dn is L and Gd is M then Ow is M and Vw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is N and Dn is L and Gd is F then Ow is M and Vw is M", engine.get()));
  // Od=M (medium)
  rules->addRule(fl::Rule::parse("if Od is M and Dn is H and Gd is N then Ow is H and Vw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and Dn is H and Gd is M then Ow is H and Vw is L", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and Dn is H and Gd is F then Ow is M and Vw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and Dn is L and Gd is N then Ow is M and Vw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and Dn is L and Gd is M then Ow is L and Vw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is M and Dn is L and Gd is F then Ow is L and Vw is H", engine.get()));
  // Od=F (far): 距离远时加速
  rules->addRule(fl::Rule::parse("if Od is F and Dn is H and Gd is N then Ow is M and Vw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and Dn is H and Gd is M then Ow is M and Vw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and Dn is H and Gd is F then Ow is M and Vw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and Dn is L and Gd is N then Ow is L and Vw is M", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and Dn is L and Gd is M then Ow is L and Vw is H", engine.get()));
  rules->addRule(fl::Rule::parse("if Od is F and Dn is L and Gd is F then Ow is L and Vw is H", engine.get()));
  engine->addRuleBlock(rules);

  engine->configure("Minimum", "Maximum", "Minimum", "Maximum", "Centroid", "General");
  return engine;
}

// ========== FuzzyDWACritic ==========

FuzzyDWACritic::FuzzyDWACritic() = default;
FuzzyDWACritic::~FuzzyDWACritic() = default;

void FuzzyDWACritic::onInit()
{
  auto nh = node_.lock();
  if (!nh) return;

  nh->declare_parameter(dwb_plugin_name_ + "." + name_ + ".max_angle",
                        rclcpp::ParameterValue(0.8));
  nh->get_parameter(dwb_plugin_name_ + "." + name_ + ".max_angle", max_angle_);

  nh->declare_parameter(dwb_plugin_name_ + "." + name_ + ".max_vel_x",
                        rclcpp::ParameterValue(1.0));
  nh->get_parameter(dwb_plugin_name_ + "." + name_ + ".max_vel_x", max_vel_x_);

  nh->declare_parameter(dwb_plugin_name_ + "." + name_ + ".sensing_radius",
                        rclcpp::ParameterValue(2.0));
  nh->get_parameter(dwb_plugin_name_ + "." + name_ + ".sensing_radius", sensing_radius_);

  nh->declare_parameter(dwb_plugin_name_ + "." + name_ + ".lookahead_dist",
                        rclcpp::ParameterValue(1.0));
  nh->get_parameter(dwb_plugin_name_ + "." + name_ + ".lookahead_dist", lookahead_dist_);

  // 构建 fuzzylite 引擎（仅一次）
  guide_engine_ = _build_guide(guide_he_, guide_od_, guide_hw_);
  safety_engine_ = _build_safety(safe_dn_, safe_od_, safe_gd_, safe_ow_, safe_vw_);

  fuzzy_weights_pub_ = nh->create_publisher<std_msgs::msg::Float32MultiArray>(
      "fuzzy_weights", 10);
}

bool FuzzyDWACritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  robot_x_ = pose.x;
  robot_y_ = pose.y;
  robot_yaw_ = pose.theta;
  goal_x_ = goal.x;
  goal_y_ = goal.y;
  global_plan_ = global_plan;

  // 环境感知
  // 计算裁剪后的前瞻点
  lookahead_x_ = goal_x_;
  lookahead_y_ = goal_y_;
  if (!global_plan_.poses.empty()) {
    double accum = 0.0;
    lookahead_x_ = global_plan_.poses.back().x;
    lookahead_y_ = global_plan_.poses.back().y;
    for (size_t i = 1; i < global_plan_.poses.size(); ++i) {
      double dx = global_plan_.poses[i].x - global_plan_.poses[i - 1].x;
      double dy = global_plan_.poses[i].y - global_plan_.poses[i - 1].y;
      accum += std::hypot(dx, dy);
      if (accum >= lookahead_dist_) {
        lookahead_x_ = global_plan_.poses[i].x;
        lookahead_y_ = global_plan_.poses[i].y;
        break;
      }
    }
  }

  double He = _heading_err_deg();
  double Od, Dn;
  _scan_costmap(Od, Dn);
  double Gd = std::hypot(lookahead_x_ - robot_x_, lookahead_y_ - robot_y_);

  // fuzzylite 推理
  guide_he_->setValue(He);
  guide_od_->setValue(Od);
  guide_engine_->process();
  hw_ = guide_hw_->getValue();

  safe_dn_->setValue(Dn);
  safe_od_->setValue(Od);
  safe_gd_->setValue(Gd);
  safety_engine_->process();
  ow_ = safe_ow_->getValue();
  vw_ = std::max(safe_vw_->getValue(), 0.3);

  // 每3秒打印权重
  auto nh = node_.lock();
  if (nh) {
    RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 3000,
        "模糊权重: hw=%.3f ow=%.3f vw=%.3f | He=%.0f° Od=%.2fm Dn=%.3f Gd=%.2fm",
        hw_, ow_, vw_, He, Od, Dn, Gd);
  }

  // 发布权重
  if (fuzzy_weights_pub_) {
    auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
    msg->data = {static_cast<float>(hw_), static_cast<float>(ow_),
                 static_cast<float>(vw_), static_cast<float>(He),
                 static_cast<float>(Od), static_cast<float>(Dn),
                 static_cast<float>(Gd)};
    fuzzy_weights_pub_->publish(std::move(msg));
  }

  return true;
}

double FuzzyDWACritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  if (traj.poses.empty()) return 0.0;

  const auto & pose = traj.poses.back();

  // 航向分
  double bearing = std::atan2(lookahead_y_ - pose.y, lookahead_x_ - pose.x);
  double angle_diff = std::abs(std::atan2(std::sin(bearing - pose.theta),
                                           std::cos(bearing - pose.theta)));
  double h = angle_diff / M_PI;

  // 障碍物分
  auto * costmap = costmap_ros_->getCostmap();
  unsigned int mx, my;
  double o = 0.0;
  if (costmap->worldToMap(pose.x, pose.y, mx, my)) {
    unsigned char cost = costmap->getCost(mx, my);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) return -1.0;
    if (cost != nav2_costmap_2d::FREE_SPACE) o = cost / 254.0;
  } else {
    return -1.0;
  }

  // 速度分
  double v = 1.0 - std::abs(traj.velocity.x) / max_vel_x_;

  return hw_ * h + ow_ * o + vw_ * v;
}

// ========== 环境感知 ==========

void FuzzyDWACritic::_scan_costmap(double & od, double & dn)
{
  od = sensing_radius_;
  dn = 0.0;

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) return;

  unsigned int cx, cy;
  if (!costmap->worldToMap(robot_x_, robot_y_, cx, cy)) return;

  int radius = static_cast<int>(sensing_radius_ / costmap->getResolution());
  int w = static_cast<int>(costmap->getSizeInCellsX());
  int h = static_cast<int>(costmap->getSizeInCellsY());

  int obstacle_cnt = 0, total_cnt = 0;
  double min_dist = sensing_radius_;

  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      int px = cx + dx, py = cy + dy;
      if (px < 0 || px >= w || py < 0 || py >= h) continue;
      total_cnt++;
      unsigned char cost = costmap->getCost(px, py);
      if (cost >= 100) {
        obstacle_cnt++;
        double wx, wy;
        costmap->mapToWorld(px, py, wx, wy);
        double dist = std::hypot(wx - robot_x_, wy - robot_y_);
        if (dist < min_dist) min_dist = dist;
      }
    }
  }

  od = min_dist;
  dn = (total_cnt > 0) ? static_cast<double>(obstacle_cnt) / total_cnt : 0.0;
}

double FuzzyDWACritic::_heading_err_deg()
{
  double bearing = std::atan2(lookahead_y_ - robot_y_, lookahead_x_ - robot_x_);
  double err = std::abs(std::atan2(std::sin(bearing - robot_yaw_),
                                    std::cos(bearing - robot_yaw_)));
  return err * 180.0 / M_PI;
}

}  // namespace sam_bot_new

PLUGINLIB_EXPORT_CLASS(sam_bot_new::FuzzyDWACritic, dwb_core::TrajectoryCritic)
