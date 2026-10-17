#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  // 从配置文件读取瞄准相关参数并做单位转换（度->弧度或直接读取）
  // yaw_offset_：整体yaw修正（度转弧度）
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  // pitch_offset_：整体pitch修正（度转弧度）
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  // comming_angle_ / leaving_angle_：用于小陀螺（旋转）时判定装甲板进入/离开射击区域的角度阈值（度->弧度）
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  // high/low speed delay：用于根据目标角速度选择不同的预测延迟
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  // decision_speed：角速度阈值，用以判断高/低速
  decision_speed_ = yaml["decision_speed"].as<double>();
  // 电控帧内弹速无效（<14）时的兜底弹速，需按实际弹速配置
  default_bullet_speed_ = yaml["default_bullet_speed"].as<double>(23.0);
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    // 双枪位偏差（可选），为左右开火时的额外yaw偏移（度->弧度）
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  // 如果没有目标，返回空命令
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  auto ekf = target.ekf();
  // 根据目标角速度选取高/低速延迟（用绝对值，带符号会漏掉反向旋转的目标）
  double delay_time =
    std::abs(target.ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  // 弹速无效（电控未回传/未配置）时用配置默认值兜底，避免弹道解算失真
  if (bullet_speed < 14) {
    if (tools::delta_time(std::chrono::steady_clock::now(), last_bullet_warn_) > 1.0) {
      tools::logger()->warn(
        "[Aimer] bullet_speed {:.1f} 无效，使用配置默认值 {:.1f}", bullet_speed,
        default_bullet_speed_);
      last_bullet_warn_ = std::chrono::steady_clock::now();
    }
    bullet_speed = default_bullet_speed_;
  }

  // 计算预测时间点（考虑检测+传输延时与发射延时）并在目标上做预测
  auto future = timestamp;
  if (to_now) {
    // 若以当前为准，则将当前实际时间与帧时间差（传输延时）加上 delay_time
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    // 若不以当前为准，使用固定小延时（detector-aimer耗时约0.005）+delay
    auto dt = 0.005 + delay_time;  // detector-aimer耗时0.005 + 发弹延时
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  // 选择初始瞄准点并做可行性检查
  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    return {false, false, 0, 0};
  }

  // 计算初始弹道（基于距离 d0 和高度），检查是否可解
  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间：目标会移动，飞行时间取决于目标未来位置，故使用迭代预测（最多10次）
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建若干目标副本用于不同预测时刻

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 基于预测目标选取瞄准点并计算弹道
    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 如果弹道不可解，则返回失败
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 收敛判定：飞行时间变化足够小则认为收敛
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 基于最终选定的瞄准点与弹道计算输出角度（yaw, pitch）并加上配置偏置
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  // 世界坐标系下pitch向上为负
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺：状态 [7] 是角速度 w（rad/s），[8] 是半径 r
  // （此前误用 r 判断，|r| 被限幅在 0.05~0.5，条件恒真，小陀螺分支成为死代码）
  // 滞回（2026-10-14 近距抖动排查）：w 是含噪的 EKF 估计值，直接与单阈值比较
  // 会在 普通/小陀螺 选板分支间逐帧横跳、指令 yaw 跟着翻转。进入 2.2、退出
  // 1.6，杜绝阈值抖动（假自转被 Target 门控刹停后 w 会快速回落，滞回保证
  // 分支切换只发生一次）。
  auto w_abs = std::abs(target.ekf_x()[7]);
  if (w_abs > 2.2) spin_mode_ = true;
  else if (w_abs < 1.6) spin_mode_ = false;

  if (!spin_mode_ && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim