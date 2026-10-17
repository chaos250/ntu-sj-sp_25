#include "target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
// Target 类实现：负责单个目标的 EKF 估计、预测与观测更新。
// 主要职责：
// - 使用扩展卡尔曼滤波器（EKF）维护目标状态（旋转中心、角速度、半径等）。
// - 在 `predict()` 中根据时间推进状态并设置过程噪声矩阵 Q（对前哨站有特判）。
// - 在 `update()` 中进行装甲板与状态的匹配，并调用 EKF 的观测更新。
// - `armor_xyza_list()` 将 EKF 状态转为每块装甲板的世界坐标并返回，用于瞄准计算。

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on
  // 过程噪声（Piecewise White Noise Model）
  // 参考：Kalman filter 白噪声模型，过程噪声方差根据 dt 放缩
  // 对前哨站（outpost）使用与普通目标不同的噪声尺度
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  // 装甲板匹配：基于目标模型（车体中心+半径）和检测到的装甲板，确定观测对应的装甲序号 id
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  update_ypda(armor, id);
}

void Target::update_ypda(const Armor & armor, int id)
{
  // 使用观测更新 EKF：
  // - 先计算观测函数的雅可比 H
  // - 组装测量噪声矩阵 R（依据角度差与距离动态调整）
  // - 定义非线性观测函数 h(x) 将状态映射为观测量（yaw, pitch, distance, angle）
  // - 使用 EKF 的 update 接口完成状态更新
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  // ---- 创新门控：逐通道比对观测与预测，超阈通道放大 R（等效跳过该通道更新）----
  // 2026-10-14 近距抖动排查的重修：旧实现超阈时 z[3] = model_angle 用预测顶替
  // 观测后照常 update —— 顶替后的创新≈0，滤波器"欣然接受"；模型角含
  // x[6]+id·90°，id 翻转注入 90° 伪测量跳变，EKF 拟合出 w=±2~4rad/s 假自转
  // （实测 a 在 0.7s 内转 250°），越过瞄准器 2rad/s 小陀螺阈值后选板分支横跳、
  // 指令左右摆 = 近距抖动根因（远距翻转帧占比低，链条转不起来所以稳）。
  // 现在超阈时放大该通道 R（不喂任何伪测量），并衰减 w 打破假自转；连续拒帧
  // 达上限仍放行（真实朝向突变场景，滤波自行追上）。门控期间该通道协方差自然
  // 增长，放行后测量权重更大，收敛反而更快。
  Eigen::VectorXd innov = z_subtract(z, h(ekf_.x));

  constexpr double MAX_ANGLE_INNOV = 0.35;  // rad ≈ 20°，角度通道
  constexpr int MAX_ANGLE_HOLD = 4;         // 帧
  constexpr double MAX_POS_INNOV = 0.30;    // rad ≈ 17°，中心 yaw/pitch 通道
  constexpr double MAX_DIST_INNOV = 0.50;   // m，距离通道
  constexpr int MAX_POS_HOLD = 5;           // 帧
  constexpr double R_SKIP = 1e4;            // 拒帧时 R 放大倍数（等效跳过）
  constexpr double R_SOFT = 25.0;           // 噪声帧降权倍数（不跳过）
  constexpr double W_DECAY_MIN = 1.5;       // rad/s，|w| 超此值才衰减（真实转动不误伤）

  bool gate_angle = false;
  auto model_angle = tools::limit_rad(ekf_.x[6] + id * 2 * CV_PI / armor_num_);
  if (std::abs(innov[3]) > MAX_ANGLE_INNOV) {
    if (angle_hold_count_ >= MAX_ANGLE_HOLD) {
      angle_hold_count_ = 0;  // 连续超阈已达上限：放行本帧，滤波自行追上
    } else {
      angle_hold_count_++;
      gate_angle = true;
      R(3, 3) *= R_SKIP;
      // 假自转刹车：门控期间衰减角速度估计，让模型角停止旋转、重新接受真实
      // 测量。仅 |w| 明显偏大时衰减（真车公转半径小、转速低，不会误伤）。
      // 前哨站转速可真实达到 2.5rad/s（predict 有钳制），不做衰减。
      if (name != ArmorName::outpost && std::abs(ekf_.x[7]) > W_DECAY_MIN) ekf_.x[7] *= 0.9;
      tools::logger()->debug(
        "[Target] 角度翻转抑制: meas={:.1f}° pred={:.1f}° skip", z[3] * 57.3,
        model_angle * 57.3);
    }
  } else {
    angle_hold_count_ = 0;
  }

  // 位置通道门控：镜像/翻转帧会把中心 yaw/pitch/距离拉偏 ±0.5m 级（位置通道
  // 此前没有任何门控，是近距抖动放大链的一环）。超阈放大 R，连续拒帧放行。
  bool gate_pos = false;
  if (
    std::abs(innov[0]) > MAX_POS_INNOV || std::abs(innov[1]) > MAX_POS_INNOV ||
    std::abs(innov[2]) > MAX_DIST_INNOV) {
    if (pos_hold_count_ >= MAX_POS_HOLD) {
      pos_hold_count_ = 0;  // 连续超阈已达上限：放行本帧
    } else {
      pos_hold_count_++;
      gate_pos = true;
      if (std::abs(innov[0]) > MAX_POS_INNOV) R(0, 0) *= R_SKIP;
      if (std::abs(innov[1]) > MAX_POS_INNOV) R(1, 1) *= R_SKIP;
      if (std::abs(innov[2]) > MAX_DIST_INNOV) R(2, 2) *= R_SKIP;
      tools::logger()->debug(
        "[Target] 位置门控: innov_yaw={:.1f}° pitch={:.1f}° d={:.2f}m", innov[0] * 57.3,
        innov[1] * 57.3, innov[2]);
    }
  } else {
    pos_hold_count_ = 0;
  }

  // 噪声帧降权（不跳过）：浅谷区（sl>30° 且 dp<DROP_MIN，即未触发 roll 拉力的
  // 0.9~1.5m 噪声地板）自由搜索的 best 是角点噪声游走；板贴画面边缘时 IPPE
  // 双解翻转最严重 —— 两类帧角度通道放大 R 降权，不让噪声喂进 w。
  bool soft = false;
  if (armor.yaw_drop < 60.0 && armor.yaw_slip > 30.0 * CV_PI / 180.0 && z[2] < 1.8)
    soft = true;
  if (armor.edge_off > 0.58) soft = true;  // tan30°
  if (soft) R(3, 3) *= R_SOFT;

  ekf_.update(z, H, R, h, z_subtract);

  // r/l 几何钳制：r 是公转半径、l = r2-r1（长板相对短板的半径增量），物理上均不可为负。
  // 手持装甲板的 id 随机翻转（armor_num_=4 时 id 1/3 才观测 l）会把 l 拉向 -0.2~-0.65、
  // r 拉到 -0.2，r+l<0 使位置镜像 → diverged 误杀轨道（19:57 日志 40 次发散的实测来源）。
  // 钳到 0 后滤波器随时可通过后续更新重新拉回正值。
  if (ekf_.x[8] < 0.0) ekf_.x[8] = 0.0;
  if (ekf_.x[9] < 0.0) ekf_.x[9] = 0.0;

  // 联调日志：观测与滤波后状态逐帧对照，用于定位 EKF 发散发生在测量侧
  // （z_yaw/d 帧间跳变）还是模型侧（r/l 漂移）。gate 标志：a=角度拒帧、
  // p=位置拒帧、s=噪声帧降权（见上方创新门控）。
  tools::logger()->debug(
    "[Target] z_yaw={:.1f}° d={:.2f}m a={:.1f}° w={:.2f} r={:.3f} l={:.3f} nis={:.1f} gate={}{}{}",
    z[3] * 57.3, z[2], ekf_.x[6] * 57.3, ekf_.x[7], ekf_.x[8], ekf_.x[9], ekf_.last_nis,
    gate_angle ? "a" : "", gate_pos ? "p" : "", soft ? "s" : "");
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

// 计算装甲板的世界坐标
// 该函数将 EKF 状态转为每块装甲板的世界坐标并返回，用于瞄准计算。

bool Target::diverged() const
{
  // r 是公转半径。静止目标（无公转）时滤波器把 r 从初值 0.2 收敛到 ~0 是
  // 自我修正——id 翻转造成的 ±90° 位置镜像跳动被 r→0 完全吸收，位置与角度
  // 解耦。旧下界 0.05 会在收敛完成瞬间把轨道误杀 → 重建 → 瞄准跳变 → 近距
  // 猛烈来回（19:57 日志实测 r 0.246→0.039 即被误杀）。负值已由 update 后
  // 的钳制挡掉，此处下界只留数值容差。
  // 上界 0.5→1.0：真实快速横向移动会被轨道模型部分吸收进 r（19:56 甩板段
  // r 实测 0.2→0.37），急转瞬时冲到 0.531 就被 0.5 上界误杀 = 甩板中途丢
  // 目标。1.0m 以上的公转半径在任何真实目标上都不可能，仍视为发散。
  auto r_ok = ekf_.x[8] > -0.05 && ekf_.x[8] < 1.0;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > -0.05 && ekf_.x[8] + ekf_.x[9] < 1.0;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
