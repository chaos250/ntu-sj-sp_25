#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

//solvePnP（获得姿态）
void Solver::solve(Armor & armor) const
{
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  // IPPE 平面位姿存在双解，逐帧翻转会导致 yaw 测量帧间跳变；此处不做选解，
  // 翻转的抑制统一放在 Target::update_ypda 的创新门控里（见 target.cpp）——
  // 按滤波预测值拒绝翻转帧，避免在求解器里用跨帧先验选解（曾因矩形姿态
  // 180° 镜像解重投影误差几乎相同，先验被锁死在镜像盆地导致云台固定偏向一侧）。
  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  optimize_yaw(armor);
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // solve
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

void Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  // 全范围搜索（云台 yaw ±70°）。此处不做小窗口/时间先验——矩形姿态的
  // 180° 镜像解重投影误差几乎相同，窗口一旦落在镜像盆地就会锁死（2026-10-12
  // 联调实测云台固定偏向一侧无法追踪）；翻转抑制统一交给 Target 的创新门控。
  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  // yaw_raw 需在搜索前赋值：roll 滑移门控要用它衡量候选 yaw 与 IPPE 原始解
  // 的偏离（见 armor_reprojection_error 注释）。
  armor.yaw_raw = armor.ypr_in_world[0];

  // 第一遍：无 roll 拉力的自由搜索（仅透视不对称惩罚），得到自由盆地误差。
  // 顺带取 raw 处的基准误差 e_raw，用于第二遍的门控判定。
  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];
  auto e_raw = 1e10;
  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
    if (std::abs(tools::limit_rad(yaw - armor.yaw_raw)) < 0.5 * CV_PI / 180.0) e_raw = error;
  }

  // roll 滑移门控（2026-10-13 定稿，当晚联调校准）：手持装甲板绕视轴的 roll 会
  // 被"仅 yaw"的模板搜索吸收进 yaw，假盆地以少量误差优势赢过 raw，滑移 30-80°，
  // 造成 EKF 拟合虚假公转（实测 w 爬到 1.8rad/s、a 一路爬坡）、位置预测绕圈、
  // 云台猛烈来回后大幅转走。判断依据是滑移本身的三个特征：
  //   1. 滑移量 |best-raw| > 8°：真实转向时 raw 与 best 都≈真值，滑移≈0；
  //      roll 或镜像盆地时滑移 10° 起步（镜像≈180°）。
  //   2. raw 附近误差面平坦：drop = e_raw - min(e(raw±15°)) < 300px。roll 谷地
  //      里 raw 两侧误差几乎不降；尖锐误差面上朝真值一侧会显著下降（drop 大），
  //      说明 raw 只是噪声偏离，自由搜索的 best 才是对的，不能拉。
  //      阈值来自联调日志经验分布：尖锐面 drop p99=36px、max=46px，真实 roll
  //      谷地 drop=163~243px（第一次定 150px 全部漏触发，近距 roll 完全没修）。
  //   3. 距离上限 1.8m：远距装甲板小、raw 噪声大，拉力会把 best 钉在噪声 raw
  //      上。注意必须是硬截止而非渐近衰减——联调日志显示 1.3~1.4m 处衰减把
  //      拉力压到 50~136px/rad，低于谷地坡度（~13px/°），argmin 落在 raw 与
  //      滑移值中间（实测 raw=-14.7 时 best=22.4，残余 37°），逐帧两侧翻转
  //      即"左右摇头"。拉力要么全强度（42px/° 压过谷地坡度，argmin 直接落在
  //      raw），要么为零（自由搜索），不存在中间态。
  // 三个条件同时成立才是 roll 谷地。此前版本（固定 10° 死区 + 42px/° 无条件
  // 拉力）的失败：拉力把 argmin 钉在死区边界 best=raw±10°，实测 p99 滑移恰为
  // 10.0°，best 逐帧 1:1 跟随 raw 噪声，且 ±10° 系统偏差喂给 EKF 造成左右抖。
  // 现在无死区：尖锐面上拉力被曲率抵消（残余 <1°），谷地里 argmin 直接落在
  // raw（roll 免疫），不存在边界可骑。
  auto slip = std::abs(tools::limit_rad(best_yaw - armor.yaw_raw));
  auto drop = e_raw - std::min(
    armor_reprojection_error(armor, tools::limit_rad(armor.yaw_raw + 15.0 * CV_PI / 180.0), 0.0),
    armor_reprojection_error(armor, tools::limit_rad(armor.yaw_raw - 15.0 * CV_PI / 180.0), 0.0));

  // 联调门控量下传：Target 创新门控用滑移/平坦度对噪声帧降权（2026-10-14 近距
  // 抖动排查：0.9~1.5m 浅谷区 dp<DROP_MIN 不拉 raw，自由搜索的 best 是角点噪声
  // 游走，sl 实测最大 59.8°，这类帧的角度测量不可信）。
  armor.yaw_slip = slip;
  armor.yaw_drop = drop;
  // 板图像中心（四角点均值）相对光轴的偏移角正切：板越靠画面边缘，IPPE 双解
  // 逐帧翻转越严重（近距抖动放大链的噪声源）。tan30° ≈ 0.58，Target 侧超此值
  // 降权角度通道。
  const auto ptc =
    (armor.points[0] + armor.points[1] + armor.points[2] + armor.points[3]) * 0.25;
  const auto cx = camera_matrix_.at<double>(0, 2);
  const auto cy = camera_matrix_.at<double>(1, 2);
  const auto fx = camera_matrix_.at<double>(0, 0);
  const auto fy = camera_matrix_.at<double>(1, 1);
  armor.edge_off = std::sqrt(
    (ptc.x - cx) * (ptc.x - cx) / (fx * fx) + (ptc.y - cy) * (ptc.y - cy) / (fy * fy));
  constexpr double SLIP_MIN = 8.0 * CV_PI / 180.0;   // rad
  constexpr double DROP_MIN = 60.0;                  // px，谷地必须够深才是 roll 谷
  constexpr double DROP_MAX = 300.0;                 // px
  constexpr double PULL_D_MAX = 1.8;                 // m，超过不拉
  constexpr double RAW_PULL_PER_RAD = 2400.0;        // px/rad ≈ 42px/°
  // DROP_MIN 的依据（2026-10-13 20:28 联调实测）：roll 谷地的 drop 随距离急剧变浅
  // ——0.4~0.7m 为 100~204px（真 roll），0.9~1.2m 只剩 15~51px。远距的 slip 大
  // （28~45°）但误差面是平坦的噪声地板：best 偏离 raw 不是 roll 吸收，而是角点
  // 噪声让 argmin 在浅谷里游走。此时拉力把 best 钉在 raw 上 = 把 raw 的白噪声
  // （逐帧 ±10~25°）1:1 注入 EKF 角度通道 → w 拟合出 ±3rad/s 的虚假自转 → 越过
  // 瞄准器 2rad/s 的小陀螺阈值 → 选板逻辑在 普通/小陀螺/无解 间反复横跳 →
  // 指令 yaw 大幅跳变（实测 60°↔94°↔-1.4°）= 远距左右晃动幅度比 j1 更大的根因
  // （j1 时轨道被 diverged 误杀，指令为 0，云台反而安静）。没有深谷就不拉：
  // 远距回到自由搜索（f_gate_only 的远距行为，历史实测最好），自由 best 的滑移
  // 被创新门控（20°）拦下，位置通道由装甲板中心支撑，不随 yaw 噪声抖动。
  auto pull_k =
    (slip > SLIP_MIN && drop > DROP_MIN && drop < DROP_MAX &&
     armor.ypd_in_world[2] < PULL_D_MAX) ?
    RAW_PULL_PER_RAD : 0.0;

  // 第二遍：门控成立（roll 谷地）时，用无死区拉力把候选拉向 roll 免疫的 raw。
  // 注意 min_error 必须重置——第一遍 incumbent 不带拉力项，若沿用会凭
  // 4~111px 的谷地误差优势压过 raw（raw 拉力为 0），滑移不被纠正。
  if (pull_k > 50.0) {
    min_error = 1e10;
    for (int i = 0; i < SEARCH_RANGE; i++) {
      double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
      auto error =
        armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0, pull_k);
      if (error < min_error) {
        min_error = error;
        best_yaw = yaw;
      }
    }
  }

  armor.ypr_in_world[0] = best_yaw;
  // 联调日志：yaw_raw 为 solvePnP/IPPE 原始解，best_yaw 为重投影搜索修正值。
  // sl/dp/k 为 roll 滑移门控量：sl=自由滑移角、dp=raw 附近误差面平坦度、
  // k=第二遍拉力系数（0 表示门控未触发）。
  // as 为左右灯条高度不对称度（近端灯条更高，>+0.03 时透视校验生效），
  // 用于观察盆地判别与翻转抑制是否按预期工作。
  auto det_h_left = cv::norm(armor.points[0] - armor.points[3]);
  auto det_h_right = cv::norm(armor.points[1] - armor.points[2]);
  auto as = (det_h_left - det_h_right) / std::max(1.0, std::min(det_h_left, det_h_right));
  tools::logger()->debug(
    "[Solver] raw={:.1f}° best={:.1f}° d={:.2f}m z={:.2f}m as={:+.2f} sl={:.1f}° dp={:.0f} k={:.0f}",
    armor.yaw_raw * 57.3, best_yaw * 57.3, armor.ypd_in_world[2], armor.xyz_in_world[2], as,
    slip * 57.3, drop, pull_k);
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined, double raw_pull_k) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  // 透视不对称性校验：装甲板斜对相机时，近端灯条在图像中更高。矩形姿态的
  // 180° 镜像解重投影误差几乎相同（误差判据本身区分不了两个盆地），但镜像解
  // 左右灯条对调、高度差符号必然相反 —— 用检测角点自带的几何信息剔除镜像
  // 盆地候选，无需任何时间先验，也就不会发生"先验锁死在镜像盆地"。
  // 角点顺序（yolo11.cpp）：points[0][3]=左灯条、points[1][2]=右灯条。
  auto det_h_left = cv::norm(armor.points[0] - armor.points[3]);
  auto det_h_right = cv::norm(armor.points[1] - armor.points[2]);
  auto det_asym = det_h_left - det_h_right;

  auto proj_h_left = cv::norm(image_points[0] - image_points[3]);
  auto proj_h_right = cv::norm(image_points[1] - image_points[2]);
  auto proj_asym = proj_h_left - proj_h_right;

  // 仅当检测到的不对称足够显著时才生效：正对相机时两盆地几乎重合，符号是
  // 角点噪声，强制判盆地反而会引入错误。罚值取 50px 下限 + 4 倍不对称量，
  // 确保压过角点噪声级的误差差异。
  auto scale = std::min(det_h_left, det_h_right);
  if (std::abs(det_asym) > 0.03 * scale && det_asym * proj_asym < 0)
    error += std::max(50.0, 4 * std::abs(det_asym));

  // roll 拉力（仅在 optimize_yaw 的门控判定通过后传入 raw_pull_k > 0）：
  // 手持装甲板绕视轴的 roll 会被"仅 yaw"的模板搜索吸收进 yaw，使假盆地
  // （roll≈45° 时偏离真实朝向 45-57°）以约 400px 的误差优势赢过真盆地
  // （仿真实测；联调日志 raw≈64° best≈14° 即此现象，导致瞄准漂移、云台扫走
  // 目标丢失）。IPPE 原始解 yaw_raw 的 z-y-x 欧拉 yaw 对 roll 免疫且恒≈模板
  // yaw（同一平面姿态 Rz·Ry·Rx 的欧拉 yaw 与 roll 无关，恒等于朝向角），
  // 故谷地内把候选拉向 yaw_raw 即可压掉滑移。
  // 注意：无死区。2026-10-12 晚版本带 10° 死区，拉力把 argmin 钉在死区边界
  // best=raw±10°（实测 p99 滑移恰为 10.0°），逐帧 1:1 跟随 raw 噪声、注入
  // ±10° 系统偏差。无死区时尖锐误差面上拉力被曲率抵消（仿真实测残余 <1°），
  // 谷地里 argmin 直接落在 raw，不存在边界可骑。
  if (raw_pull_k > 0.0)
    error += raw_pull_k * std::abs(tools::limit_rad(yaw - armor.yaw_raw));

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim