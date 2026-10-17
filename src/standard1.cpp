#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

#include "io/camera.hpp"
#include "io/cboard1.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard1 cboard("/dev/ttyACM0");
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  auto last_crc_log = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, t);
    cboard.update();
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    // 每秒显示一次串口 CRC 正确率（最近 5s 滑动窗口 + 累计）
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_crc_log).count() >= 1.0) {
      double rate = cboard.crc_rate();
      if (rate < 0.0) {
        tools::logger()->info("[CRC] no frame in last 5s, lifetime good/total = {}/{}",
                              cboard.crc_good(), cboard.crc_total());
      } else {
        tools::logger()->info("[CRC] rate = {:.1f}% (last 5s), lifetime good/total = {}/{}",
                              rate, cboard.crc_good(), cboard.crc_total());
      }
      last_crc_log = now;
    }

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[static_cast<int>(mode)]);
      last_mode = mode;
    }

    solver.set_R_gimbal2world(q);

    auto armors = detector.detect(img);
    tools::logger()->info("armors detected: {}", armors.size());
    auto targets = tracker.track(armors, t);

    // 使用 Planner（轨迹规划器），需要传入单个 Target
    std::optional<auto_aim::Target> target;
    if (!targets.empty()) {
      target = targets.front();
    }
    auto plan = planner.plan(target, cboard.bullet_speed);

    // 将 Plan 转换为 Command
    io::Command command;
    command.control = plan.control;
    command.shoot = plan.fire;
    command.yaw = plan.yaw;
    command.pitch = plan.pitch;
    command.tracking = plan.control;

    cboard.send(command);
  }

  return 0;
}
