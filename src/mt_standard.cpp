#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"  // USB CDC 虚拟串口 (SP 协议, com_port: /dev/gimbal)
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);  // 串口路径来自配置项 com_port (默认 /dev/gimbal)

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);  // true: 弹检测可视化窗口
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  // 注意：gimbal 必须在此声明之前构造，CommandGener 持有它的引用
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, gimbal, plotter, true);

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};
  auto last_crc_log = std::chrono::steady_clock::now();

    auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      if (mode.load() == io::GimbalMode::AUTO_AIM) {
        camera.read(img, t);
        detector.push(img, t);
      } else
        std::this_thread::sleep_for(5ms);  // 非自瞄模式不忙等，避免白占 CPU 抢推理资源
    }
  });

  while (!exiter.exit()) {
    // Gimbal 自带接收线程，无需像串口方案那样轮询 update()
    mode = gimbal.mode();

    // 每秒显示一次串口 CRC 正确率（最近 5s 滑动窗口 + 累计），各模式下都会打印
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_crc_log).count() >= 1.0) {
      if (!gimbal.crc_check_enabled()) {
        tools::logger()->info("[CRC] crc_check 已关闭，无统计（请把配置项 crc_check 改回 true）");
      } else {
        double rate = gimbal.crc_rate();
        if (rate < 0.0) {
          tools::logger()->info("[CRC] no frame in last 5s, lifetime good/total = {}/{}",
                                gimbal.crc_good(), gimbal.crc_total());
        } else {
          tools::logger()->info("[CRC] rate = {:.1f}% (last 5s), lifetime good/total = {}/{}",
                                rate, gimbal.crc_good(), gimbal.crc_total());
        }
      }
      last_crc_log = now;
    }

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode.load()));
      last_mode = mode.load();
      // 切出自瞄时立刻清空决策线程的最新帧：否则 commandgener 会继续用旧目标
      // 发 ~0.2s 自瞄指令，与打符指令冲突导致云台抖动
      if (mode.load() != io::GimbalMode::AUTO_AIM)
        commandgener.push({}, std::chrono::steady_clock::now(), 0, Eigen::Vector3d::Zero());
    }

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      auto [img, armors, t] = detector.debug_pop();
      // 注意：t 来自检测线程读到的图像时间（已滞后一个推理周期），
      // Gimbal::q() 会用最近两帧 IMU 反向外推到该时刻
      Eigen::Quaterniond q = gimbal.q(t - 1ms);

      // recorder.record(img, q, t);

      solver.set_R_gimbal2world(q);

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      auto targets = tracker.track(armors, t);

      commandgener.push(targets, t, gimbal.state().bullet_speed, ypr);  // 发送给决策线程

    }

    /// 打符
    else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      cv::Mat img;
      Eigen::Quaterniond q;
      std::chrono::steady_clock::time_point t;

      camera.read(img, t);
      q = gimbal.q(t - 1ms);

      // recorder.record(img, q, t);

      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);

      buff_solver.solve(power_runes);

      io::Command buff_command;
      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, gimbal.state().bullet_speed, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, gimbal.state().bullet_speed, true);
      }
      gimbal.send(buff_command);

    } else {
      std::this_thread::sleep_for(2ms);  // IDLE 等非自瞄模式不忙等
      continue;
    }
  }

  detect_thread.join();

  return 0;
}
