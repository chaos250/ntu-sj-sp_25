#include "commandgener.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();
  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      // 帧龄上限 0.35s：6 流管线实测滞后 ~159ms（见 configs/standard3.yaml），
      // 原 0.2s 只剩 ~40ms 裕量，推理一抖动就超限，停止帧与瞄准帧交替下发
      // 导致云台抽搐；放宽到 0.35s 同时保留"检测停摆后不发过期指令"的保护。
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.35) {
        input = latest_;
      } else
        input = std::nullopt;
    }
    io::Command command;
    if (input) {
      // to_now=true：以当前时刻为基准，帧滞后（now - t）由 aimer 计入预测
      command = aimer_.aim(input->targets_, input->t, input->bullet_speed, true);
      command.shoot = shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);
      command.horizon_distance = input->targets_.empty()
                                   ? 0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +
                                       tools::square(input->targets_.front().ekf_x()[2]));
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["shoot"] = command.shoot;
        data["horizon_distance"] = command.horizon_distance;
        plotter_.plot(data);
      }
    } else {
      // 无新帧或帧龄超限：发停止指令，避免电控一直执行过期的瞄准指令
      command = io::Command{false, false, 0, 0};
    }
    send_(command);

    // 1Hz 发送数据日志（联调排查跳变用，转成度便于和电控对表）
    static auto last_tx_log = std::chrono::steady_clock::time_point{};
    if (tools::delta_time(std::chrono::steady_clock::now(), last_tx_log) > 1.0) {
      tools::logger()->info(
        "[CommandGener] mode={} yaw={:+.2f}° pitch={:+.2f}° shoot={}",
        command.control ? (command.shoot ? 2 : 1) : 0, command.yaw * 57.3, command.pitch * 57.3,
        command.shoot);
      last_tx_log = std::chrono::steady_clock::now();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  //approximately 500Hz
  }
}

}  // namespace multithread

}  // namespace auto_aim
