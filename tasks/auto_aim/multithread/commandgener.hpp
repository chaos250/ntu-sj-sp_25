#ifndef AUTO_AIM_MULTITHREAD__HPP
#define AUTO_AIM_MULTITHREAD__HPP

#include <functional>
#include <optional>

#include "io/command.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/plotter.hpp"

namespace auto_aim
{
namespace multithread
{

class CommandGener
{
public:
  // board 可以是任何提供 send(const io::Command &) 的下位机对象
  // （io::CBoard 走 SocketCAN，io::Gimbal 走 USB CDC 虚拟串口）。
  // 注意：board 的生命周期必须长于本对象。
  template <typename BoardT>
  CommandGener(
    auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, BoardT & board,
    tools::Plotter & plotter, bool debug = false)
  : shooter_(shooter),
    aimer_(aimer),
    plotter_(plotter),
    send_([&board](const io::Command & command) { board.send(command); }),
    stop_(false),
    debug_(debug)
  {
    thread_ = std::thread(&CommandGener::generate_command, this);
  }

  ~CommandGener();

  void push(
    const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
    double bullet_speed, const Eigen::Vector3d & gimbal_pos);

private:
  struct Input
  {
    std::list<auto_aim::Target> targets_;
    std::chrono::steady_clock::time_point t;
    // std::function<void()> decide;
    double bullet_speed;
    Eigen::Vector3d gimbal_pos;
  };

  auto_aim::Shooter & shooter_;
  auto_aim::Aimer & aimer_;
  tools::Plotter & plotter_;
  std::function<void(const io::Command &)> send_;

  std::optional<Input> latest_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread thread_;
  bool stop_, debug_;

  void generate_command();
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM_MULTITHREAD__HPP
