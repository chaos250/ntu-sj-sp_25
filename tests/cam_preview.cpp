// 相机实时预览 + 检测框 + 帧率显示。
//
// 联调时用来一边看画面一边摆装甲板，同时判断帧率是否够用。
//
// 三个角色分开跑，否则量出来的帧率不是自瞄的帧率：
//   采集线程  camera.read -> detector.push
//   推理线程  detector.debug_pop -> 发布最新一帧
//   主线程    取最新一帧 -> 画框 -> imshow
//
// 为什么要拆：imshow 在虚拟机的软件渲染下要十几毫秒。如果推理和渲染在同一个
// 线程里，窗口就变成了瓶颈，显示出来的"处理帧率"会比 mt_standard 实际能拿到的
// 低一大截（实测 19.5 vs 31.6 fps），而且渲染期间结果在队列里越堆越旧，滞后
// 从 214ms 涨到 401ms。渲染只取最新一帧，慢就慢在画面上，不拖累自瞄。
//
// 用法: cam_preview [config.yaml]
//   q / ESC / Ctrl-C 退出

#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "io/camera.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明 }"
  "{@config-path   | configs/standard3.yaml | yaml配置文件的路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;
  io::Camera camera(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path);

  const std::string win = "cam_preview  (q / ESC / Ctrl-C to quit)";
  cv::namedWindow(win, cv::WINDOW_NORMAL);
  cv::resizeWindow(win, 1024, 820);

  // 采集线程是否还在跑。相机出错时它自己置 false，推理线程靠它避免死等。
  std::atomic<bool> grabbing{true};
  std::atomic<bool> running{true};
  // 相机真实出图帧率由采集线程统计（推理丢帧不影响它）
  std::atomic<double> cam_fps{0};
  // 流水线真实处理帧率由推理线程统计，不含渲染开销
  std::atomic<double> proc_fps{0};

  // 推理线程发布给渲染线程的"最新一帧"。渲染慢就一直被覆盖，不排队。
  std::mutex mtx;
  cv::Mat latest_img;
  std::list<auto_aim::Armor> latest_armors;
  std::chrono::steady_clock::time_point latest_t{};
  bool has_latest = false;

  std::thread grabber([&] {
    cv::Mat raw;
    std::chrono::steady_clock::time_point t_raw, t_prev{};
    bool first = true;
    double fps = 0;
    while (!exiter.exit() && running.load(std::memory_order_relaxed)) {
      camera.read(raw, t_raw);
      if (raw.empty()) break;

      if (!first) {
        double d = tools::delta_time(t_raw, t_prev);
        if (d > 1e-6) {
          double inst = 1.0 / d;
          fps = fps <= 0 ? inst : fps * 0.9 + inst * 0.1;
          cam_fps.store(fps, std::memory_order_relaxed);
        }
      }
      t_prev = t_raw;
      first = false;

      detector.push(raw, t_raw);
    }
    // 无论是正常退出还是相机读失败退出，都要通知推理线程
    grabbing.store(false, std::memory_order_relaxed);
  });

  std::thread inferer([&] {
    std::deque<std::chrono::steady_clock::time_point> stamps;
    while (!exiter.exit() && running.load(std::memory_order_relaxed)) {
      // 采集线程要是先退出了（相机报错），别死等一个永远不来的结果
      if (!grabbing.load(std::memory_order_relaxed) && !detector.has_pending()) break;

      auto [img, armors, t] = detector.debug_pop();
      if (img.empty()) break;

      auto now = std::chrono::steady_clock::now();
      stamps.push_back(now);
      while (!stamps.empty() &&
             std::chrono::duration<double>(now - stamps.front()).count() > 0.5)
        stamps.pop_front();
      if (stamps.size() >= 2) {
        double span = std::chrono::duration<double>(stamps.back() - stamps.front()).count();
        if (span > 0) proc_fps.store((stamps.size() - 1) / span, std::memory_order_relaxed);
      }

      {
        std::lock_guard<std::mutex> lock(mtx);
        latest_img = std::move(img);
        latest_armors = std::move(armors);
        latest_t = t;
        has_latest = true;
      }
    }
  });

  while (!exiter.exit()) {
    cv::Mat vis;
    std::list<auto_aim::Armor> armors;
    std::chrono::steady_clock::time_point t_frame;
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (!has_latest) {
        // 还没有第一帧，让出 CPU 等一会儿
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      vis = latest_img;  // 共享缓冲，下面是复制一份再画
      armors = latest_armors;
      t_frame = latest_t;
    }

    // 这一帧有多旧：从曝光到画出来。自瞄靠它判断控制滞后。
    double age_ms = tools::delta_time(std::chrono::steady_clock::now(), t_frame) * 1000;

    // --- 画检测框 ---
    vis = vis.clone();  // 画的这一份是发布出来的共享图，不能就地改
    for (const auto & a : armors) {
      if (a.points.size() < 2) continue;
      std::vector<cv::Point> pts;
      pts.reserve(a.points.size());
      for (const auto & p : a.points) pts.emplace_back(cvRound(p.x), cvRound(p.y));
      const cv::Point * ppt[1] = {pts.data()};
      int npt[1] = {static_cast<int>(pts.size())};
      cv::polylines(vis, ppt, npt, 1, true, {0, 255, 0}, 3);

      cv::putText(
        vis, auto_aim::COLORS[a.color] + "/" + auto_aim::ARMOR_NAMES[a.name],
        pts[0] + cv::Point(0, -12), cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
    }

    // --- 左上角帧率面板 ---
    cv::rectangle(vis, {10, 10}, {470, 215}, {0, 0, 0}, cv::FILLED);  // 深色底衬
    cv::putText(
      vis, fmt::format("处理帧率: {:5.1f} fps", proc_fps.load(std::memory_order_relaxed)),
      {25, 55}, cv::FONT_HERSHEY_SIMPLEX, 1.3, {0, 255, 255}, 3);
    cv::putText(
      vis, fmt::format("相机帧率: {:5.1f} fps", cam_fps.load(std::memory_order_relaxed)), {25, 100},
      cv::FONT_HERSHEY_SIMPLEX, 1.3, {255, 255, 0}, 3);
    cv::putText(
      vis, fmt::format("检测到 {} 个装甲板", armors.size()), {25, 150}, cv::FONT_HERSHEY_SIMPLEX,
      1.3, armors.empty() ? cv::Scalar{128, 128, 128} : cv::Scalar{0, 255, 0}, 3);
    cv::putText(
      vis, fmt::format("本帧滞后: {:5.0f} ms", age_ms), {25, 200}, cv::FONT_HERSHEY_SIMPLEX, 1.3,
      {255, 128, 0}, 3);

    cv::imshow(win, vis);
    auto key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }

  // 先停采集、再停推理，最后才销毁相机和检测器
  running.store(false, std::memory_order_relaxed);
  grabber.join();
  inferer.join();

  cv::destroyAllWindows();
  return 0;
}
