#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug = false);

  // 退出时可能还有请求在跑（已经 start_async 提交、但没人 pop 取结果）。
  // 不把它们等干净就直接析构，InferRequest 会在推理进行中被销毁。
  ~MultiThreadDetector();

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();  //暂时不支持yolov8

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

  // 有没有已经提交、还没取走的结果。pop()/debug_pop() 是阻塞的，
  // 采集线程如果先挂了（相机出错），消费线程会永远等在那里 —— 调用方
  // 用它判断"采集已经停了而且没有遗留结果"，好干净地退出。
  bool has_pending() { return !pending_.empty(); }

private:
  // 一个推理槽位：请求 + 它自己的输入缓冲 + 这一帧的图像和时间戳。
  //
  // 请求和输入缓冲都只建一次、反复用。原来每帧 create_infer_request()，
  // 实测这样 OpenVINO 走的是"单请求延迟"路径（44~57ms/帧），
  // 而不是"多流吞吐"路径（21~26ms/帧）—— 这就是帧率上不去的主因。
  struct Slot
  {
    ov::InferRequest request;
    ov::Tensor input;  // 自己持有，生命周期跟着 Slot，不受 push() 作用域影响
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
  };

  YAML::Node cfg_;  // 必须声明在 pool_/free_/pending_ 之前：长度由它决定
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::string device_;
  std::string perf_mode_;
  int streams_ = 1;
  YOLO yolo_;

  // 等 slot 上的推理结束、做后处理，然后把 slot 还回 free_
  std::list<Armor> finish(int slot);

  std::vector<Slot> pool_;
  tools::ThreadSafeQueue<int> free_;     // 空闲槽位
  tools::ThreadSafeQueue<int> pending_;  // 已提交、等待取结果，FIFO 保证按提交顺序消费

  std::atomic<int> dropped_{0};
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP