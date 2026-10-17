#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
// 顺序必须和头文件里的成员声明顺序一致：queue_ 的长度要用到 streams_
: cfg_(YAML::LoadFile(config_path)),
  // 有些配置（calibration/camera）没有 device 键，缺了要退回 CPU 而不是抛异常
  device_(cfg_["device"] ? cfg_["device"].as<std::string>() : "CPU"),
  perf_mode_(
    cfg_["performance_mode"] ? cfg_["performance_mode"].as<std::string>() : "throughput"),
  streams_(cfg_["num_streams"] ? cfg_["num_streams"].as<int>() : 4),
  yolo_(config_path, debug),
  // 槽位数就是并发上限，两个表都按它开
  free_(static_cast<std::size_t>(std::max(1, streams_))),
  pending_(static_cast<std::size_t>(std::max(1, streams_)))
{
  auto yolo_name = cfg_["yolo_name"].as<std::string>();
  auto model_path = cfg_[yolo_name + "_model_path"].as<std::string>();

  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})  // TODO
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    .scale(255.0);

  model = ppp.build();

  // 原来这里写死了 "CPU" 和 LATENCY 模式，配置里的 device 根本没生效。
  //
  // 延迟模式让单个请求独占资源，实测在这台 8 核机器上只跑出 61ms/帧；
  // 吞吐模式 + 多流能让多个请求并行填满多核，吞吐能到 ~20ms/帧（约 3 倍）。
  // 生产者线程是异步提交的，队列里天然会有多帧同时在推理，所以能吃满这些流。
  ov::AnyMap props{ov::hint::performance_mode(
    perf_mode_ == "latency" ? ov::hint::PerformanceMode::LATENCY
                            : ov::hint::PerformanceMode::THROUGHPUT)};
  if (streams_ > 1) props[ov::hint::num_requests.name()] = streams_;

  compiled_model_ = core_.compile_model(model, device_, props);

  // 请求池：一次性建好 streams_ 个请求和各自的输入缓冲，之后反复复用。
  pool_.resize(streams_);
  for (int i = 0; i < streams_; ++i) {
    pool_[i].request = compiled_model_.create_infer_request();
    pool_[i].input = ov::Tensor(ov::element::u8, {1, 640, 640, 3});
    pool_[i].request.set_input_tensor(pool_[i].input);  // 缓冲固定，绑一次就够
  }
  for (int i = 0; i < streams_; ++i) free_.push(i);

  tools::logger()->info(
    "[MultiThreadDetector] device={} 模式={} 流数={} 队列长度={}", device_, perf_mode_, streams_,
    pending_.max_size());
}

MultiThreadDetector::~MultiThreadDetector()
{
  for (auto & s : pool_) {
    if (!s.request) continue;
    try {
      s.request.wait();
    } catch (const std::exception & e) {
      tools::logger()->warn("[MultiThreadDetector] 退出时等待推理失败: {}", e.what());
    }
  }
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  // 相机出图（约 95fps）远快于推理，注定要丢掉大部分帧，这里丢掉的是新帧。
  //
  // 试过"丢掉最旧的在飞帧、让新帧顶进去"，结果更差（3 流 20.5 -> 10.6 fps）：
  // 生产者要等旧请求结束才能覆写槽位，把采集线程一起拖住了。
  //
  // 而且 FIFO 流水线的滞后本来就是 流数 x 单帧周期 —— 想让帧变新就得等
  // 最后一个提交的请求，等得更久。帧率和滞后是直接对立的，只能选工作点，
  // 靠丢帧绕不过去。见 configs 里 num_streams 的说明。
  int slot;
  if (!free_.try_pop(slot)) {
    if (++dropped_ % 300 == 0)
      tools::logger()->info(
        "[MultiThreadDetector] 推理跟不上相机，累计丢弃 {} 帧", dropped_.load());
    return;
  }

  auto & s = pool_[slot];

  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces：直接写进槽位自己的输入缓冲
  auto input = cv::Mat(640, 640, CV_8UC3, s.input.data());
  input.setTo(cv::Scalar(0, 0, 0));  // letterbox 补边
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, input(roi), {w, h});

  s.img = img.clone();
  s.t = t;
  s.request.start_async();

  // 槽位数和 pending_ 容量相同，取到槽位就一定能塞进去；这里仍按
  // "塞不进就把槽位还回去"处理，避免任何情况下漏掉槽位把流水线卡死。
  if (!pending_.try_push(slot)) free_.push(slot);
}

std::list<Armor> MultiThreadDetector::finish(int slot)
{
  auto & s = pool_[slot];
  s.request.wait();

  auto output_tensor = s.request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / s.img.rows;
  auto y_scale = static_cast<double>(640) / s.img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, s.img, 0);  //暂不支持ROI

  // 后处理做完才能把槽位交出去，否则生产者会覆盖输出张量
  free_.push(slot);
  return armors;
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  int slot = pending_.pop();
  auto t = pool_[slot].t;
  return {finish(slot), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  int slot = pending_.pop();
  // 先取出来：finish() 会把槽位交还，之后 s.img 可能被生产者改写
  auto img = pool_[slot].img;
  auto t = pool_[slot].t;
  return {std::move(img), finish(slot), t};
}

}  // namespace multithread

}  // namespace auto_aim
