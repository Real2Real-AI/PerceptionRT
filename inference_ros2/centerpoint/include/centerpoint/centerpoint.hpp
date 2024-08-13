#ifndef _CENTERPOINT_HPP_
#define _CENTERPOINT_HPP_

#include <string>
#include <algorithm>
#include <random>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include <centerpoint/voxelization.hpp>
#include <centerpoint/network.hpp>
#include <centerpoint/postprocess.hpp>

class CenterPoint
{
public:
  CenterPoint(const YAML::Node& config, const std::string& model_path);
  ~CenterPoint();
  void forward(size_t num_points, float* input_points);
  std::vector<Box>* getBoxesPointer() { return boxes_; }

private:
  void memoryInit(const std::string& model_path);


private:
  YAML::Node config_;

  // CenterPoint Pipeline
  std::shared_ptr<Voxelization> voxelization_ = nullptr;
  std::shared_ptr<Network> network_           = nullptr;
  std::shared_ptr<PostProcess> postprocess_   = nullptr;

  // Point Cloud Input
  std::vector<float> points_;
  size_t max_points;
  float* input_points_ = nullptr;
  float* dev_input_points_ = nullptr;

  // CUDA Stream
  cudaStream_t stream_;

  // Output
  std::vector<Box>* boxes_;


};

#endif // _CENTERPOINT_HPP_