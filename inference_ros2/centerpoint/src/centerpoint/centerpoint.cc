#include "centerpoint/centerpoint.hpp"

CenterPoint::CenterPoint(const YAML::Node& config, const std::string& model_path) : config_(config)
{
  checkRuntime(cudaStreamCreate(&stream_));
  memoryInit(model_path);
}

CenterPoint::~CenterPoint()
{
  checkRuntime(cudaStreamDestroy(stream_));
  checkRuntime(cudaFree(dev_input_points_));
}

void CenterPoint::memoryInit(const std::string& model_path)
{
  max_points = config_["centerpoint"]["max_points"].as<size_t>();
  size_t max_points_feature = max_points * config_["voxelization"]["num_feature"].as<size_t>();
  points_.reserve(max_points_feature);
  size_t bytes_points_capacity = max_points_feature * sizeof(float);
  checkRuntime(cudaMalloc(&dev_input_points_, bytes_points_capacity));
  checkRuntime(cudaDeviceSynchronize());

  voxelization_ = std::make_shared<Voxelization>(config_["voxelization"]);
  checkRuntime(cudaDeviceSynchronize());

  network_ = std::make_shared<Network>(model_path);
  checkRuntime(cudaDeviceSynchronize());

  postprocess_ = std::make_shared<PostProcess>(config_["postprocess"]);
  checkRuntime(cudaDeviceSynchronize());
}

void CenterPoint::forward(size_t num_points, float* input_points)
{
  input_points_ = input_points;
  if (num_points > max_points) {
    std::cout << "Max Points Over: " << num_points << std::endl;
    num_points = max_points;
  }
  size_t bytes_points = num_points * voxelization_->param().num_feature * sizeof(float);
  checkRuntime(cudaMemcpyAsync(dev_input_points_, input_points_, bytes_points, cudaMemcpyHostToDevice, stream_));

  voxelization_->forward(dev_input_points_, num_points, stream_);

  network_->forward(voxelization_->features(), voxelization_->coords(), voxelization_->nums(), stream_);

  int box_num = postprocess_->forward(network_->center(), network_->center_z(), network_->dim(), network_->rot(),
                                      network_->score(), network_->label(), network_->iou(), stream_);

  boxes_ = postprocess_->getBoxesPointer();
  checkRuntime(cudaStreamSynchronize(stream_));
}