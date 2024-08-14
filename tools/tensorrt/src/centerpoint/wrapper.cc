#include "centerpoint/wrapper.hpp"

PyCenterPoint::PyCenterPoint(const std::string& config_path, const std::string& model_path)
{
  m_config = YAML::LoadFile(config_path);
  m_centerpoint = std::make_shared<CenterPoint>(m_config, model_path);
}

std::vector<Box> PyCenterPoint::forward(PyArray<float> np_array)
{
  size_t num_points = np_array.request().shape[0];
  float* input_point = static_cast<float*>(np_array.request().ptr);
  m_centerpoint->forward(num_points, input_point);
  std::vector<Box> resutls = *(m_centerpoint->getBoxesPointer());

  return resutls;
}