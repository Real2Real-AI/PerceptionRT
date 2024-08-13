#ifndef CEMTERPOINT_WRAPPER_HPP
#define CEMTERPOINT_WRAPPER_HPP

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "centerpoint/centerpoint.hpp"

namespace py = pybind11;
template <typename T>
using PyArray = py::array_t<T, py::array::c_style | py::array::forcecast>;

class PyCenterPoint
{
public:
  PyCenterPoint(const std::string& config_path, const std::string& model_path)
  {
    m_config = YAML::LoadFile(config_path);
    m_centerpoint = std::make_shared<CenterPoint>(m_config, model_path);
  }
  ~PyCenterPoint() = default;

  std::vector<Box> forward(PyArray<float> np_array)
  {
    size_t num_points = np_array.request().shape[0];
    float* input_point = static_cast<float*>(np_array.request().ptr);
    m_centerpoint->forward(num_points, input_point);
    std::vector<Box> resutls = *(m_centerpoint->getBoxesPointer());

    return resutls;
  }

private:
  std::shared_ptr<CenterPoint> m_centerpoint;
  YAML::Node m_config;
};

#endif // CEMTERPOINT_WRAPPER_HPP