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
  PyCenterPoint(const std::string& config_path, const std::string& model_path);
  ~PyCenterPoint() = default;

  std::vector<Box> forward(PyArray<float> np_array);

private:
  std::shared_ptr<CenterPoint> m_centerpoint;
  YAML::Node m_config;
};

#endif // CEMTERPOINT_WRAPPER_HPP