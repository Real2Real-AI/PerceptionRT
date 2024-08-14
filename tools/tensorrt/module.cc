#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "centerpoint/wrapper.hpp"

namespace py = pybind11;
template <typename T>
using PyArray = py::array_t<T, py::array::c_style | py::array::forcecast>;

PYBIND11_MODULE(pycenterpoint, m) {
  py::class_<PyCenterPoint>(m, "PyCenterPoint")
    .def(py::init<const std::string&, const std::string&>())
    .def("forward", &PyCenterPoint::forward);
  py::class_<Box>(m, "Box")
  .def("x", &Box::x)
  .def("y", &Box::y)
  .def("z", &Box::z)
  .def("l", &Box::l)
  .def("w", &Box::w)
  .def("h", &Box::h)
  .def("yaw", &Box::yaw)
  .def("score", &Box::score)
  .def("cls", &Box::cls);

}