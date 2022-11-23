// cppimport

#include <pybind11/pybind11.h>
#include "tgs.hpp"

namespace py = pybind11;

PYBIND11_MODULE(travel, m) {
    py::class_<TravelGroundSeg>(m, "travel")
        .def(py::init())
        .def("setParams", &TravelGroundSeg::setParams)
        .def("getName", &TravelGroundSeg::estimateGround);
}