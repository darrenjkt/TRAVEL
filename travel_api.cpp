#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "tgs.hpp"

namespace py = pybind11;

struct PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t label;                     ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;


PYBIND11_MODULE(travel, m) {
    py::class_<travel::TravelGroundSeg<PointXYZILID>>(m, "TravelGroundSeg")
        .def(py::init<>())
        .def("setParams", &travel::TravelGroundSeg<PointXYZILID>::setParams)
        .def("estimateGround", &travel::TravelGroundSeg<PointXYZILID>::estimateGround);

    // m.def("fromArray", &fromArray, "Convert numpy array to pcl");
}