#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "robot_transmission/model.hpp"
namespace py = pybind11;
namespace rt = robot_transmission;
PYBIND11_MODULE(_core, m) {
  py::class_<rt::Axis>(m,"Axis")
    .def(py::init<double,double,double>(),py::arg("position")=0.0,py::arg("velocity")=0.0,py::arg("effort")=0.0)
    .def_readwrite("position",&rt::Axis::position).def_readwrite("velocity",&rt::Axis::velocity).def_readwrite("effort",&rt::Axis::effort);
  py::class_<rt::Group>(m,"Group")
    .def_readonly("name",&rt::Group::name).def_readonly("plugin",&rt::Group::plugin)
    .def_readonly("actuators",&rt::Group::actuators).def_readonly("joints",&rt::Group::joints)
    .def_readonly("actuator_reduction",&rt::Group::actuator_reduction)
    .def_readonly("joint_reduction",&rt::Group::joint_reduction).def_readonly("joint_offset",&rt::Group::joint_offset)
    .def_readonly("parameters",&rt::Group::parameters)
    .def_readonly("position_dependent",&rt::Group::position_dependent);
  py::class_<rt::Model>(m,"Model")
    .def_static("load",&rt::Model::load,py::arg("driver_file"),py::arg("transmissions_file")="")
    .def_property_readonly("actuator_names",&rt::Model::actuator_names)
    .def_property_readonly("joint_names",&rt::Model::joint_names)
    .def_property_readonly("actuator_ids",&rt::Model::actuator_ids)
    .def_property_readonly("groups",&rt::Model::groups)
    .def("actuator_to_joint",&rt::Model::actuator_to_joint)
    .def("joint_to_actuator",&rt::Model::joint_to_actuator)
    .def("joint_effort_to_actuator",&rt::Model::joint_effort_to_actuator,
         py::arg("effort"),py::arg("joint_positions")=std::map<std::string,double>{})
    .def("actuator_effort_to_joint",&rt::Model::actuator_effort_to_joint,
         py::arg("effort"),py::arg("actuator_positions")=std::map<std::string,double>{})
    .def("actuators_for_joint",&rt::Model::actuators_for_joint);
}
