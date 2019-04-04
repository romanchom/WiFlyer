#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct Telemetry {
    Eigen::Vector3f acceleration;
    Eigen::Vector3f angularVelocity;
    Eigen::Vector3f magneticField;
    float pressure;
    float temperature;
    Eigen::Quaternion<float> orientation;
    Eigen::Vector3f angularMomentum;
    Eigen::Vector3f position;
};
