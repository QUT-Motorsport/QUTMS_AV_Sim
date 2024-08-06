// MIT License

// Copyright (c) 2020 Edinburgh University Formula Student

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <math.h>

#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "gazebo_vehicle_plugin/utils.hpp"

#include "yaml-cpp/yaml.h"

namespace gazebo_plugins {
namespace vehicle_plugins {

struct NoiseParam {
    double position[3];
    double orientation[3];
    double linear_velocity[3];
    double angular_velocity[3];

    std::string array_to_str(const double arr[], int len) {
        std::string message;
        for (int i = 0; i < len; i++) {
            message += std::to_string(arr[i]);
            message += (i < len - 1) ? ", " : "";
        }

        return "[" + message + "]\n";
    }

    std::string to_str() {
        return "position: " + array_to_str(position, 3) + 
               " orientation: " + array_to_str(orientation, 3) +
               " linear_velocity: " + array_to_str(linear_velocity, 3) +
               " angular_velocity: " + array_to_str(angular_velocity, 3);
    }
};

class Noise {
   public:
    explicit Noise(const std::string &yaml_file) {
        YAML::Node config = YAML::LoadFile(yaml_file);
        _noise_param = config["noise"].as<NoiseParam>();
    }

    nav_msgs::msg::Odometry applyNoise(const nav_msgs::msg::Odometry &state) {
        nav_msgs::msg::Odometry new_state = state;

        // Add noise to position
        new_state.pose.pose.position.x += _gaussianKernel(0, _noise_param.position[0]);
        new_state.pose.pose.position.y += _gaussianKernel(0, _noise_param.position[1]);

        // Add noise to orientation
        // to euler
        std::vector<double> euler = to_euler(new_state.pose.pose.orientation);
        // add noise
        euler[2] += _gaussianKernel(0, _noise_param.orientation[2]);
        // to quaternion
        new_state.pose.pose.orientation = to_quaternion(euler);

        // Add noise to linear velocity
        new_state.twist.twist.linear.x += _gaussianKernel(0, _noise_param.linear_velocity[0]);
        new_state.twist.twist.linear.y += _gaussianKernel(0, _noise_param.linear_velocity[1]);

        // Add noise to angular velocity
        new_state.twist.twist.angular.z += _gaussianKernel(0, _noise_param.angular_velocity[2]);

        return new_state;
    }

    const NoiseParam &getNoiseParam() { return _noise_param; }

    std::string getString() { return _noise_param.to_str(); }

   private:
    NoiseParam _noise_param;

    // Initialise seed for pseudo-random number generator
    unsigned seed = 0.0;

    double _gaussianKernel(double mu, double sigma) {
        // using Box-Muller transform to generate two independent standard
        // normally distributed normal variables see wikipedia

        // normalized uniform random variable
        double U = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

        // normalized uniform random variable
        double V = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

        double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
        // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

        // there are 2 indep. vars, we'll just use X
        // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
    }
};

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins

namespace YAML {

template <>
struct convert<gazebo_plugins::vehicle_plugins::NoiseParam> {
    static bool decode(const Node &node, gazebo_plugins::vehicle_plugins::NoiseParam &cType) {
        if (node["positionNoise"]) {
            cType.position[0] = node["positionNoise"][0].as<double>();
            cType.position[1] = node["positionNoise"][1].as<double>();
            cType.position[2] = node["positionNoise"][2].as<double>();
        }

        if (node["orientationNoise"]) {
            cType.orientation[0] = node["orientationNoise"][0].as<double>();
            cType.orientation[1] = node["orientationNoise"][1].as<double>();
            cType.orientation[2] = node["orientationNoise"][2].as<double>();
        }

        if (node["linearVelocityNoise"]) {
            cType.linear_velocity[0] = node["linearVelocityNoise"][0].as<double>();
            cType.linear_velocity[1] = node["linearVelocityNoise"][1].as<double>();
            cType.linear_velocity[2] = node["linearVelocityNoise"][2].as<double>();
        }

        if (node["angularVelocityNoise"]) {
            cType.angular_velocity[0] = node["angularVelocityNoise"][0].as<double>();
            cType.angular_velocity[1] = node["angularVelocityNoise"][1].as<double>();
            cType.angular_velocity[2] = node["angularVelocityNoise"][2].as<double>();
        }

        return true;
    }
};

}  // namespace YAML
