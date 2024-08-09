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

#include <algorithm>
#include <string>

namespace gazebo_plugins {
namespace vehicle_plugins {

struct State {
    State operator*(const double &dt) const {
        return {dt * x,   dt * y,   dt * z,   dt * yaw, dt * v_x, dt * v_y, dt * v_z,
                dt * r_x, dt * r_y, dt * r_z, dt * a_x, dt * a_y, dt * a_z};
    }

    State operator+(const State &x2) const {
        return {x + x2.x,     y + x2.y,     z + x2.z,     yaw + x2.yaw, v_x + x2.v_x, v_y + x2.v_y, v_z + x2.v_z,
                r_x + x2.r_x, r_y + x2.r_y, r_z + x2.r_z, a_x + x2.a_x, a_y + x2.a_y, a_z + x2.a_z};
    }

    double x = 0;
    double y = 0;
    double z = 0;
    double yaw = 0;
    double v_x = 0;
    double v_y = 0;
    double v_z = 0;
    double r_x = 0;
    double r_y = 0;
    double r_z = 0;
    double a_x = 0;
    double a_y = 0;
    double a_z = 0;
};

struct Control {
    double acceleration;
    double velocity;
    double steering;
};

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins
