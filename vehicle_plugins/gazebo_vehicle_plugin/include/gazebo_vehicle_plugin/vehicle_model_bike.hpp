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

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"

#include "gazebo_vehicle_plugin/vehicle_param.hpp"
#include "gazebo_vehicle_plugin/vehicle_state.hpp"
#include "gazebo_vehicle_plugin/utils.hpp"

namespace gazebo_plugins {
namespace vehicle_plugins {

class VehicleModelBike {
   public:
    explicit VehicleModelBike(const std::string &yaml_file) : _param(yaml_file) {}

    Param &getParam() { return _param; }

    void updateState(State &state, Control &input, const double dt) {
        validateInput(input);

        double Fz = _getNormalForce(state);

        double slip_angle_front = getSlipAngle(state, input, true);
        double FyF = _getFy(Fz, true, slip_angle_front);

        double slip_angle_back = getSlipAngle(state, input, false);
        double FyR = _getFy(Fz, false, slip_angle_back);

        // Drivetrain Model
        const double Fx = _getFx(state, input);
        // Dynamics
        const auto x_dot_dyn = _f(state, input, Fx, FyF, FyR);
        const auto x_next_dyn = state + x_dot_dyn * dt;
        state = _fKinCorrection(x_next_dyn, state, input, Fx, dt);

        // Set the acceleration based on the change in velocity
        state.a_x = x_dot_dyn.v_x;
        state.a_y = x_dot_dyn.v_y;

        validateState(state);
    }

    State _f(const State &x, const Control &u, const double Fx, const double FyF, const double FyR) {
        const double FyF_tot = 2 * FyF;
        const double FyR_tot = 2 * FyR;

        State x_dot{};

        x_dot.x = std::cos(x.yaw) * x.v_x - std::sin(x.yaw) * x.v_y;
        x_dot.y = std::sin(x.yaw) * x.v_x + std::cos(x.yaw) * x.v_y;

        x_dot.yaw = x.r_z;

        x_dot.v_x = (x.r_z * x.v_y) + (Fx - std::sin(u.steering) * FyF_tot) / _param.inertia.m;
        x_dot.v_y = ((std::cos(u.steering) * FyF_tot) + FyR_tot) / _param.inertia.m - (x.r_z * x.v_x);

        x_dot.r_z =
            (std::cos(u.steering) * FyF_tot * _param.kinematic.l_F - FyR_tot * _param.kinematic.l_R) / _param.inertia.I_z;

        return x_dot;
    }

    State _fKinCorrection(const State &x_in, const State &x_state, const Control &u, const double Fx,
                                        const double dt) {
        State x = x_in;
        const double v_x_dot = Fx / (_param.inertia.m);
        const double v = std::hypot(x_state.v_x, x_state.v_y);
        const double v_blend = 0.5 * (v - 1.5);
        const double blend = std::fmax(std::fmin(1.0, v_blend), 0.0);

        x.v_x = blend * x.v_x + (1.0 - blend) * (x_state.v_x + dt * v_x_dot);

        const double v_y = std::tan(u.steering) * x.v_x * _param.kinematic.l_R / _param.kinematic.l;
        const double r = std::tan(u.steering) * x.v_x / _param.kinematic.l;

        x.v_y = blend * x.v_y + (1.0 - blend) * v_y;
        x.r_z = blend * x.r_z + (1.0 - blend) * r;
        return x;
    }

    double _getFx(const State &x, const Control &u) {
        const double acc = x.v_x <= 0.0 && u.acceleration < 0.0 ? 0.0 : u.acceleration;
        const double Fx = acc * _param.inertia.m - _getFdrag(x);
        return Fx;
    }

    double _getNormalForce(const State &x) { return _param.inertia.g * _param.inertia.m + _getFdown(x); }

    double _getFdown(const State &x) { return _param.aero.c_down * x.v_x * x.v_x; }

    double _getFdrag(const State &x) { return _param.aero.c_drag * x.v_x * x.v_x; }

    double _getFy(const double Fz, bool front, double slip_angle) {
        const double Fz_axle = front ? _getDownForceFront(Fz) : _getDownForceRear(Fz);

        const double B = _param.tire.B;
        const double C = _param.tire.C;
        const double D = _param.tire.D;
        const double E = _param.tire.E;
        const double mu_y = D * std::sin(C * std::atan(B * (1.0 - E) * slip_angle + E * std::atan(B * slip_angle)));
        const double Fy = Fz_axle * mu_y;
        return Fy;
    }

    double _getDownForceFront(const double Fz) {
        double FzAxle = 0.5 * _param.kinematic.w_front * Fz;
        return FzAxle;
    }

    double _getDownForceRear(const double Fz) {
        double FzAxle = 0.5 * (1 - _param.kinematic.w_front) * Fz;
        return FzAxle;
    }

    void validateState(State &state) { state.v_x = std::max(0.0, state.v_x); }

    void validateInput(Control &input) {
        double max_acc = _param.input_ranges.acc.max;
        double min_acc = _param.input_ranges.acc.min;

        double max_vel = _param.input_ranges.vel.max;
        double min_vel = _param.input_ranges.vel.min;

        double max_delta = _param.input_ranges.delta.max;
        double min_delta = _param.input_ranges.delta.min;

        input.acceleration = std::fmin(std::fmax(input.acceleration, min_acc), max_acc);
        input.velocity = std::fmin(std::fmax(input.velocity, min_vel), max_vel);
        input.steering = std::fmin(std::fmax(input.steering, min_delta), max_delta);
    }

    double getSlipAngle(const State &x, const Control &u, bool is_front) {
        double lever_arm_length = _param.kinematic.l * _param.kinematic.w_front;

        if (!is_front) {
            double v_x = std::max(1.0, x.v_x);
            return std::atan((x.v_y - lever_arm_length * x.r_z) / (v_x - 0.5 * _param.kinematic.axle_width * x.r_z));
        }

        double v_x = std::max(1.0, x.v_x);
        return std::atan((x.v_y + lever_arm_length * x.r_z) / (v_x - 0.5 * _param.kinematic.axle_width * x.r_z)) - u.steering;
    }

   protected:
    Param _param;

};
typedef std::unique_ptr<VehicleModelBike> VehicleModelBikePtr;

}  // namespace vehicle_plugins
}  // namespace gazebo_plugins
