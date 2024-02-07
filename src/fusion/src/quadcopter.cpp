// MIT License
//
// Copyright (c) 2023 Soheil Espahbodi Nia
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "fusion/quadcopter.hpp"

namespace Filter{
    Quadcopter::Quadcopter()
    {
        std::cout<<"Ctor of Quadrotor \n";
    }

    autodiff::VectorXreal Quadcopter::propagate(const autodiff::VectorXreal & state)
    {
        // std::cout<<"PROPAGATE OF PLUGIN \n";
        autodiff::VectorXreal newState(state.size());
        autodiff::real dt = autodiff::real(dt_.seconds());
        const auto index = states_->getStateOrder();
        // std::cout<<"x_dot:"<<(index.find("x") != index.end() )<<"\n";
        newState(0) = state(index.at("x")) +  (   state(index.at("z_dot"))*(sin(state(index.at("roll"))) * sin(state(index.at("yaw"))) + cos(state(index.at("roll")))* cos(state(index.at("yaw")))*cos(state(index.at("pitch"))) )        
                                                - state(index.at("y_dot"))*(cos(state(index.at("roll"))) * sin(state(index.at("yaw"))) - cos(state(index.at("yaw")))* sin(state(index.at("roll")))*sin(state(index.at("pitch"))) )        
                                                + state(index.at("x_dot"))*(cos(state(index.at("yaw"))) * cos(state(index.at("pitch"))))
                                                )* dt;

        newState(1) = state(index.at("y")) +  (   state(index.at("y_dot"))*(cos(state(index.at("roll"))) * cos(state(index.at("yaw"))) + sin(state(index.at("roll")))* sin(state(index.at("yaw")))*sin(state(index.at("pitch"))) )        
                                                - state(index.at("z_dot"))*(cos(state(index.at("yaw"))) * sin(state(index.at("roll"))) - cos(state(index.at("roll")))* sin(state(index.at("yaw")))*sin(state(index.at("pitch"))) )        
                                                + state(index.at("x_dot"))*(cos(state(index.at("pitch"))) * sin(state(index.at("yaw"))))
                                                )* dt;

        newState(2) = state(index.at("z")) + ( state(index.at("z_dot")) * (cos(state(index.at("roll"))) * cos(state(index.at("pitch"))))
                                              - state(index.at("x_dot")) * sin(state(index.at("pitch")))
                                              + state(index.at("y_dot")) *(cos(state(index.at("pitch"))) * sin(state(index.at("roll"))))
                                              )*dt;

        newState(3) = state(index.at("roll")) + ( state(index.at("roll_dot"))
                                                 + state(index.at("yaw_dot")) * (cos(state(index.at("roll"))) * tan(state(index.at("pitch"))))
                                                 + state(index.at("pitch_dot")) * (sin(state(index.at("roll"))) * tan(state(index.at("pitch"))))
                                                )*dt;
        newState(4) = state(index.at("pitch")) + ( state(index.at("pitch_dot")) * cos(state(index.at("roll"))) 
                                                  - state(index.at("yaw_dot"))  * sin(state(index.at("roll")))
                                                )*dt;
        newState(5) = state(index.at("yaw")) + ( state(index.at("yaw_dot")) * (cos(state(index.at("roll")))/cos(state(index.at("pitch"))))
                                                  - state(index.at("pitch_dot"))  * (sin(state(index.at("roll")))/cos(state(index.at("pitch"))))
                                                )*dt;
        newState(6) = state(index.at("x_dot"));
        newState(7) = state(index.at("y_dot"));
        newState(8) = state(index.at("z_dot"));
        newState(9) = state(index.at("roll_dot"));
        newState(10) = state(index.at("pitch_dot"));
        newState(11) = state(index.at("yaw_dot"));
        newState(12) = state(index.at("x_ddot"));
        newState(13) = state(index.at("y_ddot"));
        newState(14) = state(index.at("z_ddot"));
        return newState;
    }

    // Eigen::MatrixXd Car::getJacobian()
    // {}

} //namespace Filter