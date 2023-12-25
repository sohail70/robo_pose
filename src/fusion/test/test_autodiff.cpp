#include<iostream>
#include<gtest/gtest.h>
#include<autodiff/forward/real.hpp>
#include<autodiff/forward/real/eigen.hpp>
#include<map>
#include<vector>
#include<fusion/state_space.hpp>
using namespace autodiff;

// Discrete constant heading rate equation
// Discrete constant heading rate equations for position and yaw angle
VectorXreal updatePositionYaw(const VectorXreal& pos, const real& v, const real& theta, const real& omega, const real& dt)
{
    VectorXreal newState(3);
    newState(0) = pos(0) + v * cos(theta) * dt; // Update x
    newState(1) = pos(1) + v * sin(theta) * dt; // Update y
    newState(2) = theta + omega * dt;           // Update yaw angle
    return newState;
}

TEST(autodiff, simple)
{
     using Eigen::MatrixXd;

    VectorXreal state(3);  // Current state (x, y, theta)
    state << 1, 2, 0.0;    // Initialize state as (1, 2, 0.3 radians)

    real velocity = 5.0;   // Constant velocity
    real omega = 0.1;      // Constant yaw rate (example value)
    real deltaT = 1;     // Time step

    VectorXreal newState;  // Updated state after time step

    MatrixXd J = jacobian(updatePositionYaw, wrt(state), at(state, velocity, state(2), omega, deltaT), newState);

    std::cout << "New State = \n" << newState << std::endl;
    std::cout << "Jacobian Matrix = \n" << J << std::endl;
}



// Function for the state update equations
VectorXreal stateUpdate(const VectorXreal& state, const real& dt)
{
    VectorXreal newState(5);

    newState(0) = state(0) + state(3) * cos(state(2)) * dt; // Update x
    newState(1) = state(1) + state(3) * sin(state(2)) * dt; // Update y
    newState(2) = state(2) + state(4) * dt;                 // Update yaw angle
    newState(3) = state(3);                                 // Velocity remains constant: bezari nazari javab mide va in khoobe!
    newState(4) = state(4);                                 // Yaw rate remains constant

    return newState;
}

// // Function for the state update equationsa: lambda versoin which may come in handy in class usage
// auto stateUpdate = [](const VectorXreal& state, const real& dt) {
//     VectorXreal newState(5);

//     newState(0) = state(0) + state(3) * cos(state(2)) * dt; // Update x
//     newState(1) = state(1) + state(3) * sin(state(2)) * dt; // Update y
//     newState(2) = state(2) + state(4) * dt;                 // Update yaw angle
//     newState(3) = state(3);                                 // Velocity remains constant
//     newState(4) = state(4);                                 // Yaw rate remains constant

//     return newState;
// };
TEST(autodiff, augmented)
{
    using Eigen::MatrixXd;
    std::vector<std::string> state_names_{"x" , "y" , "yaw" , "x_dot" , "yaw_dot"};
    Filter::StateSpace states_(state_names_);
    states_.updateStates({0.0,0.0,0.0,5.0,5.0});

    VectorXreal state(5);   // Current state (x, y, yaw, v, w)
    // state << 1, 2, 0.3, 5.0, 0.1; // Method1
    //Method2
    int index = 0;
    for(auto it = states_.getStateOrder().begin() ; it!= states_.getStateOrder().end() ; ++it)
    {
        state(index) = states_.getStates()[*it];
        std::cout<<"S:"<<state(index)<<"\n";
        index++;
    }


    real deltaT = 0.001;     // Time step

    VectorXreal newState;  // Updated state after time step

    MatrixXd J = jacobian(stateUpdate, wrt(state), at(state, deltaT), newState);

    std::cout << "New State = \n" << newState << std::endl;
    std::cout << "Jacobian Matrix = \n" << J << std::endl;
}