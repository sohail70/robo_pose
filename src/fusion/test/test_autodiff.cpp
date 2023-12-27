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
TEST(autodiff, wrongApproach)
{
    using Eigen::MatrixXd;
    std::vector<std::string> state_names_{"x" , "y" , "yaw" , "x_dot" , "yaw_dot"};
    Filter::StateSpace states_(state_names_);
    states_.updateStates({0.0,0.0,0.0,5.0,5.0});

    VectorXreal& state = states_.getStates();   // Current state (x, y, yaw, v, w)
    // state << 1, 2, 0.3, 5.0, 0.1; // Method1
    //Method2

    // int index = 0;
    // for(auto it = states_.getStateOrder().begin() ; it!= states_.getStateOrder().end() ; ++it)
    // {
    //     state(index) = states_.getStates()[*it];
    //     std::cout<<"S:"<<state(index)<<"\n";
    //     index++;
    // }

    for(int i =  0 ; i<10 ; i++)
    {
        real deltaT = 0.001;     // Time step

        VectorXreal newState;  // Updated state after time step

        MatrixXd J = jacobian(stateUpdate, wrt(state), at(state, deltaT), newState);
        state = newState;
        std::cout << "New State = \n" << newState << std::endl;
        std::cout << "Jacobian Matrix = \n" << J << std::endl;
    }
}



// // Define the state update function
// VectorXreal stateUpdate2(const VectorXreal& state) {
//     real dt = 1.0;
//     real v = 1.0;
//     real w = 1.0;
//     VectorXreal new_state(5);
//     new_state(0) = state(0) + state(3) * cos(state(2)) * dt; // x(k+1)
//     new_state(1) = state(1) + state(3) * sin(state(2)) * dt; // y(k+1)
//     new_state(2) = state(2) + state(4) * dt;                // yaw(k+1)
//     new_state(3) = state(3);                         // v(k+1)
//     new_state(4) = state(4);                         // w(k+1)
//     return new_state;
// }


VectorXreal stateUpdate2(const VectorXreal& x , const VectorXreal& y) {
    real dt = 1.0;
    VectorXreal f(2);
    f(0) = x(0)*x(0);
    f(1) = y(0)*y(0); 
    // new_state(1) = state(1)*state(1);
    return f;
}


TEST(autodiff,works)
{
    using Eigen::MatrixXd;

    VectorXreal x(1);   // State vector x, y, yaw, v, w
    x <<0; // Initial state values
    VectorXreal y(1);
    y << 0;

    for (int i = 1; i < 5; ++i) {
        MatrixXd J_state;
        VectorXreal f(2);//f1(x,y) and f2(x,y)
        jacobian(stateUpdate2, wrt(x,y), at(x,y),f, J_state);
        std::cout << "Jacobian at iteration " << i + 1 << ":\n" << J_state << std::endl;
        // state = new_state;
        // std::cout<<"State: \n"<<f<<"\n";
        // stateUpdate2(state);
        x << i;
        y << i; 
        // state = new_state; // Update state for the next iteration
    }
}




VectorXreal stateUpdate3( VectorXreal& x) {
    real dt = 1.0;
    VectorXreal f(5);
    f(0) = x(0) + x(3)*cos(x(2))*dt;
    f(1) = x(1) + x(3)*sin(x(2))*dt; 
    f(2) = x(2) + x(4)*dt;
    f(3) = x(3);
    f(4) = x(4);
    // new_state(1) = state(1)*state(1);
    return f;
}


//I could've created 3 VectorXreal of 1 dimentsion as x and y and yaw similar to last example but i decided to use one x varibale

TEST(autodiff,rightApproach)
{
    using Eigen::MatrixXd;

    VectorXreal x(5);   // State vector x, y, yaw, v, w
    x <<0,0,0,1,0.1; // Initial state values

    for (int i = 1; i < 5; ++i) {
        MatrixXd J_state;
        VectorXreal f(5);//f1(x,y,yaw,v,w) and f2(x,...) etc
        jacobian(stateUpdate3, wrt(x), at(x),f, J_state);
        std::cout << "Jacobian at iteration " << i + 1 << ":\n" << J_state << std::endl;
        std::cout<<"State: \n"<<f<<"\n";
        x << f(0).val(),f(1).val(),f(2).val(),f(3).val(),f(4).val(); //if you don't put val you are gonna get wrong jacobian

    }
}