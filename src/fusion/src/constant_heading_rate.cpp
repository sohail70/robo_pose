
#include "fusion/constant_heading_rate.hpp"
namespace Filter{
    ConstantHeadingRate::ConstantHeadingRate()
    {
        std::cout<<"Ctor of constant heading rate \n";
    }



    autodiff::VectorXreal ConstantHeadingRate::propagate(const autodiff::VectorXreal& state)
    {
        autodiff::VectorXreal newState(state.size());
        autodiff::real dt = autodiff::real(dt_.seconds()); 
        const auto index = states_->getStateOrder();
        // std::cout<<"x_dot:"<<(index.find("x") != index.end() )<<"\n";
        newState(0) = state(index.at("x")) + state(index.at("x_dot")) * cos(state(index.at("yaw"))) * dt; // Update x
        newState(1) = state(index.at("y")) + state(index.at("x_dot")) * sin(state(index.at("yaw"))) * dt; // Update y
        newState(2) = state(index.at("yaw")) + state(index.at("yaw_dot")) * dt;                 // Update yaw angle
        newState(3) = state(index.at("x_dot")) + state(index.at("x_ddot"))*dt;                                 
        newState(4) = state(index.at("yaw_dot"));                                
        newState(5) = state(index.at("x_ddot"));
        return newState;
    }


    // Eigen::MatrixXd ConstantHeadingRate::getJacobian() //Hardcoded jacobina for the [x,y,yaw,x_dot,yaw_dot,x_ddot] states
    // {
    //     Eigen::MatrixXd A = Eigen::MatrixXd(states_->states_.size() , states_->states_.size());   
    //     A(0,0) = 1; A(0,1) = 0; A(0,2) = -states_->getStates()[states_->getStateOrder().at("x_dot")].val()*sin(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(0,3)= cos(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(0,4) = 0; A(0,5) = 0; 
    //     A(1,0) = 0; A(1,1) = 1; A(1,2) =  states_->getStates()[states_->getStateOrder().at("x_dot")].val()*cos(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(1,3)= sin(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(1,4) = 0; A(1,5) = 0;
    //     A(2,0) = 0; A(2,1) = 0; A(2,2) =  1;A(2,3)= 0;A(2,4) = dt_.seconds();A(2,5) = 0;
    //     A(3,0) = 0; A(3,1) = 0; A(3,2) =  0;A(3,3)= 1;A(3,4) = 0; A(3,5) = dt_.seconds();
    //     A(4,0) = 0; A(4,1) = 0; A(4,2) =  0;A(4,3)= 0;A(4,4) = 1; A(4,5) = 0;
    //     A(5,0) = 0; A(5,1) = 0; A(5,2) =  0;A(5,3)= 0;A(5,4) = 0; A(5,5) = 1;

    //     return A;
  
    // }

} //namespace Filter