
#include "fusion/constant_heading_rate.hpp"
namespace Filter{
    ConstantHeadingRate::ConstantHeadingRate()
    {
        std::cout<<"Ctor of constant heading rate \n";
        init();
    }

    void ConstantHeadingRate::init()
    {
        // previous_time_ = rclcpp::Clock().now();
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


    Eigen::MatrixXd ConstantHeadingRate::getJacobian() //Hardcoded jacobina for the [x,y,yaw,x_dot,yaw_dot,x_ddot] states
    {
        Eigen::MatrixXd A = Eigen::MatrixXd(states_->states_.size() , states_->states_.size());   
        A(0,0) = 1; A(0,1) = 0; A(0,2) = -states_->getStates()[states_->getStateOrder().at("x_dot")].val()*sin(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(0,3)= cos(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(0,4) = 0; A(0,5) = 0; 
        A(1,0) = 0; A(1,1) = 1; A(1,2) =  states_->getStates()[states_->getStateOrder().at("x_dot")].val()*cos(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(1,3)= sin(states_->getStates()[states_->getStateOrder().at("yaw")].val())*dt_.seconds(); A(1,4) = 0; A(1,5) = 0;
        A(2,0) = 0; A(2,1) = 0; A(2,2) =  1;A(2,3)= 0;A(2,4) = dt_.seconds();A(2,5) = 0;
        A(3,0) = 0; A(3,1) = 0; A(3,2) =  0;A(3,3)= 1;A(3,4) = 0; A(3,5) = dt_.seconds();
        A(4,0) = 0; A(4,1) = 0; A(4,2) =  0;A(4,3)= 0;A(4,4) = 1; A(4,5) = 0;
        A(5,0) = 0; A(5,1) = 0; A(5,2) =  0;A(5,3)= 0;A(5,4) = 0; A(5,5) = 1;

        return A;
  
    }

    Eigen::MatrixXd ConstantHeadingRate::update(const rclcpp::Time& current_time_ , const rclcpp::Duration& dt)
    {
        static rclcpp::Time previous_time_ = current_time_;
        // dt_ = current_time_ - previous_time_;
        dt_ = dt;
        autodiff::VectorXreal& state_ = states_->getStates();
        autodiff::VectorXreal newState; 
        
        //////Test///////////        
        // std::cout<<"states Jacobian Test:\n "<<this->getJacobian()<<"\n \n";
        // std::cout<<"States: \n"<<state_<<"\n \n";
        ////////////////////
        Eigen::MatrixXd J = jacobian( [&](auto state_){return this->propagate(state_); }, wrt(state_), at(state_), newState);
        // std::cout<<"states Jacobian:\n "<<J<<"\n \n";
        // std::cout<<"dt:\n "<<dt_.seconds()<<"\n \n";
        // std::cout<<"prev:\n "<<previous_time_.seconds()<<"\n \n";
        // std::cout<<"curr:\n "<<current_time_.seconds()<<"\n \n";
        // std::cout<<"prev nano:\n "<<previous_time_.nanoseconds()<<"\n \n";
        // std::cout<<"curr nano:\n "<<current_time_.nanoseconds()<<"\n \n";
        previous_time_ = current_time_;
        
        for(int i = 0 ; i<state_.size() ; i++)
            state_(i) = newState(i).val();
        normalizeAngle(states_->getStates()[states_->getStateOrder().at("yaw")].val());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("STATE") , state_);

        return J;  //retrun nazari seg fault mide!!!!
    }

    void ConstantHeadingRate::setVelocity(const Filter::Velocity& velocity_)
    {
        states_->getStates()[states_->getStateOrder().at("x_dot")] = velocity_.x_dot;
    }

    void ConstantHeadingRate::setAngularVelocity(const Filter::AngularVelocity& angular_velocity_)
    {
        states_->getStates()[states_->getStateOrder().at("yaw_dot")] = angular_velocity_.yaw_dot;
    }

    void ConstantHeadingRate::setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& twist_)
    {
        if(twist_.linear.x == 0)
            states_->getStates()[states_->getStateOrder().at("x_dot")] = twist_.linear.x;
        // states_->getStates()[states_->getStateOrder().at("yaw_dot")] = twist_.angular.z;
    }

    void ConstantHeadingRate::setStates(std::shared_ptr<StateSpace> states_)
    {
        this->states_ = states_;
    }


    void ConstantHeadingRate::setPosition(const Position& position_)
    {

    }

    Filter::Position ConstantHeadingRate::getPosition()
    {
        Filter::Position position_; position_.x = states_->getStates()[states_->getStateOrder().at("x")].val() , position_.y = states_->getStates()[states_->getStateOrder().at("y")].val();
        return position_;
    }

    void ConstantHeadingRate::setAngle(const Angle& angle_)
    {
        
    }
    Filter::Angle ConstantHeadingRate::getAngle()
    {
        Filter::Angle angle_; angle_.yaw = states_->getStates()[states_->getStateOrder().at("yaw")].val();
        return angle_;
    }

    Filter::Velocity ConstantHeadingRate::getVelocity()
    {
        Filter::Velocity velocity_; velocity_.x_dot = states_->getStates()[states_->getStateOrder().at("x_dot")].val();
        return velocity_;
    }

    Filter::AngularVelocity ConstantHeadingRate::getAngularVelocity()
    {
        Filter::AngularVelocity angular_velocity_; angular_velocity_.yaw_dot = states_->getStates()[states_->getStateOrder().at("yaw_dot")].val();
        return angular_velocity_;
    }

    rclcpp::Duration ConstantHeadingRate::getDt()
    {
        return dt_;
    }

    void ConstantHeadingRate::normalizeAngle(double& angle_)
    {
        while(angle_>M_PI)
            angle_ -= 2.0*M_PI;
        while(angle_<-M_PI)
            angle_ += 2.0*M_PI;
    }
} //namespace Filter