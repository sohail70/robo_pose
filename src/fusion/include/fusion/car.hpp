#ifndef CAR_HPP
#define CAR_HPP

#include "fusion/motion_model.hpp"

namespace Filter{

    class Car: public MotionModel
    {
        public:
            Car();
            virtual void init() override;
            virtual autodiff::VectorXreal update(const autodiff::VectorXreal&) override;
            virtual void setVelocity(const Filter::Velocity& ) override;
            virtual void setAngularVelocity(const Filter::AngularVelocity& ) override;
            virtual void setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& ) override;
            virtual void setStates(StateSpace* ) override;
            virtual Eigen::MatrixXd getJacobian() override;
            virtual Eigen::MatrixXd calcJacobianAndUpdate(const rclcpp::Time&) override;

            virtual rclcpp::Duration getDt() override;

        private:

    };
} //namespace Filter
#endif