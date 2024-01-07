#ifndef CAR_HPP
#define CAR_HPP

#include "fusion/motion_model.hpp"

namespace Filter{

    class Car: public MotionModel
    {
        public:
            Car();
            virtual void init() override;
            virtual Eigen::MatrixXd getJacobian() override;
            virtual Eigen::MatrixXd update(const rclcpp::Time& , const rclcpp::Duration&) override;
            virtual autodiff::VectorXreal propagate(const autodiff::VectorXreal&) override;
            virtual void setVelocity(const Filter::Velocity& ) override;
            virtual void setAngularVelocity(const Filter::AngularVelocity& ) override;
            virtual void setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& ) override;
            virtual void setStates(std::shared_ptr<StateSpace> ) override;

            virtual rclcpp::Duration getDt() override;

        private:

    };
} //namespace Filter
#endif