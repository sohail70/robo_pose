#ifndef CONSTANT_HEADING_RATE_HPP
#define CONSTANT_HEADING_RATE_HPP

#include "fusion/motion_model.hpp"
namespace Filter{

    class ConstantHeadingRate: public MotionModel
    {
        public:
            ConstantHeadingRate();
            virtual void init() override;
            virtual Eigen::MatrixXd update(const rclcpp::Time& , const rclcpp::Duration& ) override;
            virtual autodiff::VectorXreal propagate(const autodiff::VectorXreal&) override;
            virtual void setVelocity(const Filter::Velocity& ) override;
            virtual void setAngularVelocity(const Filter::AngularVelocity& ) override;
            virtual void setVelAndAngVelFromTwist(const geometry_msgs::msg::Twist& ) override;
            virtual void setStates(std::shared_ptr<StateSpace> ) override;
            virtual Eigen::MatrixXd getJacobian() override;

            virtual rclcpp::Duration getDt() override;

            void setPosition(const Position&);
            Filter::Position getPosition();
            void setAngle(const Angle&);
            Filter::Angle getAngle();
            Filter::Velocity getVelocity();
            Filter::AngularVelocity getAngularVelocity();
            void normalizeAngle(double&);

        private:
    };
} //namespace Filter
#endif