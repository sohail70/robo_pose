#ifndef MEDIATOR_HPP
#define MEDIATOR_HPP
#include<iostream>
#include<fusion/motion_model.hpp>
#include<geometry_msgs/msg/twist.hpp>
namespace Filter{

    struct Message{
        enum class MessageType{
            VELOCITY_UPDATE,
        };

        MessageType type;
        geometry_msgs::msg::Twist velocityData;        
    };

    class Mediator{
        public:
            virtual void notifyMessage(const Message& ) = 0;
    };

    class MessageHub: public Mediator{
        public:
        MessageHub(MotionModel* );
            virtual void notifyMessage(const Message& ) override;

        private:
            MotionModel* motion_model_;
    };

}; // namespace Filter





#endif