#include<fusion/mediator.hpp>

namespace Filter{
    MessageHub::MessageHub(MotionModel* motion_model_):motion_model_(motion_model_){}

    void MessageHub::notifyMessage(const Message& message)
    {
        switch (message.type)
        {
        case Message::MessageType::VELOCITY_UPDATE:
            motion_model_->setVelAndAngVelFromTwist(message.velocityData);
            break;
        
        default:
            break;
        }
    }
};// namespace Filter
