#include<fusion/filter_node.hpp>

namespace Filter{
    FilterNode::FilterNode(rclcpp::NodeOptions options_):Node("Filter",options_)
    {
        this->declare_parameter<std::vector<std::string>>("states");
        this->declare_parameter<int>("model_type");
        this->declare_parameter<int>("filter_type");
        int sensor_id = 0;
        while(true)
        {
            std::vector<std::string> sensor_states;
            std::string sensor_topic;
            std::string sensor_states_param = "sensor_" + std::to_string(sensor_id) + "_states";
            std::string sensor_topic_param = "sensor_" + std::to_string(sensor_id) + "_topic";
            this->declare_parameter<std::string>(sensor_topic_param , std::string());
            this->declare_parameter<std::vector<std::string>>(sensor_states_param , std::vector<std::string>());
            this->get_parameter(sensor_states_param, sensor_states);
            this->get_parameter(sensor_topic_param, sensor_topic);
            if (sensor_states.empty() || sensor_topic.empty())
            {
                break; 
            }
            RCLCPP_INFO(this->get_logger() , "Sensor id %i" , sensor_id);
            RCLCPP_INFO(this->get_logger() , "states: ");
            for (const auto& state : sensor_states) {
                RCLCPP_INFO(this->get_logger() , "--- %s" ,state.c_str() );
            }
            RCLCPP_INFO(this->get_logger() , "topic: %s" ,sensor_topic.c_str() );
            sensor_id++;
        }
        // initialize();
    }

    void FilterNode::initialize()
    {
        std::vector<std::string> config_states_;
        int model_type;
        int filter_type;
        this->get_parameter("states" , config_states_);
        this->get_parameter("model_type" , model_type);
        this->get_parameter("filter_type" , filter_type);
        states_ = std::make_unique<StateSpace>(config_states_);
        model_factory_ = std::make_unique<MotionModelFactory>();
        model_ = model_factory_->createModel( static_cast<ModelType>(model_type),  states_.get());
        hub_ = std::make_unique<Filter::MessageHub>(model_.get());
        filter_factory_ = std::make_unique<FilterFactory>();
        filter_ = filter_factory_->createFilter(static_cast<FilterType>(filter_type) , std::move(model_));

    }

    std::unique_ptr<StateSpace> FilterNode::getStateSpace()
    {
        return std::move(states_);
    }
} // namespace Filter