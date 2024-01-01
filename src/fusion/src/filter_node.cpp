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
            std::string msg_type;
            std::string sensor_states_param = "sensor_" + std::to_string(sensor_id) + "_states";
            std::string sensor_topic_param = "sensor_" + std::to_string(sensor_id) + "_topic";
            std::string sensor_msg_type_param = "sensor_" + std::to_string(sensor_id) + "_msg";
            this->declare_parameter<std::string>(sensor_topic_param , std::string()); //default value for params is neccessary essentially when it doesn't exist in the yaml file
            this->declare_parameter<std::string>(sensor_msg_type_param , std::string()); //default value for params is neccessary essentially when it doesn't exist in the yaml file
            this->declare_parameter<std::vector<std::string>>(sensor_states_param , std::vector<std::string>());
            this->get_parameter(sensor_states_param, sensor_states);
            this->get_parameter(sensor_topic_param, sensor_topic);
            this->get_parameter(sensor_msg_type_param, msg_type);
            if (sensor_states.empty() || sensor_topic.empty() || msg_type.empty())
            {
                break; 
            }
            RCLCPP_INFO(this->get_logger() , "Sensor id %i" , sensor_id);
            RCLCPP_INFO(this->get_logger() , "Sensor msg type: %s" , msg_type.c_str());
            RCLCPP_INFO(this->get_logger() , "states: ");
            for (const auto& state : sensor_states) {
                RCLCPP_INFO(this->get_logger() , "--- %s" ,state.c_str() );
            }
            RCLCPP_INFO(this->get_logger() , "topic: %s" ,sensor_topic.c_str() );
            sensor_states_[sensor_topic] = sensor_states;

            //////////////////////
            //create subscription
            if(msg_type == "sensor_msgs::msg::Imu")
            {
                RCLCPP_INFO(this->get_logger() , "IMU SUB");
                auto imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(sensor_topic,rclcpp::SensorDataQoS(),[this, sensor_topic](const sensor_msgs::msg::Imu::SharedPtr msg) {
                        imuCallback(msg, sensor_topic);
                    });
                sensor_subs_.push_back(imu_sub_);
            }
            else if (msg_type == "nav_msgs::msg::Odom")
            {
                RCLCPP_INFO(this->get_logger() , "ODOM SUB");
                auto odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(sensor_topic, 1 , [this , sensor_topic](const nav_msgs::msg::Odometry::SharedPtr msg){
                    odomCallback(msg , sensor_topic);
                });
                sensor_subs_.push_back(odom_sub_);
            }
            else{
                RCLCPP_INFO(this->get_logger() , "This type is not implemented yet");
            }
            ////////////////
            sensor_id++;
        }
        initialize();
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
        // hub_ = std::make_unique<Filter::MessageHub>(model_.get());
        filter_factory_ = std::make_unique<FilterFactory>();
        filter_ = filter_factory_->createFilter(static_cast<FilterType>(filter_type) , std::move(model_));

    }


    void FilterNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg , std::string topic_name_)
    {
        RCLCPP_INFO(this->get_logger() , "imu callback: %s" , topic_name_.c_str());

        std::vector<double> data; data.resize(2);
        data[0] = msg->angular_velocity.z;
        data[1] = msg->linear_acceleration.x;

        Observations current_obs_;
        current_obs_.time_ = msg->header.stamp;
        // RCLCPP_INFO(this->get_logger() , "Time:%f " , current_obs_.time_.seconds()); //rclcpp::Time store the double variable as seconod so no worries about the nano seconds stuff i guess
        auto index = states_->getStateOrder();
        current_obs_.states_.setZero(index.size());
        for (auto sensor_state_ : sensor_states_[topic_name_])
        {
            auto it = index.find(sensor_state_);
            if (sensor_state_ == "yaw_dot")
            {
                current_obs_.states_(it->second) = msg->angular_velocity.z;
            }
            else if (sensor_state_ == "x_ddot")
            {
                current_obs_.states_(it->second) = msg->linear_acceleration.x;
            }
            else{

            }
        }
        // for(int i = 0 ; i <current_obs_.states_.size(); i++)
        // {
        //     RCLCPP_INFO(this->get_logger() , "obs: %f" , current_obs_.states_(i));
        // }

        observations_.push(current_obs_);
    }    


    void FilterNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg , std::string topic_name_)
    {
        RCLCPP_INFO(this->get_logger() , "odom callback: %s" , topic_name_.c_str());
    }    

    std::unique_ptr<StateSpace> FilterNode::getStateSpace()
    {
        return std::move(states_);
    }
} // namespace Filter