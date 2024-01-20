// MIT License
//
// Copyright (c) 2023 Soheil Espahbodi Nia
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include<fusion/filter_node.hpp>
// #include<chorno>
using namespace std::chrono_literals;
namespace Filter{
    FilterNode::~FilterNode(){
        if(rviz_marker_.joinable())
            rviz_marker_.join();
    }
    FilterNode::FilterNode(rclcpp::NodeOptions options_):Node("Filter",options_)
    {
        this->declare_parameter<std::vector<std::string>>("states");
        this->declare_parameter<std::vector<double>>("initial_states");
        this->declare_parameter<bool>("use_cmd");
        this->declare_parameter<bool>("publish_tf");
        this->declare_parameter<int>("model_type");
        this->declare_parameter<int>("filter_type");
        this->declare_parameter<double>("rate");
        this->declare_parameter<std::string>("odom_frame");
        this->declare_parameter<std::string>("base_link_frame");
        this->declare_parameter<std::string>("model_plugin");
        this->declare_parameter<std::vector<double>>("Q");
        this->declare_parameter<std::vector<double>>("R");

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
                auto odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(sensor_topic, rclcpp::QoS(1) , [this , sensor_topic](const nav_msgs::msg::Odometry::SharedPtr msg){
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

        bool use_cmd;
        this->get_parameter("use_cmd" , use_cmd);
        if(use_cmd)
        {
            auto cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel" , 1 , [this](const geometry_msgs::msg::Twist::SharedPtr msg){
                controlCallback(msg);
            });
            sensor_subs_.push_back(cmd_vel_sub_);
        }



        this->get_parameter("odom_frame" , odom_frame_);
        this->get_parameter("base_link_frame" , base_link_frame_);
        this->get_parameter("publish_tf" , publish_tf_);


        initialize();
        initializeStateAction();

        filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_filtered" , rclcpp::QoS(1));
        yaw_odom_pub_ = this->create_publisher<std_msgs::msg::Float64>("yaw_odom" , rclcpp::QoS(1));
        yaw_filter_pub_ = this->create_publisher<std_msgs::msg::Float64>("yaw_filter" , rclcpp::QoS(1));

        double rate_;
        this->get_parameter("rate" , rate_);
        // timer_ = this->create_wall_timer(std::chrono::nanoseconds(static_cast<long long>((1.0 / rate_) * 1.0e+9)), std::bind(&FilterNode::timerCallback, this));

        const std::chrono::duration<double> timespan_{1.0 / rate_};
        timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
            this->get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(timespan_),
            std::bind(&FilterNode::timerCallback, this), this->get_node_base_interface()->get_context());
        this->get_node_timers_interface()->add_timer(timer_, nullptr);


        visualization_ = std::make_shared<Visualization::Visualization>();
        params_.visualization_ = visualization_;
        rviz_marker_ = std::thread(&FilterNode::rviz_marker,this ,&params_);


        previous_update_time_ = this->now();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        odom_base_link_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void FilterNode::initialize()
    {
        std::vector<std::string> config_states_;
        int model_type;
        int filter_type;
        std::string model_plugin;
        std::vector<double> Q_;
        std::vector<double> R_;
        std::vector<double> initial_states;
        this->get_parameter("states" , config_states_);
        this->get_parameter("model_type" , model_type);
        this->get_parameter("filter_type" , filter_type);
        this->get_parameter("model_plugin" , model_plugin);
        this->get_parameter("Q" , Q_);
        this->get_parameter("R" , R_);
        this->get_parameter("initial_states" , initial_states);

        if(config_states_.at(0).empty())
        {
            std::cout<<"No state is being set so using default states \n";
            config_states_ = std::vector<std::string>{"x","y","z",
                                                      "roll","pitch","yaw",
                                                      "x_dot","y_dot","z_dot",
                                                      "roll_dot","pitch_dot","yaw_dot",
                                                      "x_ddot","y_ddot","z_ddot"};
        }

        for(auto cs : config_states_)
        {
            RCLCPP_INFO(this->get_logger() , "State: %s" , cs.c_str());
        }
        states_ = std::make_shared<StateSpace>(config_states_);
        states_->updateStates(initial_states);
        model_factory_ = std::make_unique<MotionModelFactory>();
        if(!model_plugin.empty())
            model_ = model_factory_->createModelFromPlugin(model_plugin, states_);
        else
            model_ = model_factory_->createModel( static_cast<ModelType>(model_type),  states_);

        local_motion_model_ = model_.get();
        filter_factory_ = std::make_unique<FilterFactory>();
        filter_ = filter_factory_->createFilter(static_cast<FilterType>(filter_type) , std::move(model_) , states_);
        filter_->setProcessNoise(Q_);
        filter_->setMeasurementNoise(R_);
        filter_->initialize();

    }


    void FilterNode::initializeStateAction()
    {
        imu_state_action_["yaw_dot"] = [](const sensor_msgs::msg::Imu::SharedPtr msg_ ,
                                          Observations& current_obs_,
                                          std::unordered_map<std::string,int> index_)
                                        { current_obs_.states_(index_.at("yaw_dot")) = msg_->angular_velocity.z;};
        imu_state_action_["x_ddot"] = [](const sensor_msgs::msg::Imu::SharedPtr msg_ ,
                                          Observations& current_obs_,
                                          std::unordered_map<std::string,int> index_)
                                        {current_obs_.states_(index_.at("x_ddot")) = msg_->linear_acceleration.x;};
        imu_state_action_["y_ddot"] = [](const sensor_msgs::msg::Imu::SharedPtr msg_ ,
                                          Observations& current_obs_,
                                          std::unordered_map<std::string,int> index_)
                                        {current_obs_.states_(index_.at("y_ddot")) = msg_->linear_acceleration.y;};
        imu_state_action_["z_ddot"] = [](const sensor_msgs::msg::Imu::SharedPtr msg_ ,
                                          Observations& current_obs_,
                                          std::unordered_map<std::string,int> index_)
                                        {current_obs_.states_(index_.at("z_ddot")) = msg_->linear_acceleration.z;};
        imu_state_action_["roll"] = [](const sensor_msgs::msg::Imu::SharedPtr msg_ , Observations& current_obs_,
                                          std::unordered_map<std::string,int> index_)
                                        {
                                            tf2::Quaternion quaternion;
                                            tf2::fromMsg(msg_->orientation, quaternion);
                                            double roll, pitch, yaw;
                                            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                                            current_obs_.states_(index_.at("roll")) = roll;
                                        };
        imu_state_action_["pitch"] = [](const sensor_msgs::msg::Imu::SharedPtr msg_ , Observations& current_obs_,
                                          std::unordered_map<std::string,int> index_)
                                        {
                                            tf2::Quaternion quaternion;
                                            tf2::fromMsg(msg_->orientation, quaternion);
                                            double roll, pitch, yaw;
                                            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                                            current_obs_.states_(index_.at("pitch")) = pitch;
                                        };
        imu_state_action_["yaw"] = [](const sensor_msgs::msg::Imu::SharedPtr msg_ , Observations& current_obs_,
                                          std::unordered_map<std::string,int> index_)
                                        {
                                            tf2::Quaternion quaternion;
                                            tf2::fromMsg(msg_->orientation, quaternion);
                                            double roll, pitch, yaw;
                                            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                                            current_obs_.states_(index_.at("yaw")) = yaw;
                                        };

        odom_state_action_["x"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                    {current_obs_.states_(index_.at("x")) = msg_->pose.pose.position.x ;};
        odom_state_action_["y"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                    {current_obs_.states_(index_.at("y")) = msg_->pose.pose.position.y ;};
        odom_state_action_["z"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                    {current_obs_.states_(index_.at("z")) = msg_->pose.pose.position.z ;};
        odom_state_action_["roll"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {
                                            tf2::Quaternion quaternion;
                                            tf2::fromMsg(msg_->pose.pose.orientation, quaternion);
                                            double roll, pitch, yaw;
                                            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                                            current_obs_.states_(index_.at("roll")) = roll;
                                        };
        odom_state_action_["pitch"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {       
                                            tf2::Quaternion quaternion;
                                            tf2::fromMsg(msg_->pose.pose.orientation, quaternion);
                                            double roll, pitch, yaw;
                                            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                                            current_obs_.states_(index_.at("pitch")) = pitch;
                                        };
        odom_state_action_["yaw"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {
                                            tf2::Quaternion quaternion;
                                            tf2::fromMsg(msg_->pose.pose.orientation, quaternion);
                                            double roll, pitch, yaw;
                                            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                                            current_obs_.states_(index_.at("yaw")) = yaw;
                                        };
        odom_state_action_["x_dot"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {current_obs_.states_(index_.at("x_dot")) = msg_->twist.twist.linear.x ;};
        odom_state_action_["y_dot"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {current_obs_.states_(index_.at("y_dot")) = msg_->twist.twist.linear.y ;};
        odom_state_action_["z_dot"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {current_obs_.states_(index_.at("z_dot")) = msg_->twist.twist.linear.z ;};
        odom_state_action_["roll_dot"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {current_obs_.states_(index_.at("roll_dot")) = msg_->twist.twist.angular.x ;};
        odom_state_action_["pitch_dot"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {current_obs_.states_(index_.at("pitch_dot")) = msg_->twist.twist.angular.y ;};
        odom_state_action_["yaw_dot"] = [](const nav_msgs::msg::Odometry::SharedPtr msg_, 
                                                               Observations & current_obs_,
                                                               std::unordered_map<std::string, int> index_)
                                        {current_obs_.states_(index_.at("yaw_dot")) = msg_->twist.twist.angular.z ;};
    }

    void FilterNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg_ , std::string topic_name_)
    {
        Observations current_obs_;
        autodiff::MatrixXreal H;
        current_obs_.time_ = msg_->header.stamp;
        auto index_ = states_->getStateOrder();
        H.setZero(index_.size() , index_.size());
        current_obs_.states_.setZero(index_.size());
        for (auto sensor_state_ : sensor_states_[topic_name_])
        {
            auto it_ = index_.find(sensor_state_);
            if(imu_state_action_.find(sensor_state_) != imu_state_action_.end())
            {
                imu_state_action_[sensor_state_](msg_ , current_obs_ , index_);
                H(it_->second , it_->second) = 1;
            }
            else{

            }

        }
        current_obs_.H = H;
        observations_.push(current_obs_);
    }

    void FilterNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_, std::string topic_name_)
    {
        Observations current_obs_;
        autodiff::MatrixXreal H;
        current_obs_.time_ = msg_->header.stamp;
        auto index_ = states_->getStateOrder();

        H.setZero(index_.size() , index_.size());
        current_obs_.states_.setZero(index_.size());
        for (auto sensor_state_ : sensor_states_[topic_name_])
        {
            auto it = index_.find(sensor_state_);
            if(odom_state_action_.find(sensor_state_) != odom_state_action_.end())
            {
                odom_state_action_.at(sensor_state_)(msg_ ,current_obs_ , index_ );
                H(it->second , it->second) = 1;
            }
            else
            {

            }

        }
        current_obs_.H = H;
        observations_.push(current_obs_);
        // ///////publishing yaw data from odom in a topic//////////
        tf2::Quaternion quaternion_;
        tf2::fromMsg(msg_->pose.pose.orientation, quaternion_);
        double roll_, pitch_, yaw_;
        tf2::Matrix3x3(quaternion_).getRPY(roll_, pitch_, yaw_);
        std_msgs::msg::Float64 d_;
        d_.data = yaw_;
        yaw_odom_pub_->publish(d_);
    }


    void FilterNode::controlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if(local_motion_model_)
            local_motion_model_->setVelAndAngVelFromTwist(*msg);
        
    }

    void FilterNode::timerCallback()
    {
        if(observations_.empty())
        {
            // RCLCPP_INFO(this->get_logger() , "No obs is in queue %li" , observations_.size());
        }
        params_.time_ = this->now();
        geometry_msgs::msg::Pose2D pose_;
        while (!observations_.empty())
        {
            // if(observations_.top().time_ > this->now())
            //     break;
            rclcpp::Time cur_obs_time_ = observations_.top().time_;
            auto dt_ = cur_obs_time_ - previous_update_time_;
            filter_->predict(cur_obs_time_, dt_);
            filter_->update(observations_.top());
            previous_update_time_ = cur_obs_time_;
            observations_.pop();
        }

        ///////////////////publishing filtered data into a topic///////////// 
        autodiff::VectorXreal sta_ = filter_->getStates();
        auto index_ = states_->getStateOrder();
        filtered_odom_.header.frame_id = odom_frame_;
        filtered_odom_.child_frame_id = base_link_frame_;
        filtered_odom_.header.stamp = this->now();
        filtered_odom_.pose.pose.position.x = sta_(index_.at("x")).val();
        filtered_odom_.pose.pose.position.y = sta_(index_.at("y")).val();


        tf2::Quaternion quaternion_;
        quaternion_.setRPY(0.0,0.0,sta_(index_.at("yaw")).val());
        geometry_msgs::msg::Quaternion orientation_msg_;
        tf2::convert(quaternion_ , orientation_msg_);
        filtered_odom_.pose.pose.orientation = orientation_msg_;
        filtered_odom_pub_->publish(filtered_odom_);
        // /////////////////////publishing yaw in a topic//////////////////
        autodiff::VectorXreal st_ = filter_->getStates();
        std_msgs::msg::Float64 d;
        d.data = st_(index_.at("yaw")).val();
        yaw_filter_pub_->publish(d);
        ///////////////Send filtered states tf//////////////////////
        if(publish_tf_)
        {
            autodiff::VectorXreal s_ = filter_->getStates();
            tf2::Quaternion quat;
            auto index = states_->getStateOrder();
            quat.setRPY(index.count("roll")  ? s_(index.at("roll")).val()  : 0 ,
                        index.count("pitch") ? s_(index.at("pitch")).val() : 0 ,
                        index.count("yaw")   ? s_(index.at("yaw")).val()   : 0 );
            odom_base_link_transform_.header.frame_id = odom_frame_;
            odom_base_link_transform_.child_frame_id = base_link_frame_;
            // rclcpp::Duration offset_(0, 1000000); //1 ms
            odom_base_link_transform_.header.stamp = this->now();// - offset;
            odom_base_link_transform_.transform.translation.x = index.count("x") ? s_(index.at("x")).val() : 0;
            odom_base_link_transform_.transform.translation.y = index.count("y") ? s_(index.at("y")).val() : 0;
            odom_base_link_transform_.transform.translation.z = index.count("z") ? s_(index.at("z")).val() : 0;
            odom_base_link_transform_.transform.rotation.x = quat.getX();  
            odom_base_link_transform_.transform.rotation.y = quat.getY();  
            odom_base_link_transform_.transform.rotation.z = quat.getZ();  
            odom_base_link_transform_.transform.rotation.w = quat.getW();  
            odom_base_link_broadcaster_->sendTransform(odom_base_link_transform_);
        }
    }
    std::shared_ptr<StateSpace> FilterNode::getStateSpace()
    {
        return std::move(states_);
    }
} // namespace Filter