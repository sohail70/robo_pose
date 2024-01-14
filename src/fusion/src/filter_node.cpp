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
        this->declare_parameter<bool>("use_control");
        this->declare_parameter<bool>("publish_tf");
        this->declare_parameter<int>("model_type");
        this->declare_parameter<int>("filter_type");
        this->declare_parameter<double>("rate");
        this->declare_parameter<std::string>("odom_frame");
        this->declare_parameter<std::string>("base_link_frame");
        this->declare_parameter<std::string>("model_plugin");
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

        bool use_control;
        this->get_parameter("use_control" , use_control);
        if(use_control)
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

        const std::chrono::duration<double> timespan{1.0 / rate_};
        timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
            this->get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
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
        this->get_parameter("states" , config_states_);
        this->get_parameter("model_type" , model_type);
        this->get_parameter("filter_type" , filter_type);
        this->get_parameter("model_plugin" , model_plugin);
        for(auto cs : config_states_)
        {
            RCLCPP_INFO(this->get_logger() , "S: %s" , cs.c_str());
        }
        states_ = std::make_shared<StateSpace>(config_states_);
        model_factory_ = std::make_unique<MotionModelFactory>();
        if(!model_plugin.empty())
            model_ = model_factory_->createModelFromPlugin(model_plugin, states_);
        else
            model_ = model_factory_->createModel( static_cast<ModelType>(model_type),  states_);

        local_motion_model_ = model_.get();
        // hub_ = std::make_unique<Filter::MessageHub>(model_.get());
        filter_factory_ = std::make_unique<FilterFactory>();
        filter_ = filter_factory_->createFilter(static_cast<FilterType>(filter_type) , std::move(model_) , states_);
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
        // RCLCPP_INFO(this->get_logger() , "imu callback: %s" , topic_name_.c_str());

        // tf2::Quaternion quaternion;
        // tf2::fromMsg(msg->orientation, quaternion);
        // double roll, pitch, yaw;
        // tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger() , " ----- ");
        // RCLCPP_INFO(this->get_logger() , "state theta 0: %f" , yaw);

        Observations current_obs_;
        autodiff::MatrixXreal H;
        current_obs_.time_ = msg_->header.stamp;
        // RCLCPP_INFO(this->get_logger() , "Time:%f " , current_obs_.time_.seconds()); //rclcpp::Time store the double variable as seconod so no worries about the nano seconds stuff i guess
        auto index_ = states_->getStateOrder();
        // H.setZero(index.size() , index.size());
        H.setZero(states_->states_.size() , states_->states_.size());
        current_obs_.states_.setZero(states_->states_.size());
        // current_obs_.states_.setZero(index.size());
        // RCLCPP_INFO(rclcpp::get_logger("B") , "SIZE: %i" , index.size());
        for (auto sensor_state_ : sensor_states_[topic_name_])
        {
            RCLCPP_INFO(this->get_logger() ," HERE : %s" , sensor_state_.c_str());
            auto it_ = index_.find(sensor_state_);
            if(imu_state_action_.find(sensor_state_) != imu_state_action_.end())
            {
                imu_state_action_[sensor_state_](msg_ , current_obs_ , index_);
                H(it_->second , it_->second) = 1;
            }
            else{

            }

        }
        // for(int i = 0 ; i <current_obs_.states_.size(); i++)
        // {
        //     RCLCPP_INFO(this->get_logger() , "obs: %f" , current_obs_.states_(i));
        // }
        current_obs_.H = H;
        // RCLCPP_INFO_STREAM(this->get_logger() , H);
        // RCLCPP_INFO_STREAM(this->get_logger() , index.size());
        observations_.push(current_obs_);
        RCLCPP_INFO(this->get_logger() , "YAW_DOT IN IMU %f" , msg_->angular_velocity.z );

    }

    void FilterNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_, std::string topic_name_)
    {
        // RCLCPP_INFO(this->get_logger() , "odom callback: %s" , topic_name_.c_str());
        Observations current_obs_;
        autodiff::MatrixXreal H;
        auto cTime_ = this->now();
        std::cout<<"Current Time0: "<<cTime_.nanoseconds()<<"\n";
        std::cout<<"Message Time1: "<<msg_->header.stamp.sec<<" -- "<<msg_->header.stamp.nanosec<<"\n";
        rclcpp::Duration time_diff_ = cTime_ - msg_->header.stamp;
        std::cout<<"Time difference : "<< time_diff_.seconds()<<"\n";
        current_obs_.time_ = msg_->header.stamp;
        auto index_ = states_->getStateOrder();
        H.setZero(states_->states_.size(), states_->states_.size());
        current_obs_.states_.setZero(states_->states_.size());
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
        // for(int i = 0 ; i <current_obs_.states_.size(); i++)
        // {
        //     RCLCPP_INFO(this->get_logger() , "obs: %f" , current_obs_.states_(i));
        // }
        current_obs_.H = H;
        // RCLCPP_INFO_STREAM(this->get_logger() , H);
        // RCLCPP_INFO_STREAM(this->get_logger() , index.size());
        observations_.push(current_obs_);
            
        // ///////publishing yaw data from odom in a topic//////////
        tf2::Quaternion quaternion_;
        tf2::fromMsg(msg_->pose.pose.orientation, quaternion_);
        double roll_, pitch_, yaw_;
        tf2::Matrix3x3(quaternion_).getRPY(roll_, pitch_, yaw_);
        std_msgs::msg::Float64 d_;
        d_.data = yaw_;
        RCLCPP_INFO_STREAM(this->get_logger() , yaw_);
        yaw_odom_pub_->publish(d_);

        RCLCPP_INFO(this->get_logger() , "YAW IN ODOM %f" ,yaw_ );
        RCLCPP_INFO(this->get_logger() , "YAW_DOT IN ODOM %f" , msg_->twist.twist.angular.z );


        ///////////////////////////////
        // if (observations_.empty())
        // {
        //     RCLCPP_INFO(this->get_logger(), "No obs is in queue %i", observations_.size());
        // }

        // geometry_msgs::msg::Pose2D pose_;
        // while(!observations_.empty())
        // {
        //     // if(observations_.top().time_ > this->now())
        //     //     break;
        //     rclcpp::Time cur =msg->header.stamp;
        //     rclcpp::Duration dt_ = cur - previous_update_time_;
        //     filter_->predict(cTime_ , dt_);
        //     filter_->update(observations_.top());
        //     // RCLCPP_INFO(this->get_logger() , "some obs is in queue %i" , observations_.size());
        //     previous_update_time_ = cur;
        //     RCLCPP_INFO(this->get_logger() , "dt: %f" , dt_.seconds());
        //     observations_.pop();
        // }

        // autodiff::VectorXreal states_ = filter_->getStates();
        // std_msgs::msg::Float64 dd;
        // dd.data = states_(2).val();
        // yaw_filter_pub_->publish(dd);
    }


    // void FilterNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg , std::string topic_name_)
    // {
    //     // RCLCPP_INFO(this->get_logger() , "imu callback: %s" , topic_name_.c_str());

    //     // tf2::Quaternion quaternion;
    //     // tf2::fromMsg(msg->orientation, quaternion);
    //     // double roll, pitch, yaw;
    //     // tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    //     // RCLCPP_INFO(this->get_logger() , " ----- ");
    //     // RCLCPP_INFO(this->get_logger() , "state theta 0: %f" , yaw);

    //     Observations current_obs_;
    //     autodiff::MatrixXreal H;
    //     current_obs_.time_ = msg->header.stamp;
    //     // RCLCPP_INFO(this->get_logger() , "Time:%f " , current_obs_.time_.seconds()); //rclcpp::Time store the double variable as seconod so no worries about the nano seconds stuff i guess
    //     auto index = states_->getStateOrder();
    //     // H.setZero(index.size() , index.size());
    //     H.setZero(states_->states_.size() , states_->states_.size());
    //     current_obs_.states_.setZero(states_->states_.size());
    //     // current_obs_.states_.setZero(index.size());
    //     // RCLCPP_INFO(rclcpp::get_logger("B") , "SIZE: %i" , index.size());
    //     for (auto sensor_state_ : sensor_states_[topic_name_])
    //     {
    //         auto it = index.find(sensor_state_);
    //         if (sensor_state_ == "yaw_dot")
    //         {
    //             current_obs_.states_(it->second) = msg->angular_velocity.z;
    //             RCLCPP_INFO(this->get_logger() , "yaw dot: %f" , current_obs_.states_(it->second));
    //         }
    //         else if (sensor_state_ == "x_ddot")
    //         {
    //             current_obs_.states_(it->second) = msg->linear_acceleration.x;
    //         }
    //         else if ( sensor_state_ =="roll" || sensor_state_ =="pitch" || sensor_state_ == "yaw" ){
    //             tf2::Quaternion quaternion;
    //             tf2::fromMsg(msg->orientation, quaternion);
    //             double roll, pitch, yaw;
    //             tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    //             if(sensor_state_ =="roll")
    //                 current_obs_.states_(it->second) = roll;
    //             else if(sensor_state_ =="pitch")
    //                 current_obs_.states_(it->second) = pitch;
    //             else if(sensor_state_ =="yaw")
    //                 current_obs_.states_(it->second) = yaw;
    //         }
    //         else{

    //         }

    //         H(it->second , it->second) = 1;
    //     }
    //     // for(int i = 0 ; i <current_obs_.states_.size(); i++)
    //     // {
    //     //     RCLCPP_INFO(this->get_logger() , "obs: %f" , current_obs_.states_(i));
    //     // }
    //     current_obs_.H = H;
    //     // RCLCPP_INFO_STREAM(this->get_logger() , H);
    //     // RCLCPP_INFO_STREAM(this->get_logger() , index.size());
    //     observations_.push(current_obs_);
    //     RCLCPP_INFO(this->get_logger() , "YAW_DOT IN IMU %f" , msg->angular_velocity.z );

    // }

    // void FilterNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, std::string topic_name_)
    // {
    //     // RCLCPP_INFO(this->get_logger() , "odom callback: %s" , topic_name_.c_str());
    //     Observations current_obs_;
    //     autodiff::MatrixXreal H;
    //     auto cTime_ = this->now();
    //     std::cout<<"Current Time0: "<<cTime_.nanoseconds()<<"\n";
    //     std::cout<<"Message Time1: "<<msg->header.stamp.sec<<" -- "<<msg->header.stamp.nanosec<<"\n";
    //     rclcpp::Duration time_diff_ = cTime_ - msg->header.stamp;
    //     std::cout<<"Time difference : "<< time_diff_.seconds()<<"\n";
    //     current_obs_.time_ = msg->header.stamp;
    //     auto index = states_->getStateOrder();
    //     H.setZero(states_->states_.size(), states_->states_.size());
    //     current_obs_.states_.setZero(states_->states_.size());
    //     for (auto sensor_state_ : sensor_states_[topic_name_])
    //     {
    //         auto it = index.find(sensor_state_);
    //         if (sensor_state_ == "x")
    //         {
    //             current_obs_.states_(it->second) = msg->pose.pose.position.x;
    //         }
    //         else if (sensor_state_ == "y")
    //         {
    //             current_obs_.states_(it->second) = msg->pose.pose.position.y;
    //         }
    //         else if (sensor_state_ == "z")
    //         {
    //             current_obs_.states_(it->second) = msg->pose.pose.position.z;
    //         }
    //         else if (sensor_state_ == "roll" || sensor_state_ == "pitch" || sensor_state_ == "yaw")
    //         {
    //             tf2::Quaternion quaternion;
    //             tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    //             double roll, pitch, yaw;
    //             tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    //             if (sensor_state_ == "roll")
    //                 current_obs_.states_(it->second) = roll;
    //             else if (sensor_state_ == "pitch")
    //                 current_obs_.states_(it->second) = pitch;
    //             else if (sensor_state_ == "yaw")
    //                 current_obs_.states_(it->second) = yaw;
    //         }
    //         else if (sensor_state_ == "x_dot")
    //         {
    //             current_obs_.states_(it->second) = msg->twist.twist.linear.x;
    //         }
    //         else if (sensor_state_ == "y_dot")
    //         {
    //             current_obs_.states_(it->second) = msg->twist.twist.linear.y;
    //         }
    //         else if (sensor_state_ == "z_dot")
    //         {
    //             current_obs_.states_(it->second) = msg->twist.twist.linear.z;
    //         }
    //         else if (sensor_state_ == "roll_dot")
    //         {
    //             current_obs_.states_(it->second) = msg->twist.twist.angular.x;
    //         }
    //         else if (sensor_state_ == "pitch_dot")
    //         {
    //             current_obs_.states_(it->second) = msg->twist.twist.angular.y;
    //         }
    //         else if (sensor_state_ == "yaw_dot")
    //         {
    //             current_obs_.states_(it->second) = msg->twist.twist.angular.z;
    //         }
    //         else
    //         {
    //         }

    //         H(it->second , it->second) = 1;
    //     }
    //     // for(int i = 0 ; i <current_obs_.states_.size(); i++)
    //     // {
    //     //     RCLCPP_INFO(this->get_logger() , "obs: %f" , current_obs_.states_(i));
    //     // }
    //     current_obs_.H = H;
    //     // RCLCPP_INFO_STREAM(this->get_logger() , H);
    //     // RCLCPP_INFO_STREAM(this->get_logger() , index.size());
    //     observations_.push(current_obs_);
            
    //     // ///////publishing yaw data from odom in a topic//////////
    //     tf2::Quaternion quaternion;
    //     tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    //     double roll, pitch, yaw;
    //     tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    //     std_msgs::msg::Float64 d;
    //     d.data = yaw;
    //     RCLCPP_INFO_STREAM(this->get_logger() , yaw);
    //     yaw_odom_pub_->publish(d);

    //     RCLCPP_INFO(this->get_logger() , "YAW IN ODOM %f" ,yaw );
    //     RCLCPP_INFO(this->get_logger() , "YAW_DOT IN ODOM %f" , msg->twist.twist.angular.z );


    //     ///////////////////////////////
    //     // if (observations_.empty())
    //     // {
    //     //     RCLCPP_INFO(this->get_logger(), "No obs is in queue %i", observations_.size());
    //     // }

    //     // geometry_msgs::msg::Pose2D pose_;
    //     // while(!observations_.empty())
    //     // {
    //     //     // if(observations_.top().time_ > this->now())
    //     //     //     break;
    //     //     rclcpp::Time cur =msg->header.stamp;
    //     //     rclcpp::Duration dt_ = cur - previous_update_time_;
    //     //     filter_->predict(cTime_ , dt_);
    //     //     filter_->update(observations_.top());
    //     //     // RCLCPP_INFO(this->get_logger() , "some obs is in queue %i" , observations_.size());
    //     //     previous_update_time_ = cur;
    //     //     RCLCPP_INFO(this->get_logger() , "dt: %f" , dt_.seconds());
    //     //     observations_.pop();
    //     // }

    //     // autodiff::VectorXreal states_ = filter_->getStates();
    //     // std_msgs::msg::Float64 dd;
    //     // dd.data = states_(2).val();
    //     // yaw_filter_pub_->publish(dd);
    // }

    void FilterNode::controlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger() , "CmdVel callback");
        if(local_motion_model_)
            local_motion_model_->setVelAndAngVelFromTwist(*msg);
        
    }

    void FilterNode::timerCallback()
    {
        static rclcpp::Time pre_time_ =  rclcpp::Clock().now();
        rclcpp::Time cur_time_ = rclcpp::Clock().now();
        // RCLCPP_INFO(this->get_logger() , "TIME0: %f" , this->now().seconds()); // Nodes time which depends on use_sim_time variable
        // RCLCPP_INFO(this->get_logger() , "TIME1: %f" , rclcpp::Clock().now().seconds()); //Real time - Unix time
        if(observations_.empty())
        {
            RCLCPP_INFO(this->get_logger() , "No obs is in queue %i" , observations_.size());
            // auto cur_time_ = this->now();
            // auto dt_ = cur_time_ - previous_update_time_;
            // filter_->predict(cur_time_,dt_ );
            // previous_update_time_ = cur_time_;
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
            // RCLCPP_INFO(this->get_logger() , "some obs is in queue %i" , observations_.size());
            previous_update_time_ = cur_obs_time_;
            RCLCPP_INFO(this->get_logger(), "dt: %f", dt_.seconds());
            observations_.pop();
            ///////lookup transform for now its for debugging purpose////////////////////
            // try
            // {
            //     geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            //         "odom", "base_link", tf2::TimePointZero);

            //     // RCLCPP_INFO(this->get_logger(), "Base link pose in odom frame: %f %f %f",
            //     //             transform.transform.translation.x,
            //     //             transform.transform.translation.y,
            //     //             transform.transform.translation.z);
            //     double roll, pitch, yaw;
            //     tf2::Quaternion quat;
            //     tf2::fromMsg(transform.transform.rotation, quat);
            //     tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            //     RCLCPP_INFO(this->get_logger(), "Yaw angle of base_link w.r.t. odom: %f", yaw);
            // }
            // catch (tf2::TransformException &ex)
            // {
            //     RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
            // }
        }

        ///////////////////publishing filtered data into a topic///////////// 
        autodiff::VectorXreal sta_ = filter_->getStates();
        filtered_odom_.header.frame_id = odom_frame_;
        filtered_odom_.child_frame_id = base_link_frame_;
        filtered_odom_.header.stamp = this->now();
        filtered_odom_.pose.pose.position.x = sta_(0).val();
        filtered_odom_.pose.pose.position.y = sta_(1).val();


        tf2::Quaternion quaternion_;
        quaternion_.setRPY(0.0,0.0,sta_(2).val());
        // int maxIndex = 0;
        // for (int i = 1; i < 4; ++i) {
        //     if (std::fabs(quaternion_[i]) > std::fabs(quaternion_[maxIndex])) {
        //         maxIndex = i;
        //     }
        // }

        //     RCLCPP_INFO(this->get_logger() , "MAX INDEX %i" , maxIndex);
        // https://stackoverflow.com/questions/72219304/eliminating-sign-flips-in-quaternion-data-from-sensors
        // if (quaternion_[maxIndex] < 0.0) {
        //     quaternion_[maxIndex] = -quaternion_[maxIndex];
        // }


        geometry_msgs::msg::Quaternion orientation_msg_;
        tf2::convert(quaternion_ , orientation_msg_);
        filtered_odom_.pose.pose.orientation = orientation_msg_;
        filtered_odom_pub_->publish(filtered_odom_);
        // /////////////////////publishing yaw in a topic//////////////////
        autodiff::VectorXreal st_ = filter_->getStates();
        std_msgs::msg::Float64 d;
        d.data = st_(2).val();
        yaw_filter_pub_->publish(d);

        // ////////////////////visualization//////////////////
        // autodiff::VectorXreal states_ = filter_->getStates();

        // pose_.x = states_(0).val();
        // pose_.y = states_(1).val();
        // pose_.theta = states_(2).val();
        // // RCLCPP_INFO(this->get_logger() , "x,y,theta: %f , %f, %f" , pose_.x , pose_.y , pose_.theta);
        // RCLCPP_INFO(this->get_logger() , "state theta 1: %f" , pose_.theta);
        // visualization_->addArrow(pose_);
        // visualization_->publishArrow();
        // visualization_->initialize();


        RCLCPP_INFO_STREAM(rclcpp::get_logger("rate logger") , 1/(cur_time_-pre_time_).seconds()); // *
        pre_time_ = cur_time_;

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
            rclcpp::Duration duration(0, 10000000000); //10000 ms
            odom_base_link_transform_.header.stamp = this->now();// - duration;
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