#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "uuv_joystick_msgs/msg/uuv_command.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <array>

// Struct for storing thruster info
struct Thruster
    {
        int id;
        double max_forward_thrust;
        double max_reverse_thrust;
        // double deadband;   <-- Not sure if we need this for each thruster
        bool enabled;
        bool inverted;

        Eigen::VectorXd p_motor_offset;
        Eigen::VectorXd r_motor_dir;
    };

//Creating vector for thrusters to be able access each thruster later on
std::array<Thruster, 8> thrusters;

class MotionConverter : public rclcpp::Node
{
    public:

        // Function to handle reading YAML file and storing parameters for each thruster
        void read_params(Thruster & thruster, std::string thruster_name){
            std::string id = thruster_name + ".id";
            std::string fwd_thst = thruster_name + ".max_forward_thrust";
            std::string rev_thst = thruster_name + ".max_reverse_thrust";
            // std::string db = thruster_name + ".deadband";
            std::string enable = thruster_name + ".enabled";
            std::string inverted = thruster_name + ".inverted";
            std::string motor_offset = thruster_name + ".p_motor_offset";
            std::string motor_dir = thruster_name + ".r_motor_dir";

            // Declaring params:
            thruster.id = this->declare_parameter<int>(id);
            thruster.max_forward_thrust = this->declare_parameter<double>(fwd_thst);
            thruster.max_reverse_thrust = this->declare_parameter<double>(rev_thst);
            // thruster.deadband = this->declare_parameter<double>(db);
            thruster.enabled = this->declare_parameter<bool>(enable);
            thruster.inverted = this->declare_parameter<bool>(inverted);

            this->declare_parameter<std::vector<double>>(motor_offset);
            this->declare_parameter<std::vector<double>>(motor_dir);

            // converting vector params to vector data type
            std::vector<double> offset_vec = this->get_parameter(motor_offset).as_double_array();
            std::vector<double> motor_vec = this->get_parameter(motor_dir).as_double_array();
            thruster.p_motor_offset = Eigen::Map<Eigen::VectorXd>(offset_vec.data(), offset_vec.size());
            thruster.r_motor_dir = Eigen::Map<Eigen::VectorXd>(motor_vec.data(), motor_vec.size());        
        }


        MotionConverter()
        : Node("motion_converter_node")
        {   
            // // Subscribe to 'motion/command' joystick inputs
            motion_cmd_sub_ = this->create_subscription<uuv_joystick_msgs::msg::UUVCommand>(
                "/input/command", 10, std::bind(&MotionConverter::get_wrench_callback, this, std::placeholders::_1));

            // // Publish to '/thruster/command' 
            thruster_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thruster/command", 10);
            
            // Reading and storing parameters for each thruster
            for (size_t i = 0; i < thrusters.size(); i++){
                std::string thruster_name = "thrusters.thruster_";
                thruster_name += std::to_string(i);
                read_params(thrusters[i], thruster_name);
            }

            get_pseudo_inverse();

            find_max_wrench(); //call find_max_wrench to find max wrench
        }

    private:

    Eigen::VectorXf thrust_vec;
    Eigen::MatrixXf motion_converter_matrix_pinv;

    float max_surge_force;
    float max_sway_force;
    float max_heave_force;
    float max_roll_moment;
    float max_pitch_moment;
    float max_yaw_moment;

    void find_max_wrench() {
        //set them all and 0 first
        max_surge_force = 0.0;
        max_sway_force = 0.0;
        max_heave_force = 0.0;
        max_roll_moment = 0.0;
        max_pitch_moment = 0.0;
        max_yaw_moment = 0.0;
        
        for (int i = 0; i < 8; i++){
            //TODO: Math to find max wrench here

            Eigen::Vector3f r_motor_dir = thrusters[i].r_motor_dir.cast<float>();
            Eigen::Vector3f p_motor_offset = thrusters[i].p_motor_offset.cast<float>();

            //find max forward thrust for each thruster(in YAML, the max_reverse_thrust is just -max_forward_thrust)
            float max_thrust = static_cast<float>(thrusters[i].max_forward_thrust);

            //mult max_thrust by the p_motor_offset and add to each max force
            max_surge_force += std::abs(r_motor_dir(0)*max_thrust);
            max_sway_force += std::abs(r_motor_dir(1)*max_thrust);
            max_heave_force += std::abs(r_motor_dir(2)*max_thrust);

            //cross product of position and force direction is moment
            Eigen::Vector3f moment = p_motor_offset.cross(r_motor_dir);

            //scale moments by the max_thrust
            max_roll_moment += std::abs(moment(0)*max_thrust);
            max_pitch_moment += std::abs(moment(1)*max_thrust);
            max_yaw_moment += std::abs(moment(2)*max_thrust);

        }
    }

    //Call back function: Converts joystick inputs into motor thrust vector 
    void get_wrench_callback(const uuv_joystick_msgs::msg::UUVCommand::SharedPtr msg){
        // TODO: Implement callback logic
        // Suppress unused parameter warning
        (void)msg;
        
        Eigen::VectorXf motion_cmd(6);
        // TODO: Extract motion commands from msg
        motion_cmd(0) = msg->surge;
        motion_cmd(1) = msg->sway;
        motion_cmd(2) = msg->heave;
        motion_cmd(3) = msg->roll;
        motion_cmd(4) = msg->pitch;
        motion_cmd(5) = msg->yaw;

        for (size_t i = 0; i < msg->actions.size(); i++){
            if (msg->actions[i].action == 4){
                motion_cmd(2) *= -1;
            }
            if (msg->actions[i].action == 5){
                motion_cmd(5) *= -1;
            }
        }

        Eigen::VectorXf wrench(6);
        // TODO: Convert motion commands to wrench
        //scale each max force or moment by their percent
        wrench(0) = motion_cmd(0)*max_surge_force;
        wrench(1) = motion_cmd(1)*max_sway_force;
        wrench(2) = motion_cmd(2)*max_heave_force;
        wrench(3) = motion_cmd(3)*max_roll_moment;
        wrench(4) = motion_cmd(4)*max_pitch_moment;
        wrench(5) = motion_cmd(5)*max_yaw_moment;

        thrust_vec =  motion_converter_matrix_pinv * wrench;

        std::array<float, 8> motor_thrust_vec;
        for (int i = 0; i < 8; i++){
            motor_thrust_vec[i] = thrust_vec(i);
        }
        
        publish_motor_percentage(motor_thrust_vec);
    }

    void get_pseudo_inverse(){

        Eigen::MatrixXf motion_converter_matrix(6,8);
        // Fill allocation_matrix based on thruster configurations
        for (size_t i = 0; i < thrusters.size(); i++){
            Eigen::Vector3f r_motor_dir = thrusters[i].r_motor_dir.cast<float>();
            Eigen::Vector3f p_motor_offset = thrusters[i].p_motor_offset.cast<float>();

            // Force components
            motion_converter_matrix(0, i) = r_motor_dir(0); // Surge
            motion_converter_matrix(1, i) = r_motor_dir(1); // Sway
            motion_converter_matrix(2, i) = r_motor_dir(2); // Heave

            // Moment components
            Eigen::Vector3f moment = p_motor_offset.cross(r_motor_dir);
            motion_converter_matrix(3, i) = moment(0); // Roll
            motion_converter_matrix(4, i) = moment(1); // Pitch
            motion_converter_matrix(5, i) = moment(2); // Yaw
        }

        motion_converter_matrix_pinv = motion_converter_matrix.completeOrthogonalDecomposition().pseudoInverse();
    }


    void publish_motor_percentage(std::array<float, 8> motor_thrust_vec){
        auto thruster_cmd = std_msgs::msg::Float32MultiArray();
        thruster_cmd.data.resize(8);
        
        float factor = 1.0f;
        for (int i = 0; i < 8; i++){
            factor = std::max(factor, normalize_factor(motor_thrust_vec[i], thrusters[i]));
        }

        // Mapping motor thrusts to duty cycles
        for(int i = 0; i < 8; i++){
            // normalizing thrust
            float normalized_thrust = motor_thrust_vec[i] / factor;
            float motor_percentage = thrust_mapping(normalized_thrust, thrusters[i]);
            
            // Check if duty cycle is out of bounds and clamp
            if (motor_percentage < -1.0f || motor_percentage > 1.0f) {
                if (motor_percentage < -1.01f || motor_percentage > 1.01f) {
                    RCLCPP_ERROR(this->get_logger(), "Thruster %d duty cycle out of bounds: %.6f. Clamping to [-1, 1]", i, motor_percentage);
                }
                motor_percentage = std::clamp(motor_percentage, -1.0f, 1.0f);
            }
            
            thruster_cmd.data[i] = motor_percentage;
        }

        thruster_cmd_pub_->publish(thruster_cmd);
    }

    // Linear mapping from thrust to thrust_percentage
    float thrust_mapping(float thrust, const Thruster& thruster){
        if (thrust > 0.0f) {
            if (thruster.max_forward_thrust == 0.0) {
                RCLCPP_WARN(this->get_logger(), "Max forward thrust is zero for thruster %d", thruster.id);
                return 0.0f;
            }
            return (thrust / static_cast<float>(thruster.max_forward_thrust));
        }
        else {
            if (thruster.max_reverse_thrust == 0.0) {
                RCLCPP_WARN(this->get_logger(), "Max reverse thrust is zero for thruster %d", thruster.id);
                return 0.0f;
            }
            return (thrust / static_cast<float>(-thruster.max_reverse_thrust));
        }
    }

    float normalize_factor(float thrust, const Thruster& thruster){
        if (thrust > 0.0f) {
            if (thruster.max_forward_thrust == 0.0) {
                RCLCPP_WARN(this->get_logger(), "Max forward thrust is zero for thruster %d", thruster.id);
                return 1.0f;
            }
            return thrust / static_cast<float>(thruster.max_forward_thrust);
        } else {
            if (thruster.max_reverse_thrust == 0.0) {
                RCLCPP_WARN(this->get_logger(), "Max reverse thrust is zero for thruster %d", thruster.id);
                return 1.0f;
            }
            return (thrust) / static_cast<float>(thruster.max_reverse_thrust);
        }
    }

    rclcpp::Subscription<uuv_joystick_msgs::msg::UUVCommand>::SharedPtr motion_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_cmd_pub_;
    };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionConverter>());
  rclcpp::shutdown();
  return 0;
}