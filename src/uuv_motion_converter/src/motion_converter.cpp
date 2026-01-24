#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
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
            // motion_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            //     "/motion/command", 10, std::bind(&MotionConverter::insert_callback_function_name_here, this, std::placeholders::_1));

            // // Publish to '/thruster/command' 
            thruster_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thruster/command", 10);
            
            // Reading and storing parameters for each thruster
            for (int i = 0; i < thrusters.size(); i++){
                std::string thruster_name = "thrusters.thruster_";
                thruster_name += std::to_string(i);
                read_params(thrusters[i], thruster_name);
            }

            get_pseudo_inverse();
        }
    private:

    Eigen::VectorXf thrust_vec;
    Eigen::MatrixXf motion_converter_matrix_pinv;

    //Call back function: Converts joystick inputs into motor thrust vector 
    void insert_callback_function_name_here(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
        // TODO: Implement callback logic
        // Suppress unused parameter warning
        (void)msg;
        
        Eigen::VectorXf motion_cmd(6);
        // TODO: Extract motion commands from msg

        Eigen::VectorXf wrench(6);
        // TODO: Convert motion commands to wrench

        thrust_vec =  motion_converter_matrix_pinv * wrench;
    }

    void get_pseudo_inverse(){

        Eigen::MatrixXf motion_converter_matrix(8,6);
        // Fill allocation_matrix based on thruster configurations
        for (int i = 0; i < thrusters.size(); i++){
            Eigen::Vector3f r_motor_dir = thrusters[i].r_motor_dir.cast<float>();
            Eigen::Vector3f p_motor_offset = thrusters[i].p_motor_offset.cast<float>();

            // Force components
            motion_converter_matrix(i, 0) = r_motor_dir(0); // Surge
            motion_converter_matrix(i, 1) = r_motor_dir(1); // Sway
            motion_converter_matrix(i, 2) = r_motor_dir(2); // Heave

            // Moment components
            Eigen::Vector3f moment = p_motor_offset.cross(r_motor_dir);
            motion_converter_matrix(i, 3) = moment(0); // Roll
            motion_converter_matrix(i, 4) = moment(1); // Pitch
            motion_converter_matrix(i, 5) = moment(2); // Yaw
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
                RCLCPP_ERROR(this->get_logger(), "Thruster %d duty cycle out of bounds: %.3f. Clamping to [-1, 1]", i, motor_percentage);
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
            return (thrust / static_cast<float>(thruster.max_reverse_thrust));
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
            return (-thrust) / static_cast<float>(thruster.max_reverse_thrust);
        }
    }

    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motion_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_cmd_pub_;
    };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionConverter>());
  rclcpp::shutdown();
  return 0;
}