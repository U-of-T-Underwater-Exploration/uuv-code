#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>

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
Thruster thruster_0;
Thruster thruster_1;
Thruster thruster_2;
Thruster thruster_3;
Thruster thruster_4;
Thruster thruster_5;
Thruster thruster_6;
Thruster thruster_7;
std::vector<Thruster> thrusters = {thruster_0, thruster_1, thruster_2, thruster_3, thruster_4, thruster_5, thruster_6, thruster_7};

class MotionConverter : public rclcpp::Node
{
    public:

        // Function to handle reading YAML file and storing parameters for each thruster
        void read_params(Thruster thruster, std::string thruster_name){
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
            // thruster_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thruster/command", 10);
            
            // Reading and storing parameters for each thruster
            for (int i = 0; i < thrusters.size(); i++){
                std::string thruster_name = "thrusters.thruster_";
                thruster_name += std::to_string(i);
                read_params(thrusters[i], thruster_name);
            }
        }
    private:

    // Eigen::VectorXf thrust_vec;

    // // Call back function: Converts joystick inputs into motor thrust vector 
    // void insert_callback_function_name_here(const std_msgs::msg::Float32MultiArray::SharedPtr msg){

    //     // thrust_vec =  
    // }


    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motion_cmd_sub_;
    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_cmd_pub_;
    };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionConverter>());
  rclcpp::shutdown();
  return 0;
}