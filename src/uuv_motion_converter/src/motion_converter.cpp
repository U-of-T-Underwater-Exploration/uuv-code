#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>


class MotionConverter : public rclcpp::Node
{
    public:
        MotionConverter()
        : Node("motion_converter_node")
        {
            // // Subscribe to 'motion/command' joystick inputs
            // motion_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            //     "/motion/command", 10, std::bind(&MotionConverter::insert_callback_function_name_here, this, std::placeholders::_1));

            // // Publish to '/thruster/command' 
            // thruster_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/thruster/command", 10);
            Eigen::VectorXf b = Eigen::VectorXf::Random(6);
            RCLCPP_INFO(this->get_logger(), "Random Eigen vector: [%f, %f, %f, %f, %f, %f]", b(0), b(1), b(2), b(3), b(4), b(5));
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