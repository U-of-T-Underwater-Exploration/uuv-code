#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class ThrusterDriver : public rclcpp::Node
{
    public:
        ThrusterDriver()
        : Node("thruster_driver_node")
        {
            // Subscribe to '/thruster/command' for duty cycle for thrusters
            command_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/thruster/command", 10, std::bind(&ThrusterDriver::handle_thruster_cmd, this, std::placeholders::_1));

            // Publisher
            dty_cycl_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pwm/thrusters", 10);
        }
    
    private:

        float dty_cycl_mapping(const float cmd_thrust){
            // Setting up max and min duty cycles for respective fwd and rev thrust (In case variables change)
            const int freq = 50;    //The frequency of PWM cycles in Hz

            const float max_dty_cycl = (2000) * freq /1000000.0f; //For full fwd thrust (2000 microseconds pulse width)
            const float min_dty_cycl = (1000) * freq /1000000.0f; //For full rev thrust (1000 microseconds pulse width)

            // Return duty cycle, assuming linear mapping for now (-1.0f -> min_dty_cycl, 1.0f -> max_dty_cycl).         
            // Can modify code later, to account for S-curve, non-linear mapping, etc.

            return min_dty_cycl + ((cmd_thrust + 1.0f) / (2.0f)) * (max_dty_cycl - min_dty_cycl);
        }

        void handle_thruster_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            const auto length = msg->data.size();
            auto dty_cycl = std_msgs::msg::Float32MultiArray();      //Intializing message to be published
            dty_cycl.data.resize(8); 
            
            if(length == 8){
                for(int i = 0; i < 8; i++){
                    float cmd_thrust = msg->data[i];

                    // If the received thruster cmd is valid, map cmd to corresponding pwm duty cycle
                    // Where -1.0 -> 
                    if(-1.0f <= cmd_thrust && cmd_thrust <= 1.0f){
                        dty_cycl.data[i] = dty_cycl_mapping(cmd_thrust);
                    }
                    
                    // In the case the received thruster cmd is outside the range of [-1.0, 1.0]
                    // print out an error and clamp the output to either 0.0 or 1.0 limits 
                    else if (cmd_thrust < -1.0f)
                    {
                        RCLCPP_ERROR(this->get_logger(), 
                        "Received thrust command below -1.0. Clamped to -1.0. RECEIVED: %f", cmd_thrust);

                        dty_cycl.data[i] = dty_cycl_mapping(-1.0f);
                    }
                    
                    else if (cmd_thrust > 1.0f)
                    {
                        RCLCPP_ERROR(this->get_logger(), 
                        "Received thrust command above 1.0. Clamped to 1.0. RECEIVED: %f", cmd_thrust);

                        dty_cycl.data[i] = dty_cycl_mapping(1.0f);
                    }
                }

            dty_cycl_pub_->publish(dty_cycl);
            }   

            else{
                RCLCPP_ERROR(this->get_logger(), "Incorrect array size. Expected 8. RECEIVED: %zu", length);
            }
        }

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dty_cycl_pub_;
    };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterDriver>());
  rclcpp::shutdown();
  return 0;
}