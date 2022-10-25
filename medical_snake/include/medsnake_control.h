/*
 * A ros node that subscribe to user issued command and call medical snake 
 * command functions to move the snake. This node also publishes the smoothed
 * current reading from medsnake dynamixel motors and publish the reading 
 * under the topic /tension_readings
 */

#include <string>
#include <ros/ros.h>
#include "std_msgs/Char.h"
#include "std_msgs/String.h"
#include "medical_snake/Tension_readings.h"
#include "medical_snake.h"



class MedsnakeControl
{

 public:
  /// Constructor for SnakeControl instance
  MedsnakeControl(const char* port_name, const char* config_path,
                       const char* dxl_config_path);

  /// initialize the snake
  void snake_initialize(const char* config_path, const char* dxl_config_path);
  
  void command_set(const std_msgs::Char::ConstPtr& msg);
  
  void init_listener();

  void init_tension_publisher();

  void snake_update();

  void publish_tension_reading();

  void publish_snake_mode();

  void emergency_stop();

  void demo();

  void advance();

  void retract();

  void steer_left();

  void steer_right();

  void steer_up();

  void steer_down();

  void tighten_outer();

  void loosen_outer();

  void tighten_inner();

  void loosen_inner();

  void forward_inner();

  void backward_inner();

  void forward_outer();

  void backward_outer();

  void home_rail();

  void tighten_outer_A();

  void tighten_outer_B();

  void tighten_outer_C();

  void loosen_outer_A();

  void loosen_outer_B();

  void loosen_outer_C();

  void tension_control_inner();


  bool cmd_queue_empty();

  char get_cmd_queue_top();

  bool snake_is_ready();

 private:
  MedicalSnake snake_;
  ros::Subscriber command_sub_;
  ros::Publisher tension_pub_;
  ros::Publisher mode_pub_;
  ros::NodeHandle nh_;
  std::vector<char> command_queue_;
  std::map<std::string, double> tension_dic_;
};
