#include "medsnake_control.h"


MedsnakeControl::MedsnakeControl(const char* port_name, const char* config_path, const char* dxl_config_path) : snake_(port_name)
{
  init_listener();
  init_tension_publisher();
  snake_.initialize(config_path, dxl_config_path);
};

void MedsnakeControl::command_set(const std_msgs::Char::ConstPtr& msg)
{ // callback function
  ROS_INFO("key pressed: [%c]", char(msg->data));
  char key = msg->data;

  if(key != 'w' && key != 's' && 
     key != 'a' && key != 'd' && 
     key != 'y' && key != 'h' && 
     key != 't' && key != 'g' && 
     key != 'v' && key != 'b' && 
     key != 'e' && key != 'c' && 
     key != 'u' && key != 'm' && 
     key != 'o' && key != 'q' &&
     key != 'i' && key != 'j' && 
     key != 'k' && key != 'p' && 
     key != 'n' && key != 'l' &&
     key != ',')
  {
    ROS_INFO("[%c] is not a valid command", key);  
  }
  else if (key == 'o') 
  {
    command_queue_.clear();
    command_queue_.push_back(key);
  }
  else if (snake_.is_ready()) // is command that's not stop and snake is ready
    command_queue_.push_back(key);
  else
    ROS_INFO("Last command is executing, invalid key press");
}

void MedsnakeControl::init_listener()
{ // init subscriber
  command_sub_ = nh_.subscribe("gui_commands", 100, &MedsnakeControl::command_set, this);
}

void MedsnakeControl::init_tension_publisher()
{
  tension_pub_ = nh_.advertise<medical_snake::Tension_readings>("tension_readings", 1);
  mode_pub_ = nh_.advertise<std_msgs::String>("medsnake_mode", 1);
}

void MedsnakeControl::snake_update() {snake_.update();}

void MedsnakeControl::publish_tension_reading()
{
  // publish updated tension readings message
  tension_dic_ = snake_.get_tension_fbk();
  medical_snake::Tension_readings tension_msg;
  tension_msg.header.stamp = ros::Time::now();
  tension_msg.inner_snake_cable = tension_dic_["inner_snake_cable"];
  tension_msg.outer_snake_cable_A = tension_dic_["outer_snake_cable_A"];
  tension_msg.outer_snake_cable_B = tension_dic_["outer_snake_cable_B"];
  tension_msg.outer_snake_cable_C = tension_dic_["outer_snake_cable_C"];

  tension_pub_.publish(tension_msg);
}

void MedsnakeControl::publish_snake_mode() {
  std_msgs::String mode_msg;
  mode_msg.data = snake_.get_snake_mode();
  mode_pub_.publish(mode_msg);
}

void MedsnakeControl::emergency_stop()
{
  command_queue_.erase(command_queue_.begin());
  snake_.stop_all_motor();
}

void MedsnakeControl::demo()
{
  command_queue_.erase(command_queue_.begin());

for (int i = 0; i < 2; i++)
{
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}
for (int i = 0; i < 4; i++)
{
  command_queue_.push_back('y'); // Steer Up
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}
for (int i = 0; i < 2; i++)
{
  command_queue_.push_back('h'); // Steer Down
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}
for (int i = 0; i < 2; i++)
{
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}
}

void MedsnakeControl::advance()
{
  command_queue_.erase(command_queue_.begin());
  // command_queue_.clear();
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('e'); // Forward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('u'); // Forward Outer
}

void MedsnakeControl::retract()
{
  command_queue_.erase(command_queue_.begin());
  // command_queue_.clear();
  command_queue_.push_back('t'); // Tighten Outer
  command_queue_.push_back('b'); // Loosen Inner
  command_queue_.push_back('c'); // Backward Inner
  command_queue_.push_back('v'); // Tighten Inner
  command_queue_.push_back('g'); // Loosen Outer
  command_queue_.push_back('m'); // Backward Outer
}

void MedsnakeControl::steer_left()
{
  snake_.steer_left();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::steer_right()
{
  snake_.steer_right();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::steer_up()
{
  snake_.steer_up();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::steer_down()
{
  snake_.steer_down();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::tighten_outer()
{
  snake_.tighten_outer();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::loosen_outer()
{
  snake_.loosen_outer();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::tighten_inner()
{
  snake_.tighten_inner();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::loosen_inner()
{
  snake_.loosen_inner();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::forward_inner()
{
  snake_.forward_inner();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::backward_inner()
{
  snake_.backward_inner();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::forward_outer()
{
  snake_.forward_outer();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::backward_outer()
{
  snake_.backward_outer();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::home_rail()
{
  snake_.home_rail();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::tighten_outer_A()
{
  snake_.tighten_outer_A();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::tighten_outer_B()
{
  snake_.tighten_outer_B();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::tighten_outer_C()
{
  snake_.tighten_outer_C();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::loosen_outer_A()
{
  snake_.loosen_outer_A();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::loosen_outer_B()
{
  snake_.loosen_outer_B();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::loosen_outer_C()
{
  snake_.loosen_outer_C();
  command_queue_.erase(command_queue_.begin());
}

void MedsnakeControl::tension_control_inner()
{
  snake_.tension_control_inner();
  command_queue_.erase(command_queue_.begin());
}




bool MedsnakeControl::cmd_queue_empty() {return command_queue_.empty();}

char MedsnakeControl::get_cmd_queue_top() {return command_queue_[0];}

bool MedsnakeControl::snake_is_ready() {return snake_.is_ready();}

