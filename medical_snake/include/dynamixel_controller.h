/*******************************************************************************
* Copyright Biorobics Lab, Carnegie Mellon University
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Nico Zevallos, Yilin Cai */

#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include <map>
#include <chrono>
#include <vector>
#include <dynamixel_sdk.h>  // Uses Dynamixel SDK library

// template<class T>
typedef struct {
  uint16_t    address;
  uint16_t    data_length;
} Register;

typedef std::map<std::string, Register> ControlTable;

// Mapping from name to address and data length as defined in
// https://emanual.robotis.com/docs/en/dxl/pro/m42-10-s260-ra/#bus-watchdog
const ControlTable ctrl_table_ = {
  // EEPROM
    {"Model_Number",         { 0,  2}},
    {"Firmware_Version",     { 6,  1}},
    {"ID",                   { 7,  1}},
    {"Baud_Rate",            { 8,  1}},
    {"Return_Delay_Time",    { 9,  1}},
    {"Drive_Mode",           { 10, 1}},
    {"Operating_Mode",       { 11, 1}},
    {"Homing_Offset",        { 20, 4}},
    {"Moving_Threshold",     { 24, 4}},
    {"Temperature_Limit",    { 31, 1}},
    {"Max_Voltage_Limit",    { 32, 2}},
    {"Min_Voltage_Limit",    { 34, 2}},
    {"Current_Limit",        { 38, 2}},
    {"Acceleration_Limit",   { 40, 4}},
    {"Velocity_Limit",       { 44, 4}},
    {"Max_Position_Limit",   { 48, 4}},
    {"Min_Position_Limit",   { 52, 4}},
    {"External_Port_Mode_1", { 56, 1}},
    {"External_Port_Mode_2", { 57, 1}},
    {"External_Port_Mode_3", { 58, 1}},
    {"External_Port_Mode_4", { 59, 1}},
    {"Shutdown",             { 63, 1}},
    {"Indirect_Address",     {168, 2}},
  // RAM
    {"Torque_Enable",         {512, 1}},
    {"LED_RED",               {513, 1}},
    {"LED_GREEN",             {514, 1}},
    {"LED_BLUE",              {515, 1}},
    {"Registered_Instruction",{517, 1}},
    {"Hardware_Error_Status", {518, 1}},
    {"Velocity_I_Gain",       {524, 2}},
    {"Velocity_P_Gain",       {526, 2}},
    {"Position_D_Gain",       {528, 2}},
    {"Position_P_Gain",       {532, 2}},
    {"Position_I_Gain",       {530, 2}},
    {"Bus_Watchdog",          {546, 1}},
    {"Goal_Position",         {564, 4}},
    {"Goal_Velocity",         {552, 4}},
    {"Goal_Current",          {604, 2}},
    {"Profile_Acceleration",  {556, 4}},
    {"Profile_Velocity",      {560, 4}},
    {"Moving",                {570, 1}},
    {"Present_Position",      {580, 4}},
    {"Present_Velocity",      {576, 4}},
    {"Present_Current",       {574, 2}},
    {"Present_Input_Voltage", {592, 2}},
    {"Present_Temperature",   {594, 1}},
    {"External_Port_Data_1",  {600, 2}},
    {"External_Port_Data_2",  {602, 2}},
    {"External_Port_Data_3",  {604, 2}},
    {"External_Port_Data_4",  {606, 2}},
    {"Indirect_Data",         {634, 1}},
};

int32_t get_data(const uint8_t* byte_array, uint16_t length);
void get_param(int32_t data, uint8_t *param);

// Class for controlling a group of dynamixels snake robot
class DynamixelController
{
 protected:
  // Dynamixel port handling
  dynamixel::PortHandler* port_handler_;
  dynamixel::PacketHandler* packet_handler_;
  dynamixel::GroupSyncRead* group_sync_read_;
  uint32_t timeout_ms_;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_update_;

  // Dynamixel data handling
  std::map<std::string, uint8_t> dynamixel_;  // Map from name to dynamixel ID
  uint8_t* id_param_;
  uint8_t* fbk_param_;
  uint32_t fbk_param_length_;
  ControlTable feedback_table_;  // Map from item name to indirect address

  /**
   * Get location in memory of parameters for a specific dynamixel
   * 
   * @param name Dynamixel name as defined in YAML file
   */
  uint8_t* get_fbk_params(const std::string dxl_name);

  /**
   * Read YAML files and set parameters on dynamixel
   * 
   * @param Path to dynamixel configuration 
   */
  bool get_dynamixel_info(const char* path);

  // State variables
  bool initialized_ = false;

 public:
  /**
   * Create a DynamixelController instance
   * 
   * @param port_name Dynamixel USB port (Linux /dev/ttyUSB[X] Windows COM[X])
   */
  explicit DynamixelController(const char* port_name);

  ~DynamixelController();

  /// Print an optionally formatted string to stderr
  template<typename ... Args>
  void print_error(const char* func_name, const char* msg, Args ... args);
  void print_error(const char* func_name, const char* msg);

  /**
   * Get the address in memory of a control table item
   * 
   * @param[in] item_name Name of parameter as defined in control table
   * @param[out] address Parameter address in memory
   * @param[out] length Parameter length
   * \return Value stored at address
   */
  int32_t get_item_address(std::string item_name, uint16_t* address,
                           uint16_t* length);

  /**
   * Initialize DynamixelController
   * 
   * @param config_path Path to dynamixel configuration ([PATH_TO_SRC]/config/dynamixels.yaml)
   * @param feedback_items Names of parameters to read during sync_read
   * \return True if initialized, false if not
   */
  bool initialize(const char* config_path,
                  std::vector<std::string> feedback_items,
                  uint32_t timeout = 100000);

  // Uninitialize DynamixelController
  bool uninitialize();

  /**
   * Read values from dynamixels and perform safety checks
   * 
   * @param timeout Dynamixel Bus Watchdog timeout in milliseconds
   * \return True if updated, false if not
   */
  bool update();

  /**
   * Write a register on a dynamixel
   * 
   * @param name Dynamixel name as defined in YAML file
   * @param item_name Name of parameter as defined in control table
   * @param data Value to write to register
   * \return True if register was written to, false if not
   */
  bool write_register(const std::string name, const std::string item_name,
                      const int32_t data);

  /**
   * Read a register on a dynamixel
   * 
   * @param[in] name Dynamixel name as defined in YAML file
   * @param[in] item_name Name of parameter as defined in control table
   * @param[out] data Value of register
   * \return True if data was read, false if not
   */
  bool read_register(const std::string name, const std::string item_name,
                     int32_t *data);

  // Read dynamixel feedback
  bool sync_read();

  /**
   * Write registers on multiple dynamixels simultaneously
   * 
   * @param[in] names Dynamixel names as defined in YAML file
   * @param[in] item_name Name of parameter as defined in control table
   * @param[out] data Values to write to register on each dynamixel
   * \return True if registers were written to, false if not
   */
  bool sync_write_register(const std::vector<std::string> names,
                           const std::string item_name,
                           std::vector<int32_t> data);

  /**
   * Read Get parameter value from last feedback received during update
   * 
   * @param name Dynamixel name as defined in YAML file
   * @param item_name Name of parameter as defined in control table
   * \return Feedback value address
   */
  int32_t get_fbk(const std::string dxl_name, const std::string item_name);

  /**
   * Reboot selected dynamixel
   * 
   * @param name Dynamixel name as defined in YAML file
   * \return True if reboot was successful, false if not
   */
  bool reboot(const std::string dxl_name);

  /**
   * Reset multi-turn revolution information of Dynamixel
   * 
   * @param name Dynamixel name as defined in YAML file
   * \return True if clear was successful, false if not
   */
  bool clear_multi_turn(const std::string dxl_name);

  /**
   * Reset every value of the dynamixel to their defaults, excluding baudrate and ID
   * 
   * @param name Dynamixel name as defined in YAML file
   * \return True if reset was successful, false if not
   */
  bool factory_reset(const std::string dxl_name);
};

#endif //DYNAMIXEL_CONTROLLER_H