#include <iostream>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include "dynamixel_controller.h"

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define BAUD_RATE 115200

// Start and end of ctrl_table_ sections
#define EEPROM_START 0
#define EEPROM_END 63

#define FBK_START 512
#define FBK_END 602
#define FBK_LEN (FBK_END - FBK_START)

#define CMD_START 548
#define CMD_END 568
#define CMD_LEN (CMD_END - CMD_START)

#define TXPACKET_MAX_LEN    (1*1024)
#define RXPACKET_MAX_LEN    (1*1024)

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

///////////////// Protocol 2.0 Error bit /////////////////
#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION      2       // Instruction error
#define ERRNUM_CRC              3       // CRC check error
#define ERRNUM_DATA_RANGE       4       // Data range error
#define ERRNUM_DATA_LENGTH      5       // Data length error
#define ERRNUM_DATA_LIMIT       6       // Data limit error
#define ERRNUM_ACCESS           7       // Access error

#define ERRBIT_ALERT            128     //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

#define INST_FAST_SYNC_READ     138     // 0x8A
#define INST_SYNC_READ          130     // 0x82

// Format a string
template<typename ... Args>
std::string strfmt(const std::string& format, Args ... args )
{
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if ( size_s <= 0 ) { throw std::runtime_error( "[strfmt] Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

// Send a sync read command to dynamixels
int fastSyncReadTx(dynamixel::PortHandler *port, dynamixel::PacketHandler *ph,
                   uint16_t start_address, uint16_t data_length, uint8_t *param,
                   uint16_t param_length)
{

  int result                  = COMM_TX_FAIL;

  uint8_t *txpacket           = (uint8_t *)malloc(param_length + 14 + (param_length / 3));
  // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

  if (txpacket == NULL)
    return result;

  txpacket[PKT_ID]            = BROADCAST_ID;
  txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  // TODO: Switch to Fast_Sync_Read
  txpacket[PKT_INSTRUCTION]   = INST_SYNC_READ;
  txpacket[PKT_PARAMETER0 + 0]  = DXL_LOBYTE(start_address);
  txpacket[PKT_PARAMETER0 + 1]  = DXL_HIBYTE(start_address);
  txpacket[PKT_PARAMETER0 + 2]  = DXL_LOBYTE(data_length);
  txpacket[PKT_PARAMETER0 + 3]  = DXL_HIBYTE(data_length);

  for (uint16_t s = 0; s < param_length; s++)
    txpacket[PKT_PARAMETER0 + 4 + s] = param[s];

  result = ph->txPacket(port, txpacket);
  if (result == COMM_SUCCESS)
    port->setPacketTimeout((uint16_t)((11 + data_length) * param_length));

  free(txpacket);
  return result;
}

// Turn a byte array into an integer
int32_t get_data(const uint8_t* byte_array, uint16_t length)
{
  switch (length)
  {
  case 1:
    return byte_array[0];

  case 2:
    return DXL_MAKEWORD(byte_array[0], byte_array[1]);

  case 4:
    return DXL_MAKEDWORD(DXL_MAKEWORD(byte_array[0], byte_array[1]),
                         DXL_MAKEWORD(byte_array[2], byte_array[3]));

  default:
    return 0;
  }
}

// Turn an integer into a byte array
void get_param(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

// Print out a byte array
void print_param(const uint8_t* byte_array, uint16_t length)
{
  for (uint16_t s = 0; s < length; s++)
  {
    printf("0x%.2X|", (uint8_t) * (byte_array + s));
  } printf("\n");
}

// Get address of register in Dynamixel memory
int32_t DynamixelController::get_item_address(std::string item_name,
                                              uint16_t* address,
                                              uint16_t* length)
{
  try {
    uint16_t offset = 0;
    if (item_name.find("Indirect_Address") == 0 &&
        item_name.length() > strlen("Indirect_Address"))
    {
      if (address)
      {
        offset = std::stoi(item_name.substr(strlen("Indirect_Address"))) - 1;
        *address = ctrl_table_.at("Indirect_Address").address + offset * 2;
      }
      if (length) *length = 2;
    } else if (item_name.find("Indirect_Data") == 0 &&
               item_name.length() > strlen("Indirect_Data")) {
      if (address)
      {
        offset = std::stoi(item_name.substr(strlen("Indirect_Data"))) - 1;
        *address = ctrl_table_.at("Indirect_Data").address + offset;
      }
      if (length) *length = 0;  // We can't know the length of indirect data
    } else {
      if (address) *address = ctrl_table_.at(item_name).address;
      if (length) *length = ctrl_table_.at(item_name).data_length;
    }
    return true;
  } catch (const std::out_of_range&) {
    print_error(__func__, "item %s does not exist in ctrl_table_\n",
                item_name.c_str());
    return false;
  }
}

DynamixelController::DynamixelController(const char* port_name)
  : last_update_()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  std::cout << port_name << std::endl;
  port_handler_ = dynamixel::PortHandler::getPortHandler(port_name);
  port_handler_->setBaudRate(BAUD_RATE);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

DynamixelController::~DynamixelController()
{
  uninitialize();
}

// Print an error
template<typename ... Args>
void DynamixelController::print_error(const char* func_name, const char* msg,
                                      Args ... args)
{
  print_error(func_name, strfmt(msg, args...).c_str());
}
void DynamixelController::print_error(const char* func_name, const char* msg)
{
  fprintf(stderr, "[Dynamixel::%s] %s\n", func_name, msg);
}

bool DynamixelController::write_register(const std::string dynamixel_name,
                                         const std::string item_name,
                                         int32_t data)
{
  uint16_t address, length;
  get_item_address(item_name, &address, &length);
  uint8_t id = 0;
  try {
    id = dynamixel_.at(dynamixel_name);
  } catch (const std::out_of_range&)
  {
    print_error(__func__, "Dynamixel %s does not exist\n", dynamixel_name.c_str());
    return false;
  }

  uint8_t parameter[4] = {0, 0, 0, 0};
  uint8_t error = 0;
  get_param(data, parameter);

  int result = packet_handler_->writeTxRx(port_handler_, id, address, length,
                                          parameter, &error);

  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "%s PacketHandler comm Error %s\n", packet_handler_->getTxRxResult(result));
    return false;
  }
  else if (result != 0)
  {
    print_error(__func__, "%s PacketHandler hardware Error %s\n", packet_handler_->getRxPacketError(error));
    return false;
  }
  return true;
}

bool DynamixelController::read_register(const std::string dynamixel_name,
                                        const std::string item_name,
                                        int32_t *data)
{
  uint16_t address, length;
  get_item_address(item_name, &address, &length);

  uint8_t parameter[4] = {0, 0, 0, 0};
  uint8_t error = 0;
  uint8_t id = 0;
  try {
    id = dynamixel_.at(dynamixel_name);
  } catch (const std::out_of_range&)
  {
    print_error(__func__, "Dynamixel %s does not exist\n", dynamixel_name.c_str());
    return false;
  }

  int result = packet_handler_->readTxRx(port_handler_, id, address, length, parameter, &error);

  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "PacketHandler comm Error %s\n", packet_handler_->getTxRxResult(result));
    return false;
  } else if (result != 0)
  {
    print_error(__func__, "PacketHandler hardware Error %s\n", packet_handler_->getRxPacketError(error));
    return false;
  }
  *data = (int32_t)get_data(parameter, length);
  return true;
}

bool DynamixelController::get_dynamixel_info(const char* yaml_file)
{
  YAML::Node config;
  config = YAML::LoadFile(yaml_file);

  // if (config == NULL)
  if (config.IsNull())
    return false;

  for (auto it_file = config.begin(); it_file != config.end(); ++it_file)
  {
    // name of the motor defined in dynamixels.yaml, i.e. inner_snake_rail
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    // configuration of each motor
    YAML::Node item = config[name];
    for (auto it_item = item.begin(); it_item != item.end(); ++it_item)
    {
      // each config name of the motor:
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
      {
        dynamixel_[name] = value;
        uint16_t model_number = 0;
        uint8_t dxl_error;
        int result = packet_handler_->ping(port_handler_, dynamixel_[name],
                                           &model_number, &dxl_error);
        if (result != COMM_SUCCESS)
        {
          print_error(__func__, "PacketHandler comm Error %s\n", packet_handler_->getTxRxResult(result));
          fprintf(stderr, "                                          could not ping %s [ID: %d]\n", name.c_str(), int(dynamixel_[name]));
          return false;
        }
        if (!write_register(name, "Torque_Enable", false))
        {
          return false;
        }
      } else if (item_name != "Baud_Rates")
      {
        if (!write_register(name, item_name, value))
        {
          print_error(__func__, "Failed to write [%s] to %s [ID: %d]\n",
                      item_name.c_str(), name.c_str(), int(dynamixel_[name]));
          return false;
        }
      }
    }
  }

  return true;
}

bool DynamixelController::uninitialize()
{
  fbk_param_length_ = 0;
  if (initialized_)
  {
    // Shut down dynamixels
    for ( const auto &dxl : dynamixel_ )
    {
      write_register(dxl.first, "Torque_Enable", false);
    }
    delete(fbk_param_);
    delete(id_param_);
    delete(group_sync_read_);
  }
  // Close port
  port_handler_->closePort();
  dynamixel_.clear();
  initialized_ = false;
  return true;
}

bool DynamixelController::initialize(const char* config_path,
                                     std::vector<std::string> feedback_items,
                                     uint32_t timeout)
{
  timeout_ms_ = timeout;
  uninitialize();
  if (!port_handler_->openPort())
  {
    print_error(__func__, "Failed to open port %s\n", port_handler_->getPortName());
    return false;
  }

  if (!get_dynamixel_info(config_path))
  {
    print_error(__func__, "Failed to get dynamixel info from %s\n", config_path);
    return false;
  }

  // Create an array of indirect addresses for feedback
  bool success = true;
  uint16_t address, length, indirect_address;
  dynamixel::GroupBulkWrite bulk_write(port_handler_, packet_handler_);
  uint8_t addresses[feedback_items.size() * (4 * 2)]; // Max size is (max_param_len * address_len) for each address
  uint32_t packet_length = 0;

  for (int i = 0; success && i < feedback_items.size(); i++)
  {
    get_item_address(feedback_items[i], &address, &length);
    get_item_address(strfmt("Indirect_Data%d", packet_length + 1),
                     &indirect_address, nullptr);
    feedback_table_[feedback_items[i]] = {indirect_address, length};
    for (int j = 0; success && j < length; j++)
    {
      uint16_t start = (packet_length) * 2;
      uint8_t param[4];
      get_param(address + j, param);
      std::copy(param, param + 2, addresses + start);
      packet_length++;
    }
  }
  fbk_param_length_ = packet_length;

  // Send array of indirect addresses
  get_item_address("Indirect_Address", &indirect_address, nullptr);
  for ( const auto &dxl : dynamixel_ )
  {
    if (!write_register(dxl.first, "Torque_Enable", false))
    {
      return false;
    }
    bulk_write.addParam(dxl.second, indirect_address,
                        fbk_param_length_ * 2, addresses);
  }
  int result = bulk_write.txPacket();

  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "PacketHandler comm Error %s\n",
                packet_handler_->getTxRxResult(result));
    return false;
  }

  // Set up indirect addresses
  for ( const auto &dxl : dynamixel_ )
  {
    if (!write_register(dxl.first, "Torque_Enable", true))
    {
      print_error(__func__, dxl.first.c_str());
      // return false;
    }
    if (!write_register(dxl.first, "Bus_Watchdog", 0))
    {
      // return false;
    }
  }

  // Set up parameters for feedback
  fbk_param_ = new uint8_t[dynamixel_.size() * (fbk_param_length_ + 2)];  //ERR(1) + ID(1) + DATA(data_length)
  id_param_ = new uint8_t[dynamixel_.size()];
  int idx = 0;
  for ( const auto &dxl : dynamixel_ )
  {
    *(get_fbk_params(dxl.first) - 2) = COMM_RX_WAITING;
    *(get_fbk_params(dxl.first) - 1) = dxl.second;
    id_param_[idx] = dxl.second;
    idx++;
  }

  // Initialize GroupSyncRead instance
  get_item_address("Indirect_Data", &address, nullptr);
  group_sync_read_ = new dynamixel::GroupSyncRead(port_handler_,
      packet_handler_,
      address, fbk_param_length_);

  initialized_ = true;
  return true;
}

uint8_t* DynamixelController::get_fbk_params(const std::string dxl_name)
{
  if (!fbk_param_)
  {
    throw std::out_of_range("[DynamixelController::get_fbk_params] get_fbk_params called before initialized");
  }
  uint16_t idx = distance(dynamixel_.begin(), dynamixel_.find(dxl_name));
  if (idx == dynamixel_.size())
  {
    throw std::out_of_range("[DynamixelController::get_fbk_params] " + dxl_name + " not found");
  }
  idx = idx * (fbk_param_length_ + 2) + 2;
  return fbk_param_ + idx;
}


int32_t DynamixelController::get_fbk(const std::string dxl_name,
                                     const std::string item_name)
{
  Register fbk;
  try {
    fbk = feedback_table_.at(item_name);
  } catch (const std::out_of_range&)
  {
    throw std::out_of_range(strfmt("[DynamixelController::get_fbk] %s not in feedback_table_\n",
                                   item_name.c_str()));
    return false;
  }
  uint16_t address;
  get_item_address("Indirect_Data", &address, nullptr);
  address = fbk.address - address;
  return get_data(&get_fbk_params(dxl_name)[address], fbk.data_length);
}

bool DynamixelController::sync_read()
{
  int result;
  uint16_t address;
  get_item_address("Indirect_Data", &address, nullptr);
  result = fastSyncReadTx(port_handler_, packet_handler_, address,
                          fbk_param_length_, id_param_,
                          (uint16_t)dynamixel_.size() * 1);

  int idx = 0;
  uint8_t error = 0;

  for ( const auto &dxl : dynamixel_ )
  {
    if (result != COMM_SUCCESS)
      break;
    uint8_t id = dxl.second;
    uint8_t* data = get_fbk_params(dxl.first);
    result = packet_handler_->readRx(port_handler_, id, fbk_param_length_,
                                     data, &error);
    *(data - 2) = result; // Set error in feedback
    idx++;
  }

  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "PacketHandler comm Error %s\n",
                packet_handler_->getTxRxResult(result));
    return false;
  } else if (result != 0) {
    print_error(__func__, "PacketHandler hardware Error %s\n",
                packet_handler_->getRxPacketError(error));
    return false;
  }

  return true;
}

bool DynamixelController::update()
{
  if (!initialized_)
  {
    return false;
  }

  sync_read();

  last_update_ = std::chrono::high_resolution_clock::now();
  // print_param(fbk_param_, (fbk_param_length_ + 2) * dynamixel_.size());

  for (const auto &dxl : dynamixel_)
  {
    int32_t watchdog = get_fbk(dxl.first, "Bus_Watchdog");
    if (watchdog == 0)
    {
      write_register(dxl.first, "Bus_Watchdog", timeout_ms_ / 20);
    } else if (watchdog == 0xFF) {
      throw (std::runtime_error("[DynamixelController::update] Bus watchdog timeout."));
    }
  }

  return true;
}

bool DynamixelController::sync_write_register(const std::vector<std::string> names,
    const std::string item_name,
    std::vector<int32_t> data)
{
  uint16_t address, length;
  get_item_address(item_name, &address, &length);
  dynamixel::GroupSyncWrite group_sync_write(port_handler_, packet_handler_,
      address, length);
  uint8_t param[4];
  for (int i = 0; i < names.size(); i++)
  {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                   (std::chrono::high_resolution_clock::now() - last_update_);
    if (duration.count() > timeout_ms_)
    {
      print_error(__func__, "Dynamixel timeout (Did you call update?)");
      return false;
    }
    get_param(data[i], param);
    group_sync_write.addParam(dynamixel_[names[i]], param);
  }
  int result = group_sync_write.txPacket();

  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "PacketHandler comm Error %s\n",
                packet_handler_->getTxRxResult(result));
    return false;
  }
  return true;
}

bool DynamixelController::reboot(const std::string dxl_name)
{
  uint8_t error;
  int result;
  result = packet_handler_->reboot(port_handler_, dynamixel_[dxl_name], &error);
  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "PacketHandler comm Error %s\n",
                packet_handler_->getTxRxResult(result));
    return false;
  } else if (result != 0) {
    print_error(__func__, "PacketHandler hardware Error %s\n",
                packet_handler_->getRxPacketError(error));
    return false;
  }
  initialized_ = false;
  return true;
}

bool DynamixelController::clear_multi_turn(const std::string dxl_name)
{
  uint8_t error;
  int result;
  result = packet_handler_->clearMultiTurn(port_handler_, dynamixel_[dxl_name],
           &error);
  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "PacketHandler comm Error %s\n",
                packet_handler_->getTxRxResult(result));
    return false;
  } else if (result != 0) {
    print_error(__func__, "PacketHandler hardware Error %s\n",
                packet_handler_->getRxPacketError(error));
    return false;
  }
  return true;
}

bool DynamixelController::factory_reset(const std::string dxl_name)
{
  uint8_t error;
  int result;
  result = packet_handler_->factoryReset(port_handler_, dynamixel_[dxl_name],
                                         0x02, &error);
  if (result != COMM_SUCCESS)
  {
    print_error(__func__, "PacketHandler comm Error %s\n",
                packet_handler_->getTxRxResult(result));
    return false;
  } else if (result != 0) {
    print_error(__func__, "PacketHandler hardware Error %s\n",
                packet_handler_->getRxPacketError(error));
    return false;
  }
  initialized_ = false;
  return true;
}