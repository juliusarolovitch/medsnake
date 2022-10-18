#include <gtest/gtest.h>
#include <dynamixel_sdk.h>  // Uses Dynamixel SDK library
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "dynamixel_controller.h"

// Basic assertions.
TEST(BasicTests, get_param)
{
  // Expect two strings not to be equal.
  // EXPECT_STRNE("hello", "world");
  // Expect equality.
  uint8_t param[4];
  get_param(-303454, param);
  EXPECT_EQ(param[0], 0xA2);
  EXPECT_EQ(param[1], 0x5E);
  EXPECT_EQ(param[2], 0xFB);
  EXPECT_EQ(param[3], 0xFF);
}

TEST(BasicTests, get_data)
{
  // Expect two strings not to be equal.
  // EXPECT_STRNE("hello", "world");
  // Expect equality.
  uint8_t param[4] = {0xA2, 0x5E, 0xFB, 0xFF};
  EXPECT_EQ(get_data(param, 4), -303454);
}



class FakePacketHandler : public dynamixel::PacketHandler {
 private:
  static FakePacketHandler *unique_instance_;
 public:
  uint8_t* last_packet_;
  uint32_t last_packet_length_;

  static FakePacketHandler *getPacketHandler(float protocol_version = 2.0) {return unique_instance_;}

  int writeTxRx(dynamixel::PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error)
  {
    if(last_packet_) delete last_packet_;
    last_packet_ = new uint8_t[length];
    for (uint16_t s = 0; s < length; s++)
      last_packet_[s] = data[s];
    return COMM_SUCCESS;
  };
};
class FakePortHandler : public dynamixel::PacketHandler {
 private:
  static FakePortHandler *unique_instance_;
 public:
  static FakePortHandler *getPortHandler(const char *port_name) {return unique_instance_;}
  bool    openPort() { return true; };
};

// Dynamixel controller with fake serial port
class FakeDynamixelController : public DynamixelController
{
protected:
  FakePacketHandler* packet_handler_;
  FakePortHandler* port_handler_;
  
public:
  FakeDynamixelController() : DynamixelController("USB0")
  {
    timeout_ms_ = 200;

    packet_handler_= FakePacketHandler::getPacketHandler(2.0);
    port_handler_= FakePortHandler::getPortHandler("USB0");
  }
};

// The fixture for testing class DynamixelController.
class DynamixelControllerTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  DynamixelControllerTest() {
     // You can do set-up work for each test here.
  }

  ~DynamixelControllerTest() override {
     // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
    YAML::Emitter emitter;
    emitter << "inner_snake_rail:\n";
    emitter << "  ID: 0\n";
    emitter << "  Return_Delay_Time: 0\n";
    emitter << "  Operating_Mode: 1\n";
    emitter << "  Profile_Acceleration: 0\n";
    emitter << "  Profile_Velocity: 0\n";
    emitter << "  Current_Limit: 350\n";
    emitter << "  Drive_Mode: 0;\n";
    std::ofstream fout("test.yaml");
    fout << emitter.c_str();
  }

  void TearDown() override {
     // Code here will be called immediately after each test (right
     // before the destructor).
  }

  // Class members declared here can be used by all tests in the test suite
  // for Foo.
};

// // Tests that the Foo::Bar() method does Abc.
// TEST_F(DynamixelControllerTest, MethodBarDoesAbc) {
//   const std::string input_filepath = "this/package/testdata/myinputfile.dat";
//   const std::string output_filepath = "this/package/testdata/myoutputfile.dat";
//   DynamixelController f;
//   EXPECT_EQ(f.Bar(input_filepath, output_filepath), 0);
// }