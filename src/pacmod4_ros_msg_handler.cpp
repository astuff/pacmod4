// Copyright (c) 2022 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <pacmod4/pacmod4_ros_msg_handler.h>

#include <vector>
#include <string>
#include <memory>


namespace pacmod4
{

// LockedData
LockedData::LockedData(unsigned char data_length) :
  _data(),
  _data_mut()
{
  _data.assign(data_length, 0);
}

std::vector<unsigned char> LockedData::getData() const
{
  std::lock_guard<std::mutex> lck(_data_mut);
  return _data;
}

void LockedData::setData(std::vector<unsigned char> new_data)
{
  std::lock_guard<std::mutex> lck(_data_mut);
  _data = new_data;
}

// Pacmod4RosMsgHandler
Pacmod4RosMsgHandler::Pacmod4RosMsgHandler(uint32_t dbc_major_version)
{
  switch (dbc_major_version)
  {
    case (13):
    default:
      msg_api_ = std::make_unique<pacmod4_common::Dbc13Api>();
      break;
  }
  ROS_INFO("Initialized API for DBC version %d", msg_api_->GetDbcVersion());

  // Bool Reports
  parse_functions[HORN_RPT_CANID] =
  parse_functions[MARKER_LAMP_RPT_CANID] =
  parse_functions[SPRAYER_RPT_CANID] =
  parse_functions[HAZARD_LIGHTS_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSystemRptBool, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[HORN_RPT_CANID] =
  pub_functions[MARKER_LAMP_RPT_CANID] =
  pub_functions[SPRAYER_RPT_CANID] =
  pub_functions[HAZARD_LIGHTS_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SystemRptBool>, this, std::placeholders::_1, std::placeholders::_2);

  // Bool With Control Status Reports
  parse_functions[PARKING_BRAKE_RPT_CANID] =
  parse_functions[PARKING_BRAKE_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSystemRptBoolWithControlStatus, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[PARKING_BRAKE_RPT_CANID] =
  pub_functions[PARKING_BRAKE_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SystemRptBoolWithControlStatus>, this, std::placeholders::_1, std::placeholders::_2);

  // Int Reports
  parse_functions[CRUISE_CONTROL_BUTTONS_RPT_CANID] =
  parse_functions[ENGINE_RPT_CANID] =
  parse_functions[EXHAUST_BRAKE_RPT_CANID] =
  parse_functions[HEADLIGHT_RPT_CANID] =
  parse_functions[REAR_PASS_DOOR_RPT_CANID] =
  parse_functions[MEDIA_CONTROLS_RPT_CANID] =
  parse_functions[TURN_RPT_CANID] =
  parse_functions[TIPPER_BODY_RPT_00_CANID] =
  parse_functions[TIPPER_BODY_RPT_01_CANID] =
  parse_functions[TIPPER_BODY_RPT_02_CANID] =
  parse_functions[WIPER_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSystemRptInt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[CRUISE_CONTROL_BUTTONS_RPT_CANID] =
  pub_functions[ENGINE_RPT_CANID] =
  pub_functions[EXHAUST_BRAKE_RPT_CANID] =
  pub_functions[HEADLIGHT_RPT_CANID] =
  pub_functions[REAR_PASS_DOOR_RPT_CANID] =
  pub_functions[MEDIA_CONTROLS_RPT_CANID] =
  pub_functions[TURN_RPT_CANID] =
  pub_functions[TIPPER_BODY_RPT_00_CANID] =
  pub_functions[TIPPER_BODY_RPT_01_CANID] =
  pub_functions[TIPPER_BODY_RPT_02_CANID] =
  pub_functions[WIPER_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SystemRptInt>, this, std::placeholders::_1, std::placeholders::_2);

  // Int With Control Status Reports
  parse_functions[SHIFT_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSystemRptIntWithControlStatus, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[SHIFT_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SystemRptIntWithControlStatus>, this, std::placeholders::_1, std::placeholders::_2);

  // Float Reports
  parse_functions[ACCEL_RPT_CANID] =
  parse_functions[BRAKE_RPT_CANID] =
  parse_functions[BRAKE_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSystemRptFloat, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[STEERING_RPT_CANID] =
  parse_functions[STEERING_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSteeringRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ACCEL_RPT_CANID] =
  pub_functions[BRAKE_RPT_CANID] =
  pub_functions[BRAKE_RPT_2_CANID] =
  pub_functions[STEERING_RPT_CANID] =
  pub_functions[STEERING_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SystemRptFloat>, this, std::placeholders::_1, std::placeholders::_2);

  // Aux Reports
  parse_functions[ACCEL_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseAccelAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[BRAKE_AUX_RPT_CANID] =
  parse_functions[BRAKE_AUX_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseBrakeAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ENGINE_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseEngineAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ENGINE_AUX_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseEngineAuxRpt2, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ENGINE_BRAKE_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseEngineBrakeAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[HEADLIGHT_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseHeadlightAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[PARKING_BRAKE_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseParkingBrakeAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[SHIFT_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseShiftAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[STEERING_AUX_RPT_CANID] =
  parse_functions[STEERING_AUX_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSteeringAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[TURN_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseTurnAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[WIPER_AUX_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseWiperAuxRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ACCEL_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::AccelAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[BRAKE_AUX_RPT_CANID] =
  pub_functions[BRAKE_AUX_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::BrakeAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ENGINE_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::EngineAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ENGINE_AUX_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::EngineAuxRpt2>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ENGINE_BRAKE_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::EngineBrakeAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[HEADLIGHT_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::HeadlightAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[PARKING_BRAKE_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::ParkingBrakeAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[SHIFT_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::ShiftAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[STEERING_AUX_RPT_CANID] =
  pub_functions[STEERING_AUX_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SteeringAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[TURN_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::TurnAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[WIPER_AUX_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::WiperAuxRpt>, this, std::placeholders::_1, std::placeholders::_2);

  // Command Limit Reports
  parse_functions[ACCEL_CMD_LIMIT_RPT_CANID] =
  parse_functions[BRAKE_CMD_LIMIT_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseCmdLimitRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[STEERING_CMD_LIMIT_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSteeringCmdLimitRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[ACCEL_CMD_LIMIT_RPT_CANID] =
  pub_functions[BRAKE_CMD_LIMIT_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SystemCmdLimitRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[STEERING_CMD_LIMIT_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SteeringCmdLimitRpt>, this, std::placeholders::_1, std::placeholders::_2);

  // Component Reports
  parse_functions[COMPONENT_RPT_00_CANID] =
  parse_functions[COMPONENT_RPT_01_CANID] =
  parse_functions[COMPONENT_RPT_02_CANID] =
  parse_functions[COMPONENT_RPT_04_CANID] = std::bind(&pacmod4_common::DbcApi::ParseComponentRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[COMPONENT_RPT_00_CANID] =
  pub_functions[COMPONENT_RPT_01_CANID] =
  pub_functions[COMPONENT_RPT_02_CANID] =
  pub_functions[COMPONENT_RPT_04_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::ComponentRpt>, this, std::placeholders::_1, std::placeholders::_2);
  
  // Software Version Rpt Reports
  parse_functions[SOFTWARE_VERSION_RPT_00_CANID] =
  parse_functions[SOFTWARE_VERSION_RPT_01_CANID] =
  parse_functions[SOFTWARE_VERSION_RPT_02_CANID] =
  parse_functions[SOFTWARE_VERSION_RPT_04_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSoftwareVersionRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[SOFTWARE_VERSION_RPT_00_CANID] =
  pub_functions[SOFTWARE_VERSION_RPT_01_CANID] =
  pub_functions[SOFTWARE_VERSION_RPT_02_CANID] =
  pub_functions[SOFTWARE_VERSION_RPT_04_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SoftwareVersionRpt>, this, std::placeholders::_1, std::placeholders::_2);

  // Motor Reports
  parse_functions[BRAKE_MOTOR_RPT_1_CANID] =
  parse_functions[STEERING_MOTOR_RPT_1_CANID] = std::bind(&pacmod4_common::DbcApi::ParseMotorRpt1, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[BRAKE_MOTOR_RPT_2_CANID] =
  parse_functions[STEERING_MOTOR_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseMotorRpt1, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[BRAKE_MOTOR_RPT_3_CANID] =
  parse_functions[STEERING_MOTOR_RPT_3_CANID] = std::bind(&pacmod4_common::DbcApi::ParseMotorRpt3, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[BRAKE_MOTOR_RPT_1_CANID] =
  pub_functions[STEERING_MOTOR_RPT_1_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::MotorRpt1>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[BRAKE_MOTOR_RPT_2_CANID] =
  pub_functions[STEERING_MOTOR_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::MotorRpt2>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[BRAKE_MOTOR_RPT_3_CANID] =
  pub_functions[STEERING_MOTOR_RPT_3_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::MotorRpt3>, this, std::placeholders::_1, std::placeholders::_2);


  // Other Reports
  parse_functions[AIR_PRESSURE_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseAirPressureRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ANG_VEL_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseAngVelRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[AUTOMS_MAN_SWITCH_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseAutomsManSwitchRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[BATTERY_VOLTAGE_LEVEL_RPT_1_CANID] =
  parse_functions[BATTERY_VOLTAGE_LEVEL_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseBatteryVoltageLevelRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[CABIN_CLIMATE_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseCabinClimateRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[DOOR_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseDoorRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[DRIVE_TRAIN_FEATURE_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseDrivetrainFeatureRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ENGINE_BRAKE_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseEngineBrakeRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ESTOP_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseEStopRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[ENGINE_LOAD_FACTOR_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseEngineLoadFactorRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[GLOBAL_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseGlobalRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[GLOBAL_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseGlobalRpt2, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[INTERIOR_LIGHTS_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseInteriorLightsRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[NOTIFICATION_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseNotificationRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[OCCUPANCY_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseOccupancyRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[REAR_LIGHTS_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseRearLightsRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[REMOTE_STOP_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseRemoteStopRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[SAFETY_FUNC_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSafetyFuncRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[SAFETY_FUNC_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseSafetyFuncRpt2, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[WHEEL_SPEED_RPT_CANID] =
  parse_functions[WHEEL_SPEED_RPT_2_CANID] = std::bind(&pacmod4_common::DbcApi::ParseWheelSpeedRpt, std::ref(*msg_api_), std::placeholders::_1);
  parse_functions[VEHICLE_SPEED_RPT_CANID] = std::bind(&pacmod4_common::DbcApi::ParseVehicleSpeedRpt, std::ref(*msg_api_), std::placeholders::_1);
  pub_functions[AIR_PRESSURE_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::AirPressureRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ANG_VEL_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::AngVelRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[AUTOMS_MAN_SWITCH_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::AutomsManSwitchRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[BATTERY_VOLTAGE_LEVEL_RPT_1_CANID] =
  pub_functions[BATTERY_VOLTAGE_LEVEL_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::BatteryVoltageLevelRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[CABIN_CLIMATE_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::CabinClimateRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[DRIVE_TRAIN_FEATURE_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::DrivetrainFeatureRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[DOOR_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::DoorRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ENGINE_BRAKE_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::EngineBrakeRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ESTOP_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::EStopRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[ENGINE_LOAD_FACTOR_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::EngineLoadFactorRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[GLOBAL_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::GlobalRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[GLOBAL_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::GlobalRpt2>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[INTERIOR_LIGHTS_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::InteriorLightsRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[NOTIFICATION_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::NotificationRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[OCCUPANCY_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::OccupancyRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[REAR_LIGHTS_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::RearLightsRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[REMOTE_STOP_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::RemoteStopRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[SAFETY_FUNC_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SafetyFuncRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[SAFETY_FUNC_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::SafetyFuncRpt2>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[WHEEL_SPEED_RPT_CANID] =
  pub_functions[WHEEL_SPEED_RPT_2_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::WheelSpeedRpt>, this, std::placeholders::_1, std::placeholders::_2);
  pub_functions[VEHICLE_SPEED_RPT_CANID] = std::bind(&Pacmod4RosMsgHandler::ParseAndPublishType<pacmod4_msgs::VehicleSpeedRpt>, this, std::placeholders::_1, std::placeholders::_2);
}

void Pacmod4RosMsgHandler::ParseAndPublish(const can_msgs::Frame& can_msg, const ros::Publisher& pub)
{
  if (pub_functions.count(can_msg.id))
  {
    pub_functions[can_msg.id](can_msg, pub);
  }
}

}  // namespace pacmod4

