// Copyright (c) 2021 AutonomouStuff, LLC
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

#include "pacmod4/pacmod4_nodelet.h"

#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace pacmod4
{

void Pacmod4Nl::onInit()
{
  nh_ = getMTNodeHandle();
  pnh_ = getMTPrivateNodeHandle();
  loadParams();

  handler = std::make_unique<Pacmod4RosMsgHandler>(dbc_major_version_);

  // Publishers that should be needed with any platform
  can_rx_pub = nh_.advertise<can_msgs::Frame>("can_rx", 20);
  enabled_pub = nh_.advertise<std_msgs::Bool>("enabled", 20, true);
  all_system_statuses_pub = nh_.advertise<pacmod4_msgs::AllSystemStatuses>("all_system_statuses", 20);

  pub_tx_list.emplace(ACCEL_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::AccelAuxRpt>("accel_aux_rpt", 20));
  pub_tx_list.emplace(ACCEL_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptFloat>("accel_rpt", 20));
  pub_tx_list.emplace(BRAKE_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::BrakeAuxRpt>("brake_aux_rpt", 20));
  pub_tx_list.emplace(BRAKE_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptFloat>("brake_rpt", 20));
  pub_tx_list.emplace(SHIFT_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::ShiftAuxRpt>("shift_aux_rpt", 20));
  pub_tx_list.emplace(SHIFT_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptIntWithControlStatus>("shift_rpt", 20));
  pub_tx_list.emplace(STEERING_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::SteeringAuxRpt>("steering_aux_rpt", 20));
  pub_tx_list.emplace(STEERING_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptFloat>("steering_rpt", 20));
  pub_tx_list.emplace(TURN_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::TurnAuxRpt>("turn_aux_rpt", 20));
  pub_tx_list.emplace(TURN_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("turn_rpt", 20));
  pub_tx_list.emplace(VEHICLE_SPEED_RPT_CANID, nh_.advertise<pacmod4_msgs::VehicleSpeedRpt>("vehicle_speed_rpt", 20));

  // Subscribers that should be needed with any platform
  can_tx_sub = nh_.subscribe("can_tx", 20, &Pacmod4Nl::can_read, this);

  accel_cmd_sub = nh_.subscribe("accel_cmd", 20, &Pacmod4Nl::callback_accel_cmd_sub, this);
  brake_cmd_sub = nh_.subscribe("brake_cmd", 20, &Pacmod4Nl::callback_brake_cmd_sub, this);
  shift_cmd_sub = nh_.subscribe("shift_cmd", 20, &Pacmod4Nl::callback_shift_set_cmd, this);
  steer_cmd_sub = nh_.subscribe("steering_cmd", 20, &Pacmod4Nl::callback_steer_cmd_sub, this);
  turn_cmd_sub = nh_.subscribe("turn_cmd", 20, &Pacmod4Nl::callback_turn_signal_set_cmd, this);

  rx_list.emplace(
    ACCEL_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    BRAKE_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    SHIFT_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    STEERING_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));
  rx_list.emplace(
    TURN_CMD_CANID,
    std::shared_ptr<LockedData>(new LockedData()));

  auto accel_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdFloat>();
  auto brake_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdFloat>();
  auto steer_cmd_msg = std::make_shared<pacmod4_msgs::SteeringCmd>();
  auto shift_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
  auto turn_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
  // Initialize Turn Signal command with non-0 value
  turn_cmd_msg->command = pacmod4_msgs::SystemCmdInt::TURN_NONE;
  
  init_rx_msg(ACCEL_CMD_CANID, accel_cmd_msg);
  init_rx_msg(BRAKE_CMD_CANID, brake_cmd_msg);
  init_rx_msg(STEERING_CMD_CANID, steer_cmd_msg);
  init_rx_msg(SHIFT_CMD_CANID, shift_cmd_msg);
  init_rx_msg(TURN_CMD_CANID, turn_cmd_msg);

  // Set initial state
  set_enable(false);

  // Start timers for repeating tasks
  can_send_timer_ = nh_.createTimer(ros::Duration(1/PACMOD_UPDATE_FREQ), &Pacmod4Nl::can_write, this);
  status_update_timer_ = nh_.createTimer(ros::Duration(1/PACMOD_UPDATE_FREQ), &Pacmod4Nl::SystemStatusUpdate, this);
}

void Pacmod4Nl::loadParams()
{
  // Get and validate parameters
  pnh_.param<int>("dbc_major_version", dbc_major_version_, 3);
  if (dbc_major_version_ != 13)
  {
    NODELET_ERROR("This driver currently only supports PACMod DBC version 13");
    ros::shutdown();
  }
}

// generic apis

void Pacmod4Nl::initializeAngVelRptApi()
{
  pub_tx_list.emplace(ANG_VEL_RPT_CANID, nh_.advertise<pacmod4_msgs::AngVelRpt>("ang_vel_rpt", 20));
  NODELET_INFO("Initialized Ang Vel Rpt API");
}

void Pacmod4Nl::initializeAutomsManSwitchRptApi()
{
  pub_tx_list.emplace(AUTOMS_MAN_SWITCH_RPT_CANID, nh_.advertise<pacmod4_msgs::AutomsManSwitchRpt>("automs_man_switch_rpt", 20));
  NODELET_INFO("Initialized Automs Man Switch Rpt API");
}

void Pacmod4Nl::initializeBatteryVoltageLevelRptApi(uint32_t can_id)
{
  switch(can_id)
  {
    case BATTERY_VOLTAGE_LEVEL_RPT_1_CANID:
    {
      pub_tx_list.emplace(BATTERY_VOLTAGE_LEVEL_RPT_1_CANID,
        nh_.advertise<pacmod4_msgs::BatteryVoltageLevelRpt>("battery_voltage_level_rpt_1", 20));
      NODELET_INFO("Initialized Battery Voltage Level Rpt 1 API");
    }
    case BATTERY_VOLTAGE_LEVEL_RPT_2_CANID:
    {
      pub_tx_list.emplace(BATTERY_VOLTAGE_LEVEL_RPT_2_CANID,
        nh_.advertise<pacmod4_msgs::BatteryVoltageLevelRpt>("battery_voltage_level_rpt_2", 20));
      NODELET_INFO("Initialized Battery Voltage Level Rpt 2 API");
    }
  }
}

void Pacmod4Nl::initializeBrakeDecelApi()
{
  pub_tx_list.emplace(BRAKE_DECEL_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptFloat>("brake_decel_rpt", 20));
  pub_tx_list.emplace(BRAKE_DECEL_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::BrakeDecelAuxRpt>("brake_decel_aux_rpt", 20));
  brake_decel_set_cmd = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "brake_decel_cmd", 20, &Pacmod4Nl::callback_brake_decel_set_cmd, this));
  auto brake_decel_cmd_msg = std::make_shared<pacmod4_msgs::BrakeDecelCmd>();
  init_rx_msg(BRAKE_DECEL_CMD_CANID, brake_decel_cmd_msg);
  NODELET_INFO("Initialized Brake Decel Api");
}

void Pacmod4Nl::initializeBrakeRpt2Api()
{
  pub_tx_list.emplace(BRAKE_RPT_2_CANID, nh_.advertise<pacmod4_msgs::SystemRptFloat>("brake_rpt_2", 20));
  pub_tx_list.emplace(BRAKE_AUX_RPT_2_CANID, nh_.advertise<pacmod4_msgs::BrakeAuxRpt>("brake_aux_rpt_2", 20));
  NODELET_INFO("Initialized Brake Rpt 2 API");
}

void Pacmod4Nl::initializeCabinClimateRpt()
{
  pub_tx_list.emplace(CABIN_CLIMATE_RPT_CANID, nh_.advertise<pacmod4_msgs::CabinClimateRpt>("cabin_climate_rpt", 20));
  cabin_climate_set_cmd = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "cabin_climate_set_cmd", 20, &Pacmod4Nl::callback_cabin_climate_set_cmd, this));
  auto cabin_climate_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
  init_rx_msg(CABIN_CLIMATE_CMD_CANID, cabin_climate_cmd_msg);

  NODELET_INFO("Initialized Cabin Climate Rpt API");
}

void Pacmod4Nl::initializeCmdLimitRpt(uint32_t can_id)
{
  switch (can_id)
  {
  case ACCEL_CMD_LIMIT_RPT_CANID:
    pub_tx_list.emplace(ACCEL_CMD_LIMIT_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemCmdLimitRpt>("accel_cmd_limit_rpt", 20));
    NODELET_INFO("Initialized Accel Cmd Limit Rpt API");
    break;
  case BRAKE_CMD_LIMIT_RPT_CANID:
    pub_tx_list.emplace(BRAKE_CMD_LIMIT_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemCmdLimitRpt>("brake_cmd_limit_rpt", 20));
    NODELET_INFO("Initialized Brake Cmd Limit Rpt API");
    break;
  case STEERING_CMD_LIMIT_RPT_CANID:
    pub_tx_list.emplace(STEERING_CMD_LIMIT_RPT_CANID, nh_.advertise<pacmod4_msgs::SteeringCmdLimitRpt>("steering_cmd_limit_rpt", 20));
    NODELET_INFO("Initialized Steering Cmd Limit Rpt API");
    break;
  }
}

void Pacmod4Nl::initializeComponentRptApi(uint32_t can_id)
{
  switch(can_id)
  {
    case COMPONENT_RPT_00_CANID:
    {
      pub_tx_list.emplace(COMPONENT_RPT_00_CANID, nh_.advertise<pacmod4_msgs::ComponentRpt>("component_rpt_00", 20));
      NODELET_INFO("Initialized Component Rpt 00 API");
      break;
    }
    case COMPONENT_RPT_01_CANID:
    {
      pub_tx_list.emplace(COMPONENT_RPT_01_CANID, nh_.advertise<pacmod4_msgs::ComponentRpt>("component_rpt_01", 20));
      NODELET_INFO("Initialized Component Rpt 01 API");
      break;
    }
    case COMPONENT_RPT_02_CANID:
    {
      pub_tx_list.emplace(COMPONENT_RPT_02_CANID, nh_.advertise<pacmod4_msgs::ComponentRpt>("component_rpt_02", 20));
      NODELET_INFO("Initialized Component Rpt 02 API");
      break;
    }
    case COMPONENT_RPT_03_CANID:
    {
      pub_tx_list.emplace(COMPONENT_RPT_03_CANID, nh_.advertise<pacmod4_msgs::ComponentRpt>("component_rpt_03", 20));
      NODELET_INFO("Initialized Component Rpt 03 API");
      break;
    }
    case COMPONENT_RPT_04_CANID:
    {
      pub_tx_list.emplace(COMPONENT_RPT_04_CANID, nh_.advertise<pacmod4_msgs::ComponentRpt>("component_rpt_04", 20));
      NODELET_INFO("Initialized Component Rpt 04 API");
      break;
    }
    case COMPONENT_RPT_05_CANID:
    {
      pub_tx_list.emplace(COMPONENT_RPT_05_CANID, nh_.advertise<pacmod4_msgs::ComponentRpt>("component_rpt_05", 20));
      NODELET_INFO("Initialized Component Rpt 05 API");
      break;
    }
  }
}

void Pacmod4Nl::initializeDoorRptApi()
{
  pub_tx_list.emplace(DOOR_RPT_CANID, nh_.advertise<pacmod4_msgs::DoorRpt>("door_rpt", 20));
  NODELET_INFO("Initialized DoorRpt API");
}

void Pacmod4Nl::initializeEStopRptApi()
{
  pub_tx_list.emplace(ESTOP_RPT_CANID, nh_.advertise<pacmod4_msgs::EStopRpt>("estop_rpt", 20));
  NODELET_INFO("Initialized EStopRpt API");
}

void Pacmod4Nl::initializeEngineBrakeApi()
{
  pub_tx_list.emplace(ENGINE_BRAKE_RPT_CANID, nh_.advertise<pacmod4_msgs::EngineBrakeRpt>("engine_brake_rpt", 20));
  pub_tx_list.emplace(ENGINE_BRAKE_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::EngineBrakeAuxRpt>("engine_brake_aux_rpt", 20));
  engine_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "engine_brake_cmd", 20, &Pacmod4Nl::callback_engine_brake_set_cmd, this));
  auto engine_brake_cmd_msg = std::make_shared<pacmod4_msgs::EngineBrakeCmd>();
  init_rx_msg(ENGINE_CMD_CANID, engine_brake_cmd_msg);
  NODELET_INFO("Initialized Engine Brake API");
}

void Pacmod4Nl::initializeEngineRptApi()
{
  pub_tx_list.emplace(ENGINE_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("enigne_rpt", 20));
  NODELET_INFO("Initialized Engine Rpt API");
}

void Pacmod4Nl::initializeGlobalRptApi(uint32_t can_id)
{
  switch(can_id)
  {
    case GLOBAL_RPT_CANID:
    {
      pub_tx_list.emplace(GLOBAL_RPT_CANID, nh_.advertise<pacmod4_msgs::GlobalRpt>("global_rpt", 20));
      NODELET_INFO("Initialized Global Rpt API");
      break;
    }
    case GLOBAL_RPT_2_CANID:
    {
      pub_tx_list.emplace(GLOBAL_RPT_2_CANID, nh_.advertise<pacmod4_msgs::GlobalRpt2>("global_rpt_2", 20));
      NODELET_INFO("Initialized Global Rpt 2 API");
      break;
    }
  }
}

void Pacmod4Nl::initializeHazardLightApi()
{
  pub_tx_list.emplace(HAZARD_LIGHTS_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptBool>("hazard_lights_rpt", 20));

  hazard_lights_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("hazard_lights_cmd", 20, &Pacmod4Nl::callback_hazard_lights_set_cmd, this));
  auto hazard_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdBool>();
  init_rx_msg(HAZARD_LIGHTS_CMD_CANID, hazard_cmd_msg);
  NODELET_INFO("Initialized HazardLight API");
}

void Pacmod4Nl::initializeHeadlightApi()
{
  pub_tx_list.emplace(HEADLIGHT_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("headlight_rpt", 20));
  pub_tx_list.emplace(HEADLIGHT_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::HeadlightAuxRpt>("headlight_aux_rpt", 20));

  headlight_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "headlight_cmd", 20, &Pacmod4Nl::callback_headlight_set_cmd, this));
  auto headlight_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
  init_rx_msg(HEADLIGHT_CMD_CANID, headlight_cmd_msg);
  NODELET_INFO("Initialized Headlight API");
}

void Pacmod4Nl::initializeHornApi()
{
  pub_tx_list.emplace(HORN_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptBool>("horn_rpt", 20));

  horn_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "horn_cmd", 20, &Pacmod4Nl::callback_horn_set_cmd, this));
  auto horn_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdBool>();
  init_rx_msg(HORN_CMD_CANID, horn_cmd_msg);
  NODELET_INFO("Initialized Horn API");
}

void Pacmod4Nl::initializeInteriorLightsRptApi()
{
  pub_tx_list.emplace(INTERIOR_LIGHTS_RPT_CANID,
    nh_.advertise<pacmod4_msgs::InteriorLightsRpt>("interior_lights_rpt", 20));
  NODELET_INFO("Initialized InteriorLights API");
}

void Pacmod4Nl::initializeMediaControlsApi()
{
  pub_tx_list.emplace(MEDIA_CONTROLS_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("media_controls_rpt", 20));

  media_controls_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("media_controls_cmd", 20, &Pacmod4Nl::callback_media_controls_set_cmd, this));
  auto media_controls_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
  init_rx_msg(MEDIA_CONTROLS_CMD_CANID, media_controls_cmd_msg);
  NODELET_INFO("Initialized Media Controls API");
}

void Pacmod4Nl::initializeMotorRptApi(uint32_t can_id)
{
  switch (can_id)
  {
  case BRAKE_MOTOR_RPT_1_CANID:
    pub_tx_list.emplace(BRAKE_MOTOR_RPT_1_CANID, nh_.advertise<pacmod4_msgs::MotorRpt1>("brake_motor_rpt_1", 20));
    NODELET_INFO("Initialized BrakeMotorRpt 1 API");
    break;
  case BRAKE_MOTOR_RPT_2_CANID:
    pub_tx_list.emplace(BRAKE_MOTOR_RPT_2_CANID, nh_.advertise<pacmod4_msgs::MotorRpt2>("brake_motor_rpt_2", 20));
    NODELET_INFO("Initialized BrakeMotorRpt 2 API");
    break;
  case BRAKE_MOTOR_RPT_3_CANID:
    pub_tx_list.emplace(BRAKE_MOTOR_RPT_3_CANID, nh_.advertise<pacmod4_msgs::MotorRpt3>("brake_motor_rpt_3", 20));
    NODELET_INFO("Initialized BrakeMotorRpt 3 API");
    break;
  case STEERING_MOTOR_RPT_1_CANID:
    pub_tx_list.emplace(STEERING_MOTOR_RPT_1_CANID, nh_.advertise<pacmod4_msgs::MotorRpt1>("steering_motor_rpt_1", 20));
    NODELET_INFO("Initialized steeringMotorRpt 1 API");
    break;
  case STEERING_MOTOR_RPT_2_CANID:
    pub_tx_list.emplace(STEERING_MOTOR_RPT_2_CANID, nh_.advertise<pacmod4_msgs::MotorRpt2>("steering_motor_rpt_2", 20));
    NODELET_INFO("Initialized steeringMotorRpt 2 API");
    break;
  case STEERING_MOTOR_RPT_3_CANID:
    pub_tx_list.emplace(STEERING_MOTOR_RPT_3_CANID, nh_.advertise<pacmod4_msgs::MotorRpt3>("steering_motor_rpt_3", 20));
    NODELET_INFO("Initialized steeringMotorRpt 3 API");
    break;
  }
}

void Pacmod4Nl::initializeOccupancyRptApi()
{
  pub_tx_list.emplace(OCCUPANCY_RPT_CANID, nh_.advertise<pacmod4_msgs::OccupancyRpt>("occupancy_rpt", 20));
  NODELET_INFO("Initialized OccupancyRpt API");
}

void Pacmod4Nl::initializeParkingBrakeRptApi()
{
  pub_tx_list.emplace(PARKING_BRAKE_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptBool>("parking_brake_rpt", 20));
  NODELET_INFO("Initialized ParkingBrakeRpt API");
}

void Pacmod4Nl::initializeRearLightsRptApi()
{
  pub_tx_list.emplace(REAR_LIGHTS_RPT_CANID, nh_.advertise<pacmod4_msgs::RearLightsRpt>("rear_lights_rpt", 20));
  NODELET_INFO("Initialized RearLightsRpt API");
}

void Pacmod4Nl::initializeSteeringRpt2Api()
{
  pub_tx_list.emplace(STEERING_RPT_2_CANID, nh_.advertise<pacmod4_msgs::SystemRptFloat>("steering_rpt_2", 20));
  pub_tx_list.emplace(STEERING_AUX_RPT_2_CANID, nh_.advertise<pacmod4_msgs::SteeringAuxRpt>("steering_aux_rpt_2", 20));
  NODELET_INFO("Initialized Steering Rpt 2 API");
}

void Pacmod4Nl::initializeTipperBodyApi(uint32_t can_id)
{
  switch (can_id) {
    case TIPPER_BODY_RPT_00_CANID:
    {
      pub_tx_list.emplace(TIPPER_BODY_RPT_00_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("tipper_body_rpt_00", 20));

      tipper_body_00_set_cmd_sub = std::make_shared<ros::Subscriber>(
        nh_.subscribe("tipper_body_cmd_00", 20, &Pacmod4Nl::callback_tipper_body_00_set_cmd, this));
      auto tipper_body_00_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
      init_rx_msg(TIPPER_BODY_RPT_00_CANID, tipper_body_00_cmd_msg);
      NODELET_INFO("Initialized Tipper Body 00 API");
    }
    case TIPPER_BODY_RPT_01_CANID:
    {
      // pub_tx_list.emplace(TIPPER_BODY_RPT_01_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("tipper_body_rpt_01", 20));
    }
    case TIPPER_BODY_RPT_02_CANID:
    {

    }
  }
}

void Pacmod4Nl::initializeWheelSpeedApi(uint32_t can_id)
{
  switch (can_id)
  {
  case WHEEL_SPEED_RPT_CANID:
    pub_tx_list.emplace(WHEEL_SPEED_RPT_CANID, nh_.advertise<pacmod4_msgs::WheelSpeedRpt>("wheel_speed_rpt", 20));
    NODELET_INFO("Initialized Wheel Speed Rpt API");
    break;
  case WHEEL_SPEED_RPT_2_CANID:
    pub_tx_list.emplace(WHEEL_SPEED_RPT_2_CANID, nh_.advertise<pacmod4_msgs::WheelSpeedRpt>("wheel_speed_rpt_2", 20));
    NODELET_INFO("Initialized Wheel Speed Rpt 2 API");
    break;
  }
}

void Pacmod4Nl::initializeWiperApi()
{
  pub_tx_list.emplace(WIPER_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("wiper_rpt", 20));
  pub_tx_list.emplace(WIPER_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::WiperAuxRpt>("wiper_aux_rpt", 20));

  wiper_set_cmd_sub = std::make_shared<ros::Subscriber>(nh_.subscribe(
    "wiper_cmd", 20, &Pacmod4Nl::callback_wiper_set_cmd, this));
  auto wiper_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
  init_rx_msg(WIPER_CMD_CANID, wiper_cmd_msg);

  NODELET_INFO("Initialized Wiper API");
}

// vehicle specific apis

void Pacmod4Nl::initializeVehicle0SpecificApi()
{
  pub_tx_list.emplace(AIR_PRESSURE_RPT_CANID, nh_.advertise<pacmod4_msgs::AirPressureRpt>("air_pressure_rpt", 20));
  pub_tx_list.emplace(DRIVE_TRAIN_FEATURE_RPT_CANID, nh_.advertise<pacmod4_msgs::DrivetrainFeatureRpt>("drivetrain_feature_rpt", 20));
  pub_tx_list.emplace(ENGINE_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::EngineAuxRpt>("engine_aux_rpt", 20));
  pub_tx_list.emplace(ENGINE_AUX_RPT_2_CANID, nh_.advertise<pacmod4_msgs::EngineAuxRpt2>("engine_aux_rpt_2", 20));
  pub_tx_list.emplace(ENGINE_LOAD_FACTOR_RPT_CANID, nh_.advertise<pacmod4_msgs::EngineLoadFactorRpt>("engine_load_factor_rpt", 20));
  pub_tx_list.emplace(EXHAUST_BRAKE_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptInt>("exhaust_brake_rpt", 20));
  pub_tx_list.emplace(NOTIFICATION_RPT_CANID, nh_.advertise<pacmod4_msgs::NotificationRpt>("notification_rpt", 20));
  pub_tx_list.emplace(PARKING_BRAKE_AUX_RPT_CANID, nh_.advertise<pacmod4_msgs::ParkingBrakeAuxRpt>("parking_brake_aux_rpt", 20));
  pub_tx_list.emplace(PARKING_BRAKE_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptBoolWithControlStatus>("parking_brake_rpt", 20));
  pub_tx_list.emplace(REMOTE_STOP_RPT_CANID, nh_.advertise<pacmod4_msgs::RemoteStopRpt>("remote_stop_rpt", 20));
  pub_tx_list.emplace(SAFETY_FUNC_RPT_2_CANID, nh_.advertise<pacmod4_msgs::SafetyFuncRpt2>("safety_func_rpt_2", 20));
  pub_tx_list.emplace(SPRAYER_RPT_CANID, nh_.advertise<pacmod4_msgs::SystemRptBool>("sprayer_rpt", 20));
  pub_tx_list.emplace(STEERING_AUX_RPT_2_CANID, nh_.advertise<pacmod4_msgs::SteeringAuxRpt>("steering_aux_rpt_2", 20));
  pub_tx_list.emplace(STEERING_RPT_2_CANID, nh_.advertise<pacmod4_msgs::SystemRptFloat>("steering_rpt_2", 20));

  engine_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("engine_brake_cmd", 20, &Pacmod4Nl::callback_engine_brake_set_cmd, this));
  auto engine_brake_cmd_msg = std::make_shared<pacmod4_msgs::EngineBrakeCmd>();
  init_rx_msg(ENGINE_BRAKE_CMD_CANID, engine_brake_cmd_msg);

  exhaust_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("exhaust_brake_cmd", 20, &Pacmod4Nl::callback_exhaust_brake_set_cmd, this));
  auto exhaust_brake_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdBool>();
  init_rx_msg(EXHAUST_BRAKE_CMD_CANID, exhaust_brake_cmd_msg);

  global_cmd_sub = std::make_shared<ros::Subscriber>( 
    nh_.subscribe("global_cmd", 20, &Pacmod4Nl::callback_global_set_cmd, this));
  auto global_cmd_msg = std::make_shared<pacmod4_msgs::GlobalCmd>();
  init_rx_msg(GLOBAL_CMD_CANID, global_cmd_msg);

  parking_brake_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("parking_brake_cmd", 20, &Pacmod4Nl::callback_parking_brake_set_cmd, this));
  auto parking_brake_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdBool>();
  init_rx_msg(PARKING_BRAKE_CMD_CANID, parking_brake_cmd_msg);

  safety_func_set_cmd_sub = std::make_shared<ros::Subscriber>(
    nh_.subscribe("safety_func_cmd", 20, &Pacmod4Nl::callback_safety_func_set_cmd, this));
  auto safety_func_cmd_msg = std::make_shared<pacmod4_msgs::SafetyFuncCmd>();
  init_rx_msg(SAFETY_FUNC_CMD_CANID, safety_func_cmd_msg);

  sprayer_set_cmd_sub = std::make_shared<ros::Subscriber>( 
    nh_.subscribe("sprayer_cmd", 20, &Pacmod4Nl::callback_sprayer_set_cmd, this));
  auto sprayer_cmd_msg = std::make_shared<pacmod4_msgs::SystemCmdInt>();
  init_rx_msg(SPRAYER_CMD_CANID, sprayer_cmd_msg);

  NODELET_INFO("Initialized Vehicle0-specific API");
}

void Pacmod4Nl::initializeApiForMsg(uint32_t msg_can_id)
{
  // Need to initialize pubs/subs for this message group
  switch (msg_can_id)
  {
    case AIR_PRESSURE_RPT_CANID:
    case DRIVE_TRAIN_FEATURE_RPT_CANID:
    case ENGINE_AUX_RPT_CANID:
    case ENGINE_AUX_RPT_2_CANID:
    case SAFETY_FUNC_RPT_2_CANID:
      {
        initializeVehicle0SpecificApi();
        break;
      }
    case ANG_VEL_RPT_CANID:
      {
        initializeAngVelRptApi();
        break;
      }
    case AUTOMS_MAN_SWITCH_RPT_CANID:
      {
        initializeAutomsManSwitchRptApi();
        break;
      }
    case BATTERY_VOLTAGE_LEVEL_RPT_1_CANID:
    case BATTERY_VOLTAGE_LEVEL_RPT_2_CANID:
      {
        initializeBatteryVoltageLevelRptApi(msg_can_id);
        break;
      }
    case BRAKE_MOTOR_RPT_1_CANID:
    case BRAKE_MOTOR_RPT_2_CANID:
    case BRAKE_MOTOR_RPT_3_CANID:
    case STEERING_MOTOR_RPT_1_CANID:
    case STEERING_MOTOR_RPT_2_CANID:
    case STEERING_MOTOR_RPT_3_CANID:
      {
        initializeMotorRptApi(msg_can_id);
        break;
      }
    case BRAKE_DECEL_AUX_RPT_CANID:
    case BRAKE_DECEL_RPT_CANID:
      {
        initializeBrakeDecelApi();
        break;
      }
    case BRAKE_RPT_2_CANID:
    case BRAKE_AUX_RPT_2_CANID:
      {
        initializeBrakeRpt2Api();
      }
    case COMPONENT_RPT_00_CANID:
    case COMPONENT_RPT_01_CANID:
    case COMPONENT_RPT_02_CANID:
    case COMPONENT_RPT_03_CANID:
    case COMPONENT_RPT_04_CANID:
    case COMPONENT_RPT_05_CANID:
      {
        initializeComponentRptApi(msg_can_id);
        break;
      }
    case DOOR_RPT_CANID:
      {
        initializeDoorRptApi();
        break;
      }
    case ENGINE_BRAKE_RPT_CANID:
    case ENGINE_BRAKE_AUX_RPT_CANID:
      {
        initializeEngineBrakeApi();
      }
    case ENGINE_RPT_CANID:
      {
        initializeEngineRptApi();
        break;
      }
    case ESTOP_RPT_CANID:
      {
        initializeEStopRptApi();
        break;
      }
    case GLOBAL_RPT_CANID:
    case GLOBAL_RPT_2_CANID:
      {
        initializeGlobalRptApi(msg_can_id);
        break;
      }
    case HAZARD_LIGHTS_RPT_CANID:
      {
        initializeHazardLightApi();
        break;
      }
    case HEADLIGHT_RPT_CANID:
    case HEADLIGHT_AUX_RPT_CANID:
      {
        initializeHeadlightApi();
        break;
      }
    case HORN_RPT_CANID:
      {
        initializeHornApi();
        break;
      }
    case INTERIOR_LIGHTS_RPT_CANID:
      {
        initializeInteriorLightsRptApi();
        break;
      }
    case MEDIA_CONTROLS_RPT_CANID:
      {
        initializeMediaControlsApi();
        break;
      }
    case OCCUPANCY_RPT_CANID:
      {
        initializeOccupancyRptApi();
        break;
      }
    case PARKING_BRAKE_RPT_CANID:
      {
        initializeParkingBrakeRptApi();
        break;
      }
    case REAR_LIGHTS_RPT_CANID:
      {
        initializeRearLightsRptApi();
        break;
      }
    case STEERING_RPT_2_CANID:
    case STEERING_AUX_RPT_2_CANID:
      {
        initializeSteeringRpt2Api();
        break;
      }
    case TIPPER_BODY_RPT_00_CANID:
    case TIPPER_BODY_RPT_01_CANID:
    case TIPPER_BODY_RPT_02_CANID:
      {
        initializeTipperBodyApi(msg_can_id);
      }
    case WHEEL_SPEED_RPT_CANID:
    case WHEEL_SPEED_RPT_2_CANID:
      {
        initializeWheelSpeedApi(msg_can_id);
        break;
      }
    case WIPER_RPT_CANID:
    case WIPER_AUX_RPT_CANID:
      {
        initializeWiperApi();
        break;
      }
  }
}

void Pacmod4Nl::callback_accel_cmd_sub(const pacmod4_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(ACCEL_CMD_CANID, msg);
}

void Pacmod4Nl::callback_brake_cmd_sub(const pacmod4_msgs::SystemCmdFloat::ConstPtr& msg)
{
  lookup_and_encode(BRAKE_CMD_CANID, msg);
}

void Pacmod4Nl::callback_brake_decel_set_cmd(const pacmod4_msgs::BrakeDecelCmd::ConstPtr& msg)
{
  lookup_and_encode(BRAKE_DECEL_CMD_CANID, msg);
}

void Pacmod4Nl::callback_cabin_climate_set_cmd(const pacmod4_msgs::CabinClimateCmd::ConstPtr& msg)
{
  lookup_and_encode(CABIN_CLIMATE_CMD_CANID, msg);
}

void Pacmod4Nl::callback_cabin_fan_speed_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(CABIN_FAN_SPEED_CMD_CANID, msg);
}

void Pacmod4Nl::callback_cruise_control_buttons_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(CRUISE_CONTROL_BUTTONS_CMD_CANID, msg);
}

void Pacmod4Nl::callback_dash_controls_left_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(DASH_CONTROLS_LEFT_CMD_CANID, msg);
}

void Pacmod4Nl::callback_dash_controls_right_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(DASH_CONTROLS_RIGHT_CMD_CANID, msg);
}

void Pacmod4Nl::callback_differential_locks_set_cmd(const pacmod4_msgs::DifferentialLocksCmd::ConstPtr& msg)
{
  lookup_and_encode(DIFFERENTIAL_LOCKS_CMD_CANID, msg);
}

void Pacmod4Nl::callback_engine_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(ENGINE_CMD_CANID, msg);
}

void Pacmod4Nl::callback_engine_brake_set_cmd(const pacmod4_msgs::EngineBrakeCmd::ConstPtr& msg)
{
  lookup_and_encode(ENGINE_BRAKE_CMD_CANID, msg);
}

void Pacmod4Nl::callback_exhaust_brake_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(EXHAUST_BRAKE_CMD_CANID, msg);
}

void Pacmod4Nl::callback_global_set_cmd(const pacmod4_msgs::GlobalCmd::ConstPtr& msg)
{
  lookup_and_encode(GLOBAL_CMD_CANID, msg);
}

void Pacmod4Nl::callback_hazard_lights_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HAZARD_LIGHTS_CMD_CANID, msg);
}

void Pacmod4Nl::callback_headlight_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(HEADLIGHT_CMD_CANID, msg);
}

void Pacmod4Nl::callback_horn_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(HORN_CMD_CANID, msg);
}

void Pacmod4Nl::callback_marker_lamp_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(MARKER_LAMP_CMD_CANID, msg);
}

void Pacmod4Nl::callback_media_controls_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(MEDIA_CONTROLS_CMD_CANID, msg);
}

void Pacmod4Nl::callback_notification_set_cmd(const pacmod4_msgs::NotificationCmd::ConstPtr& msg)
{
  lookup_and_encode(NOTIFICATION_CMD_CANID, msg);
}

void Pacmod4Nl::callback_parking_brake_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(PARKING_BRAKE_CMD_CANID, msg);
}

void Pacmod4Nl::callback_power_take_off_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(POWER_TAKE_OFF_CMD_CANID, msg);
}

void Pacmod4Nl::callback_safety_func_set_cmd(const pacmod4_msgs::SafetyFuncCmd::ConstPtr& msg)
{
  lookup_and_encode(SAFETY_FUNC_CMD_CANID, msg);
}

void Pacmod4Nl::callback_shift_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(SHIFT_CMD_CANID, msg);
}

void Pacmod4Nl::callback_sprayer_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg)
{
  lookup_and_encode(SPRAYER_CMD_CANID, msg);
}

void Pacmod4Nl::callback_steer_cmd_sub(const pacmod4_msgs::SteeringCmd::ConstPtr& msg)
{
  lookup_and_encode(STEERING_CMD_CANID, msg);
}

void Pacmod4Nl::callback_tipper_body_00_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TIPPER_BODY_CMD_00, msg);
}

void Pacmod4Nl::callback_tipper_body_01_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TIPPER_BODY_CMD_01, msg);
}

void Pacmod4Nl::callback_tipper_body_02_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TIPPER_BODY_CMD_02, msg);
}

void Pacmod4Nl::callback_turn_signal_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(TURN_CMD_CANID, msg);
}

void Pacmod4Nl::callback_rear_pass_door_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(REAR_PASS_DOOR_CMD_CANID, msg);
}

void Pacmod4Nl::callback_wiper_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg)
{
  lookup_and_encode(WIPER_CMD_CANID, msg);
}

void Pacmod4Nl::SystemStatusUpdate(const ros::TimerEvent& event)
{
  pacmod4_msgs::AllSystemStatuses ss_msg;

  std::unique_lock<std::mutex> lock(sys_status_mutex_);
  for (auto system = system_statuses.begin(); system != system_statuses.end(); ++system)
  {
    pacmod4_msgs::KeyValuePair kvp;

    if (system->first == ACCEL_RPT_CANID)
      kvp.key = "Accelerator";
    else if (system->first == BRAKE_RPT_CANID)
      kvp.key = "Brakes";
    else if (system->first == CRUISE_CONTROL_BUTTONS_RPT_CANID)
      kvp.key = "Cruise Control Buttons";
    else if (system->first == HAZARD_LIGHTS_RPT_CANID)
      kvp.key = "Hazard Lights";
    else if (system->first == HEADLIGHT_RPT_CANID)
      kvp.key = "Headlights";
    else if (system->first == HORN_RPT_CANID)
      kvp.key = "Horn";
    else if (system->first == MEDIA_CONTROLS_RPT_CANID)
      kvp.key = "Media Controls";
    else if (system->first == PARKING_BRAKE_RPT_CANID)
      kvp.key = "Parking Brake";
    else if (system->first == SHIFT_RPT_CANID)
      kvp.key = "Shifter";
    else if (system->first == STEERING_RPT_CANID)
      kvp.key = "Steering";
    else if (system->first == TURN_RPT_CANID)
      kvp.key = "Turn Signals";
    else if (system->first == REAR_PASS_DOOR_RPT_CANID)
      kvp.key = "Rear Passenger Door";
    else if (system->first == WIPER_RPT_CANID)
      kvp.key = "Wipers";

    kvp.value = std::get<0>(system->second) ? "True" : "False";

    ss_msg.enabled_status.push_back(kvp);

    kvp.value = std::get<1>(system->second) ? "True" : "False";

    ss_msg.overridden_status.push_back(kvp);

    kvp.value = std::get<2>(system->second) ? "True" : "False";

    ss_msg.fault_status.push_back(kvp);
  }

  all_system_statuses_pub.publish(ss_msg);
}

void Pacmod4Nl::can_write(const ros::TimerEvent& event)
{
  for (const auto& can_id : received_cmds_)
  {
    auto data = rx_list[can_id]->getData();

    can_msgs::Frame frame;
    frame.id = can_id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    frame.dlc = data.size();
    std::move(data.begin(), data.end(), frame.data.begin());

    frame.header.stamp = ros::Time::now();

    can_rx_pub.publish(frame);

    std::this_thread::sleep_for(std::chrono::milliseconds(INTER_MSG_PAUSE));
  }
}

void Pacmod4Nl::can_read(const can_msgs::Frame::ConstPtr &msg)
{
  auto pub = pub_tx_list.find(msg->id);

  if (pub == pub_tx_list.end())
  {
    initializeApiForMsg(msg->id);
  }

  // Only parse messages for which we have a publisher.
  if (pub != pub_tx_list.end())
  {
    handler->ParseAndPublish(*msg, pub->second);

    if (msg->id == GLOBAL_RPT_CANID)
    {
      pacmod4_msgs::GlobalRpt global_rpt_msg;
      handler->ParseType(*msg, global_rpt_msg);

      std_msgs::Bool bool_msg;
      bool_msg.data = global_rpt_msg.enabled;
      enabled_pub.publish(bool_msg);

      // Auto-disable
      if (global_rpt_msg.override_active ||
          global_rpt_msg.pacmod_sys_fault_active ||
          global_rpt_msg.config_fault_active)
      {
        set_enable(false);
      }
    }
    if (msg->id == GLOBAL_RPT_2_CANID)
    {
      pacmod4_msgs::GlobalRpt2 global_rpt_2_msg;
      handler->ParseType(*msg, global_rpt_2_msg);

      std_msgs::Bool bool_msg;
      bool_msg.data = global_rpt_2_msg.system_enabled;
      enabled_pub.publish(bool_msg);

      // Auto-disable
      if (global_rpt_2_msg.system_override_active ||
          global_rpt_2_msg.system_fault_active)
      {
        set_enable(false);
      }
    }
  }
}

// Sets the PACMod4 enable flag through CAN.
void Pacmod4Nl::set_enable(bool val)
{
  for (auto & cmd : rx_list)
  {
    // safety_func_cmd doesn't follow the normal command
    // message design
    // real-life pacmod subsystem enabled = subsystem enable bit high AND safety_func_cmd.command > 0
    if (cmd.first != SAFETY_FUNC_CMD_CANID)
    {
      // This assumes that all data in rx_list are encoded
      // command messages which means the least significant
      // bit in their first byte will be the enable flag.
      std::vector<uint8_t> current_data = cmd.second->getData();

      if (val)
        current_data[0] |= 0x01;  // Enable true
      else
        current_data[0] &= 0xFE;  // Enable false

      cmd.second->setData(current_data);
    }
  }
}

// Looks up the appropriate LockedData and inserts the command info
template<class RosMsgType>
void Pacmod4Nl::lookup_and_encode(const uint32_t& can_id, const RosMsgType& msg)
{
  auto rx_it = rx_list.find(can_id);
  if (rx_it != rx_list.end())
  {
    can_msgs::Frame packed_frame = handler->Encode(can_id, msg);

    std::vector<unsigned char> new_data;
    new_data.resize(8);
    std::move(packed_frame.data.begin(), packed_frame.data.end(), new_data.begin());
    new_data.resize(packed_frame.dlc);

    rx_it->second->setData(new_data);
    received_cmds_.insert(can_id);
  }
  else
  {
    ROS_WARN("Received command message for ID 0x%x for which we did not have an encoder.", can_id);
  }
}

template<class RosMsgType>
void Pacmod4Nl::init_rx_msg(const uint32_t& can_id, const RosMsgType& msg)
{
  rx_list.emplace(
    can_id,
    std::shared_ptr<LockedData>(new LockedData()));
  lookup_and_encode(can_id, msg);
}
}  // namespace pacmod4

PLUGINLIB_EXPORT_CLASS(pacmod4::Pacmod4Nl, nodelet::Nodelet);
