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

#ifndef PACMOD4_PACMOD4_NODELET_H
#define PACMOD4_PACMOD4_NODELET_H

#include "pacmod4/pacmod4_ros_msg_handler.h"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <can_msgs/Frame.h>
#include <pacmod4_msgs/AccelAuxRpt.h>
#include <pacmod4_msgs/AirPressureRpt.h>
#include <pacmod4_msgs/AllSystemStatuses.h>
#include <pacmod4_msgs/AngVelRpt.h>
#include <pacmod4_msgs/AutomsManSwitchRpt.h>
#include <pacmod4_msgs/BatteryVoltageLevelRpt.h>
#include <pacmod4_msgs/BrakeAuxRpt.h>
#include <pacmod4_msgs/BrakeDecelAuxRpt.h>
#include <pacmod4_msgs/CabinClimateRpt.h>
#include <pacmod4_msgs/ComponentRpt.h>
#include <pacmod4_msgs/DateTimeRpt.h>
#include <pacmod4_msgs/DetectedObjectRpt.h>
#include <pacmod4_msgs/DifferentialLocksCmd.h>
#include <pacmod4_msgs/DifferentialLocksRpt.h>
#include <pacmod4_msgs/DoorRpt.h>
#include <pacmod4_msgs/DrivetrainFeatureRpt.h>
#include <pacmod4_msgs/EStopRpt.h>
#include <pacmod4_msgs/EngineAuxRpt.h>
#include <pacmod4_msgs/EngineAuxRpt2.h>
#include <pacmod4_msgs/EngineBrakeAuxRpt.h>
#include <pacmod4_msgs/EngineBrakeCmd.h>
#include <pacmod4_msgs/EngineBrakeRpt.h>
#include <pacmod4_msgs/EngineLoadFactorRpt.h>
#include <pacmod4_msgs/FireSuppressionRpt.h>
#include <pacmod4_msgs/GlobalCmd.h>
#include <pacmod4_msgs/GlobalRpt.h>
#include <pacmod4_msgs/GlobalRpt2.h>
#include <pacmod4_msgs/GnssTime.h>
#include <pacmod4_msgs/HeadlightAuxRpt.h>
#include <pacmod4_msgs/InteriorLightsRpt.h>
#include <pacmod4_msgs/LatLonHeadingRpt.h>
#include <pacmod4_msgs/LinearAccelRpt.h>
#include <pacmod4_msgs/MotorRpt1.h>
#include <pacmod4_msgs/MotorRpt2.h>
#include <pacmod4_msgs/MotorRpt3.h>
#include <pacmod4_msgs/NotificationRpt.h>
#include <pacmod4_msgs/OccupancyRpt.h>
#include <pacmod4_msgs/ParkingBrakeAuxRpt.h>
#include <pacmod4_msgs/RearLightsRpt.h>
#include <pacmod4_msgs/RemoteStopRpt.h>
#include <pacmod4_msgs/SafetyBrakeCmd.h>
#include <pacmod4_msgs/SafetyBrakeRpt.h>
#include <pacmod4_msgs/SafetyFuncCmd.h>
#include <pacmod4_msgs/SafetyFuncCriticalStopRpt.h>
#include <pacmod4_msgs/SafetyFuncRpt.h>
#include <pacmod4_msgs/SafetyFuncRpt2.h>
#include <pacmod4_msgs/SafetyResponseRpt.h>
#include <pacmod4_msgs/ShiftAuxRpt.h>
#include <pacmod4_msgs/SoftwareVersionRpt.h>
#include <pacmod4_msgs/SteeringAuxRpt.h>
#include <pacmod4_msgs/SteeringCmd.h>
#include <pacmod4_msgs/SteeringCmdLimitRpt.h>
#include <pacmod4_msgs/SupervisoryCtrl.h>
#include <pacmod4_msgs/SystemCmdBool.h>
#include <pacmod4_msgs/SystemCmdFloat.h>
#include <pacmod4_msgs/SystemCmdInt.h>
#include <pacmod4_msgs/SystemCmdLimitRpt.h>
#include <pacmod4_msgs/SystemRptBool.h>
#include <pacmod4_msgs/SystemRptBoolWithControlStatus.h>
#include <pacmod4_msgs/SystemRptFloat.h>
#include <pacmod4_msgs/SystemRptInt.h>
#include <pacmod4_msgs/SystemRptIntWithControlStatus.h>
#include <pacmod4_msgs/TipperBodyAuxRpt.h>
#include <pacmod4_msgs/TirePressureExtendedRpt.h>
#include <pacmod4_msgs/TrailerBrakePressureRpt.h>
#include <pacmod4_msgs/TrailerFaultRpt.h>
#include <pacmod4_msgs/TrailerPayloadRpt.h>
#include <pacmod4_msgs/TrailerWheelSpeedRpt.h>
#include <pacmod4_msgs/TurnAuxRpt.h>
#include <pacmod4_msgs/UserPcHealthRpt.h>
#include <pacmod4_msgs/VehicleDynamicsRpt.h>
#include <pacmod4_msgs/VehicleFaultRpt.h>
#include <pacmod4_msgs/VehicleFaultRpt2.h>
#include <pacmod4_msgs/VehicleSpeedRpt.h>
#include <pacmod4_msgs/WatchdogRpt.h>
#include <pacmod4_msgs/WatchdogRpt2.h>
#include <pacmod4_msgs/WheelSpeedRpt.h>
#include <pacmod4_msgs/WheelSpeedRpt.h>
#include <pacmod4_msgs/WiperAuxRpt.h>
#include <pacmod4_msgs/YawRateRpt.h>

namespace pacmod4
{

  const uint32_t SEND_CMD_INTERVAL = 33;
  const uint32_t INTER_MSG_PAUSE = 1;
  const float PACMOD_UPDATE_FREQ = 30.0;

class Pacmod4Nl : public nodelet::Nodelet
{
private:
  void onInit() override;
  void loadParams();

  // Generic APIs
  void initializeAngVelRptApi();
  void initializeAutomsManSwitchRptApi();
  void initializeBatteryVoltageLevelRptApi(uint32_t can_id);
  void initializeBrakeDecelApi();
  void initializeBrakeRpt2Api();
  void initializeCabinClimateRpt();
  void initializeCmdLimitRpt(uint32_t can_id);
  void initializeComponentRptApi(uint32_t can_id);
  void initializeDoorRptApi();
  void initializeEStopRptApi();
  void initializeEngineBrakeApi();
  void initializeEngineRptApi();
  void initializeGlobalRptApi(uint32_t can_id);
  void initializeHazardLightApi();
  void initializeHeadlightApi();
  void initializeHornApi();
  void initializeInteriorLightsRptApi();
  void initializeMediaControlsApi();
  void initializeMotorRptApi(uint32_t can_id);
  void initializeOccupancyRptApi();
  void initializeParkingBrakeRptApi();
  void initializeRearLightsRptApi();
  void initializeSteeringRpt2Api();
  void initializeWheelSpeedApi(uint32_t can_id);
  void initializeWiperApi();

  // Vehicle-specific APIs
  void initializeVehicle0SpecificApi();

  void initializeApiForMsg(uint32_t msg_can_id);

  // ROS Callbacks
  void callback_accel_cmd_sub(const pacmod4_msgs::SystemCmdFloat::ConstPtr& msg);
  void callback_brake_cmd_sub(const pacmod4_msgs::SystemCmdFloat::ConstPtr& msg);
  void callback_brake_decel_set_cmd(const pacmod4_msgs::BrakeDecelCmd::ConstPtr& msg);
  void callback_cabin_climate_set_cmd(const pacmod4_msgs::CabinClimateCmd::ConstPtr& msg);
  void callback_cabin_fan_speed_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_cabin_temp_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_cruise_control_buttons_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_dash_controls_left_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_dash_controls_right_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_differential_locks_set_cmd(const pacmod4_msgs::DifferentialLocksCmd::ConstPtr& msg);
  void callback_engine_brake_set_cmd(const pacmod4_msgs::EngineBrakeCmd::ConstPtr& msg);
  void callback_engine_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_exhaust_brake_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_global_set_cmd(const pacmod4_msgs::GlobalCmd::ConstPtr& msg);
  void callback_hazard_lights_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_headlight_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_horn_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_marker_lamp_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_media_controls_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_notification_set_cmd(const pacmod4_msgs::NotificationCmd::ConstPtr& msg);
  void callback_parking_brake_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_power_take_off_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_rear_pass_door_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_safety_func_set_cmd(const pacmod4_msgs::SafetyFuncCmd::ConstPtr& msg);
  void callback_shift_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_sprayer_set_cmd(const pacmod4_msgs::SystemCmdBool::ConstPtr& msg);
  void callback_steer_cmd_sub(const pacmod4_msgs::SteeringCmd::ConstPtr& msg);
  void callback_turn_signal_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void callback_wiper_set_cmd(const pacmod4_msgs::SystemCmdInt::ConstPtr& msg);
  void SystemStatusUpdate(const ros::TimerEvent& event);

  void can_read(const can_msgs::Frame::ConstPtr &msg);
  void can_write(const ros::TimerEvent& event);
  void set_enable(bool val);
  template<class RosMsgType>
  void lookup_and_encode(const uint32_t& can_id, const RosMsgType& msg);

  std::unordered_map<uint32_t, ros::Publisher> pub_tx_list;
  std::unique_ptr<Pacmod4RosMsgHandler> handler;

  ros::Publisher enabled_pub;
  ros::Publisher can_rx_pub;
  ros::Publisher all_system_statuses_pub;

  ros::Subscriber can_tx_sub;
  ros::Subscriber accel_cmd_sub;
  ros::Subscriber brake_cmd_sub;
  ros::Subscriber shift_cmd_sub;
  ros::Subscriber steer_cmd_sub;
  ros::Subscriber turn_cmd_sub;

  ros::Timer status_update_timer_;
  ros::Timer can_send_timer_;

  // std::thread can_write_thread;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Vehicle-Specific Subscribers
  std::shared_ptr<ros::Subscriber> brake_decel_set_cmd,
      cabin_climate_set_cmd,
      cruise_control_buttons_set_cmd_sub,
      dash_controls_left_set_cmd_sub,
      dash_controls_right_set_cmd_sub,
      engine_brake_set_cmd_sub,
      engine_set_cmd_sub,
      exhaust_brake_set_cmd_sub,
      global_cmd_sub,
      hazard_lights_set_cmd_sub,
      headlight_set_cmd_sub,
      horn_set_cmd_sub,
      marker_lamp_set_cmd_sub,
      media_controls_set_cmd_sub,
      parking_brake_set_cmd_sub,
      rear_pass_door_cmd_sub,
      safety_func_set_cmd_sub,
      sprayer_set_cmd_sub,
      wiper_set_cmd_sub;


  int dbc_major_version_;

  // Commands that have been received from ROS subscribers
  std::set<uint32_t> received_cmds_;

  // Data shared across threads
  std::unordered_map<uint32_t, std::shared_ptr<LockedData>> rx_list;
  std::map<uint32_t, std::tuple<bool, bool, bool>> system_statuses;
  std::mutex sys_status_mutex_;
};

}  // namespace pacmod4


#endif  // PACMOD4_PACMOD4_NODELET_H
