cmake_minimum_required(VERSION 2.8.3)
project(morai_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

 add_message_files(
   DIRECTORY msg
   FILES
   CtrlCmd.msg
   EgoVehicleStatus.msg
   GPSMessage.msg
   GhostMessage.msg
   ObjectStatusList.msg
   ObjectStatus.msg
   TrafficLight.msg
   ERP42Info.msg
   GetTrafficLightStatus.msg
   SetTrafficLight.msg
   IntersectionControl.msg
   IntersectionStatus.msg
   CollisionData.msg
   MultiEgoSetting.msg
   IntscnTL.msg
   SensorPosControl.msg
   MoraiSimProcHandle.msg
   MoraiSimProcStatus.msg
   MoraiSrvResponse.msg
   ScenarioLoad.msg
   MoraiTLIndex.msg
   MoraiTLInfo.msg
   SaveSensorData.msg
   ReplayInfo.msg
   EventInfo.msg
   Lamps.msg
   VehicleSpec.msg
   VehicleSpecIndex.msg
   NpcGhostCmd.msg
   NpcGhostInfo.msg
   VehicleCollisionData.msg
   VehicleCollision.msg
   SyncModeAddObj.msg
   SyncModeInfo.msg
   WaitForTickResponse.msg
   SyncModeCmd.msg   
   SyncModeRemoveObj.msg
   SyncModeCmdResponse.msg  
   WaitForTick.msg
   MapSpec.msg
   MapSpecIndex.msg
   SyncModeCtrlCmd.msg
   SyncModeSetGear.msg
   SyncModeResultResponse.msg
   SyncModeScenarioLoad.msg
   RadarTrack.msg
   RadarTracks.msg
 )

 add_service_files(
   DIRECTORY srv
   FILES
   MoraiScenarioLoadSrv.srv
   MoraiSimpProcSrv.srv
   MoraiTLInfoSrv.srv
   MoraiEventCmdSrv.srv
   MoraiVehicleSpecSrv.srv
   MoraiSyncModeCmdSrv.srv
   MoraiWaitForTickSrv.srv
   MoraiMapSpecSrv.srv
   MoraiSyncModeCtrlCmdSrv.srv
   MoraiSyncModeSetGearSrv.srv
   MoraiSyncModeSLSrv.srv
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS 
  std_msgs
  geometry_msgs
  message_runtime
)

