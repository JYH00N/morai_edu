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
   CollisionData.msg
   CtrlCmd.msg
   EgoVehicleStatus.msg
   EventInfo.msg
   GPSMessage.msg
   Lamps.msg
   Mgeo.msg
   MgeoLink.msg
   MgeoLinkList.msg
   MgeoNode.msg
   MgeoNodeList.msg
   MoraiSimProcHandle.msg
   MoraiTimestamp.msg
   ObjectStatus.msg
   ObjectStatusList.msg
   TrafficLight.msg
   VehicleCommand.msg
   VehicleSpec.msg
   VehicleSpecIndex.msg
 )

 add_service_files(
   DIRECTORY srv
   FILES
   MoraiVehicleSpecSrv.srv
   MoraiEventCmdSrv.srv
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

