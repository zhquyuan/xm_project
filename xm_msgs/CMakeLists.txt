cmake_minimum_required(VERSION 2.8.3)
project(xm_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roslib
  sensor_msgs
  std_msgs
  message_generation
  actionlib_msgs
#  message_runtime #肖加于1.10通过此法解决了编译时找不到消息类头文件的情况
)#该死，似乎只是暂时解决了，1.27重新碰到了这个问题，先编译大法好 1.27发现修改package.xml后注释最好

 add_message_files(
   FILES
   xm_Task.msg
   xm_Object.msg
   xm_Person.msg
   xm_Gesture.msg
   xm_TaskArray.msg
   xm_FollowPerson.msg
   xm_GripperCommand.msg
   xm_detect_request.msg
   xm_detect_response.msg
   xm_ObjList.msg
 )

 add_service_files(
   FILES
   xm_ObjectDetect.srv
   xm_IkObtain.srv
   xm_PickPlace.srv
   xm_Move.srv
   xm_FaceRecognize.srv
   xm_Look.srv
   xm_SolveIK.srv
   xm_PickOrPlace.srv
   xm_Gripper.srv
   xm_Question.srv
   xm_Speak.srv
   xm_gender.srv
   xm_Plat.srv
   xm_Speech.srv
   xm_Speech_meaning.srv
   xm_Obstacles.srv
   xm_People.srv
   xm_SpeechName.srv
   xm_Speech_tts.srv
   xm_house.srv
 )

 add_action_files(
  DIRECTORY action
  FILES xm_GripperBool.action
)


 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs	actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS geometry_msgs roslib sensor_msgs std_msgs actionlib_msgs 
  message_runtime #肖加于1.10 message_generation删于1.27
  #DEPENDS system_lib肖注释于1.10
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
