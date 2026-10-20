cmake_minimum_required(VERSION 3.8)
project(rob_box_dialog_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Типы диалога v2 (ADR-0150 §10.1). SceneFrame и ProviderState - вложенные
# типы (SessionState, ProviderStateArray, DialogControl.srv).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpeechRequest.msg"
  "msg/SpeechCancel.msg"
  "msg/SpeechEvent.msg"
  "msg/SpeechEnvelope.msg"
  "msg/ExpressionRequest.msg"
  "msg/ExpressionEvent.msg"
  "msg/SoundEvent.msg"
  "msg/PlanEvent.msg"
  "msg/SceneFrame.msg"
  "msg/SessionState.msg"
  "msg/TurnEvent.msg"
  "msg/ActionRequest.msg"
  "msg/ActionResult.msg"
  "msg/ProviderState.msg"
  "msg/ProviderStateArray.msg"
  "msg/DialogOutput.msg"
  "msg/Intent.msg"
  "msg/Utterance.msg"
  "msg/SpeakerResult.msg"
  "srv/DialogControl.srv"
  DEPENDENCIES
    builtin_interfaces
)

ament_package()
