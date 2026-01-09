cmake_minimum_required(VERSION 3.20)
project(continuum_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EchoRequest.msg"
  "msg/EchoResponse.msg"
  "msg/JoystickButtonEvent.msg"
  "msg/JoystickAxisEvent.msg"
  "msg/MicRequest.msg"
  "msg/MicResponse.msg"
  "msg/MicStreamingResponse.msg"
  "msg/VadRequest.msg"
  "msg/VadResponse.msg"
  "msg/VadStreamingResponse.msg"
  "msg/AsrRequest.msg"
  "msg/AsrResponse.msg"
  "msg/AsrStreamingResponse.msg"
  "msg/LlmRequest.msg"
  "msg/LlmResponse.msg"
  "msg/LlmStreamingResponse.msg"
  "msg/TtsRequest.msg"
  "msg/TtsResponse.msg"
  "msg/TtsStreamingResponse.msg"
  "msg/AgentRequest.msg"
  "msg/AgentResponse.msg"
  "msg/AgentStreamingResponse.msg"
  "msg/DictationRequest.msg"
  "msg/DictationResponse.msg"
  "msg/DictationStreamingResponse.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
