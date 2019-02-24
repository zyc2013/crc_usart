cmake_minimum_required(VERSION 2.8.3)
project(serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#添加标准库支持
set(CMAKE_CXX_FLAGS "-std=c++11")
#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})
#寻找opencv库
find_package(OpenCV REQUIRED)
add_executable(serial src/main.cpp src/usart.cpp)
target_link_libraries(serial ${catkin_LIBRARIES} ${OpenCV_LIBS} -lpthread)
add_executable(send src/send.cpp)
target_link_libraries(send ${catkin_LIBRARIES} )
add_executable(rserial src/recv.cpp src/usart.cpp)
target_link_libraries(rserial ${catkin_LIBRARIES} ${OpenCV_LIBS} -lpthread)
#add_executable(bringup src/riki_base_node_l.cpp)
#target_link_libraries(bringup ${catkin_LIBRARIES} )


