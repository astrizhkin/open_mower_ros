cmake_minimum_required(VERSION 3.0.2)
project(e3_lib)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        )

add_message_files(
        FILES
        E3KVInput.msg
        )

add_service_files(
        FILES
        ScheduleE3KV.srv
        )

generate_messages(
        DEPENDENCIES
        std_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES e3_lib
        CATKIN_DEPENDS roscpp std_msgs message_runtime
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        )

# Header-only library — dummy source so catkin can link it
add_library(e3_lib src/e3_lib_dummy.cpp)
add_dependencies(e3_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(e3_lib ${catkin_LIBRARIES})

# Bridge node
add_executable(radio_bridge_node src/radio_bridge_node.cpp)
add_dependencies(radio_bridge_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(radio_bridge_node e3_lib ${catkin_LIBRARIES})
