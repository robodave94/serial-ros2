cmake_minimum_required(VERSION 3.5)
project(serial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

## Sources
## Add serial library
add_library(${PROJECT_NAME}
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if(APPLE) # macOS
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_osx.cc
    )
     target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # .*nix
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_linux.cc
     )
         target_link_libraries(${PROJECT_NAME} rt pthread)
elseif(WIN32) # Windows
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc
    )
      target_link_libraries(${PROJECT_NAME} setupapi)
    ament_export_libraries(setupapi)
endif()


## Include headers
target_include_directories(${PROJECT_NAME} PRIVATE include)

## Uncomment for example
# add_executable(serial_example examples/serial_example.cc)
# add_dependencies(serial_example ${PROJECT_NAME})
# target_link_libraries(serial_example ${PROJECT_NAME})

## Autofind devices ROS2 node
add_executable(autofind_devices src/autofind_devices.cpp)
target_include_directories(autofind_devices PRIVATE include)
target_link_libraries(autofind_devices ${PROJECT_NAME})
ament_target_dependencies(autofind_devices rclcpp)


## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

## Install autofind_devices node
install(TARGETS autofind_devices
    DESTINATION lib/${PROJECT_NAME}
)

## Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
    DESTINATION include/serial
)

## Tests
#if(BUILD_TESTING)
#    add_subdirectory(tests)
#endif()

ament_package()
