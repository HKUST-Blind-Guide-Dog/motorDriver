cmake_minimum_required(VERSION 3.25)
project(motorsTest)

# include all directories
include_directories(
    ${PROJECT_SOURCE_DIR}/curi_seial
    ${PROJECT_SOURCE_DIR}/motor
)

# Add executable
add_executable(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/curi_serial/serial.h
  ${PROJECT_SOURCE_DIR}/curi_serial/serial.c
  ${PROJECT_SOURCE_DIR}/motorTest.cpp
  ${PROJECT_SOURCE_DIR}/motor/motor.cpp
  ${PROJECT_SOURCE_DIR}/motor/motor.h
)

target_link_libraries(${PROJECT_NAME} 
  PRIVATE "-lm -lpthread -lrt -Wall"
)

