cmake_minimum_required(VERSION 2.8.3)
project(low_level_intent_predictor_simulation)
add_compile_options(-std=c++11)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


find_package(Eigen3 REQUIRED)

include_directories(${INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})
add_executable(low_level_intent_predictor_simulator 
  ${SRC_DIR}/low_level_intent_predictor_simulation.cpp
  ${SRC_DIR}/LowLevelIntentPredictorSimulator.cpp
  ${SRC_DIR}/Hand.cpp
  ${SRC_DIR}/LpvDs.cpp
  )

find_package(PythonLibs 2.7)
target_include_directories(low_level_intent_predictor_simulator PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(low_level_intent_predictor_simulator ${PYTHON_LIBRARIES})
