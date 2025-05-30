cmake_minimum_required(VERSION 3.11)
#cmake_policy(SET CMP0074 NEW)
#cmake_policy(SET CMP0144 NEW)
#cmake_policy(SET CMP0167 NEW)
#cmake_policy(SET CMP0148 NEW)
project(hba)

set(CMAKE_BUILD_TYPE "Debug")
ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -std=c++17 -fexceptions")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl") 

#add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable -Wno-unused-but-set-variable)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GeographicLib REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(hba src/hba.cpp)
target_link_libraries(
  hba
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  gtsam
)

ament_auto_add_executable(calculate_MME src/calculate_MME.cpp)
target_link_libraries(
  calculate_MME
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  gtsam
)

ament_auto_add_executable(visualize src/visualize.cpp)
target_link_libraries(
  visualize
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

install(TARGETS
  hba										
  DESTINATION lib/${PROJECT_NAME}
  )

install(TARGETS
  calculate_MME
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  visualize
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
