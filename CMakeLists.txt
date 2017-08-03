cmake_minimum_required(VERSION 2.8)

project(front_3dmap)

list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/Ceres")

find_package(PCL 1.7 REQUIRED)

find_package(Eigen 3 REQUIRED)

find_package(Ceres REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(front_3dmap main.cpp)

add_executable(pose_graph posegraph.cpp)

add_executable(vizopimizedpointcloud vizopimizedpointcloud.cpp)


target_link_libraries(front_3dmap ${PCL_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(pose_graph ${PCL_LIBRARIES} ${CERES_LIBRARIES})
target_link_libraries(vizopimizedpointcloud  ${PCL_LIBRARIES} ${CERES_LIBRARIES})
#install(TARGETS front_3dmap RUNTIME DESTINATION bin)
