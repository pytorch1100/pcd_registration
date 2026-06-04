cmake_minimum_required(VERSION 3.2)

project(icp)

find_package(PCL 1.8 REQUIRED COMPONENTS
    common
    io
    visualization
)
message(STATUS "PCL include dir: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL lib dirs: ${PCL_LIBRARIES}")

include_directories(
    ${PCL_INCLUDE_DIRS}
)

add_executable(1_icp_test 1_icp_test.cpp)
target_link_libraries(1_icp_test
    ${PCL_LIBRARIES}
)

add_executable(2_pcd_io 2_pcd_io.cpp)
target_link_libraries(2_pcd_io
    ${PCL_LIBRARIES}
)

add_executable(3_icp_svd 3_icp_svd.cpp)
target_link_libraries(3_icp_svd
    ${PCL_LIBRARIES}
)