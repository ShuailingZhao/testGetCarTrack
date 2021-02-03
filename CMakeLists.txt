cmake_minimum_required(VERSION 2.8)
project(getDisFromHomograph)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
#set("OpenCV_DIR" "/usr/local/share/OpenCV")
set("OpenCV_DIR" "/home/zhaosl/dataDisk/zhaosl/local_install/opencv3.1.0/share/OpenCV/")
#set("OpenCV_DIR" "/home/zhaosl/local_install/opencv2.4.13/lib")
find_package( OpenCV REQUIRED )
MESSAGE("OpenCV vesion: ${OpenCV_VERSION}")
find_package( Ceres REQUIRED)


include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories( ${CERES_INCLUDE_DIRS})
include_directories("/home/zhaosl/dataDisk/zhaosl/local_install/eigen3.3.7/")

set(PROJECT_ROOT ".")
include_directories(${PROJECT_ROOT}/include)
set(MODULE_SRC_ROOT ${PROJECT_ROOT}/src/)

#file(GLOB src_testGetXYWithHomograph_files ${MODULE_SRC_ROOT}/testGetXYWithHomograph.cpp ${MODULE_SRC_ROOT}/getXYWithHomograph.cpp ${MODULE_SRC_ROOT}/CSVRow.cpp)
#add_executable(testGetXYWithHomograph ${src_testGetXYWithHomograph_files})
#target_link_libraries(testGetXYWithHomograph ${OpenCV_LIBS})

file(GLOB src_testGetXYWithGeometry_files ${MODULE_SRC_ROOT}/testGetXYWithGeometry.cpp ${MODULE_SRC_ROOT}/getXYWithGeometry.cpp ${MODULE_SRC_ROOT}/bA.cpp)
add_executable(testGetXYWithGeometry ${src_testGetXYWithGeometry_files})
target_link_libraries(testGetXYWithGeometry ${OpenCV_LIBS} ${CERES_LIBRARIES})

#file(GLOB src_testGetXYSignWithGeometry_files ${MODULE_SRC_ROOT}/testGetXYSignWithGeometry.cpp ${MODULE_SRC_ROOT}/getXYWithGeometry.cpp ${MODULE_SRC_ROOT}/bA.cpp)
#add_executable(testGetXYSignWithGeometry ${src_testGetXYSignWithGeometry_files})
#target_link_libraries(testGetXYSignWithGeometry ${OpenCV_LIBS} ${CERES_LIBRARIES})

#file(GLOB src_testWheelDR_files ${MODULE_SRC_ROOT}/testWheelDR.cpp ${MODULE_SRC_ROOT}/wheelDR.cpp)
#add_executable(testWheelDR ${src_testWheelDR_files})
#target_link_libraries(testWheelDR ${OpenCV_LIBS})

#file(GLOB src_testBA_files ${MODULE_SRC_ROOT}/testBA.cpp ${MODULE_SRC_ROOT}/bA.cpp)
#add_executable(testBA ${src_testBA_files})
#target_link_libraries(testBA ${CERES_LIBRARIES})

file(GLOB src_testGetCarTrackXY_files ${MODULE_SRC_ROOT}/testGetCarTrackXY.cpp ${MODULE_SRC_ROOT}/getXYWithGeometry.cpp  ${MODULE_SRC_ROOT}/wgs84_3degreeGauss.cpp ${MODULE_SRC_ROOT}/getCarInWitchLane.cpp ${MODULE_SRC_ROOT}/bA.cpp ${MODULE_SRC_ROOT}/CSVRow.cpp ${MODULE_SRC_ROOT}/ToolsBox.cpp)
add_executable(testGetCarTrackXY ${src_testGetCarTrackXY_files})
target_link_libraries(testGetCarTrackXY ${OpenCV_LIBS} ${CERES_LIBRARIES})


file(GLOB src_testGetCarInWitchLane_files ${MODULE_SRC_ROOT}/testGetCarInWitchLane.cpp ${MODULE_SRC_ROOT}/getCarInWitchLane.cpp)
add_executable(testGetCarInWitchLane ${src_testGetCarInWitchLane_files})
target_link_libraries(testGetCarInWitchLane ${OpenCV_LIBS})

file(GLOB src_testSortRoadData_files ${MODULE_SRC_ROOT}/testSortRoadData.cpp ${MODULE_SRC_ROOT}/getXYWithGeometry.cpp  ${MODULE_SRC_ROOT}/wgs84_3degreeGauss.cpp ${MODULE_SRC_ROOT}/getCarInWitchLane.cpp ${MODULE_SRC_ROOT}/bA.cpp ${MODULE_SRC_ROOT}/CSVRow.cpp ${MODULE_SRC_ROOT}/ToolsBox.cpp)
add_executable(testSortRoadData ${src_testSortRoadData_files})
target_link_libraries(testSortRoadData ${OpenCV_LIBS} ${CERES_LIBRARIES})
