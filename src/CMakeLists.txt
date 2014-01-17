cmake_minimum_required (VERSION 2.6)
project (OpenCV)
find_package(OpenCV REQUIRED)

add_executable(loadimage ch1/Ex1_loadimage.cpp)
target_link_libraries(loadimage ${OpenCV_LIBS})

add_executable(playavi ch1/Ex2_playavi.cpp)
target_link_libraries(playavi ${OpenCV_LIBS})

add_executable(trackbar ch1/Ex3_trackbar.cpp)
target_link_libraries(trackbar ${OpenCV_LIBS})

add_executable(smoothing ch1/Ex4_smoothing.cpp)
target_link_libraries(smoothing ${OpenCV_LIBS})

add_executable(trans ch1/Ex5_trans.cpp)
target_link_libraries(trans ${OpenCV_LIBS})