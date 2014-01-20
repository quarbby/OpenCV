cmake_minimum_required (VERSION 2.6)
project (OpenCV)
find_package(OpenCV REQUIRED)

#For ch2 exercises
add_executable(loadimage ch2/Ex1_loadimage.cpp)
target_link_libraries(loadimage ${OpenCV_LIBS})

add_executable(playavi ch2/Ex2_playavi.cpp)
target_link_libraries(playavi ${OpenCV_LIBS})

add_executable(trackbar ch2/Ex3_trackbar.cpp)
target_link_libraries(trackbar ${OpenCV_LIBS})

add_executable(smoothing ch2/Ex4_smoothing.cpp)
target_link_libraries(smoothing ${OpenCV_LIBS})

add_executable(trans ch2/Ex5_trans.cpp)
target_link_libraries(trans ${OpenCV_LIBS})

add_executable(canny ch2/Ex6_canny.cpp)
target_link_libraries(canny ${OpenCV_LIBS})

add_executable(capture ch2/Ex9_cameracapture.cpp)
target_link_libraries(capture ${OpenCV_LIBS})

add_executable(writeavi ch2/Ex10_writetoavi.cpp)
target_link_libraries(writeavi ${OpenCV_LIBS})

#For ch3 exercises
add_executable(addroi ch3/Ex3_12.cpp)
target_link_libraries(addroi ${OpenCV_LIBS})

add_executable(widthstep ch3/Ex3_13.cpp)
target_link_libraries(widthstep ${OpenCV_LIBS})

add_executable(alpha ch3/Ex3_14.cpp)
target_link_libraries(alpha ${OpenCV_LIBS})

#For ch4 exercises 
add_executable(mouse ch4/Ex4_1.cpp)
target_link_libraries(mouse ${OpenCV_LIBS})

add_executable(trackbar2 ch4/Ex4_2.cpp)
target_link_libraries(trackbar2 ${OpenCV_LIBS})
