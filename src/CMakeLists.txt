add_executable(CircleDetect CircleDetect.cpp)

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu)

target_link_libraries( CircleDetect "dc1394" ${OpenCV_LIBS} ) 

set(exec_targets CircleDetect)

install(TARGETS ${exec_targets} RUNTIME DESTINATION bin LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

