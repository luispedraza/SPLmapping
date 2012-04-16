file(GLOB splmfiles "*.cpp")
rosbuild_add_library(SPLM ${splmfiles})
target_link_libraries(SPLM -lnewmat -lglut -lGL -lGLU)