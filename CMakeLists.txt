cmake_minimum_required(VERSION 3.10)  
# Nama proyek 
project(BallMovement)  

# Mencari OpenCV yang terinstal di sistem 
find_package(OpenCV REQUIRED)  

# Menambahkan executable dari file main.cpp 
add_executable(BallMovement src/main.cpp)  

# Menautkan library OpenCV 
target_link_libraries(BallMovement ${OpenCV_LIBS})