cmake_minimum_required(VERSION 3.27)
project(ft_kalman)

set(CMAKE_CXX_STANDARD 17)

add_executable(ft_kalman srcs/main.cpp
        srcs/Message.cpp
        srcs/Message.hpp
        srcs/Data.cpp
        srcs/Data.hpp
        srcs/Matrix.hpp
        srcs/KalmanFilter.cpp
        srcs/KalmanFilter.hpp)
