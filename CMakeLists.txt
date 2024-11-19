cmake_minimum_required(VERSION 3.27)
project(ft_kalman)

set(CMAKE_CXX_STANDARD 20)

# Define the list of source files
set(SOURCES
        srcs/Message.cpp
        srcs/Message.hpp
        srcs/Data.cpp
        srcs/Data.hpp
        srcs/Matrix.hpp
        srcs/KalmanFilter.hpp
        srcs/Timestamp.cpp
        srcs/Timestamp.hpp
        srcs/ft_split.cpp
        srcs/UdpConnection.cpp
)
add_executable(ft_kalman srcs/main.cpp ${SOURCES}
        )

Include(FetchContent)

FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.7.1 # or a later release
)

FetchContent_MakeAvailable(Catch2)

# Add the directory with KalmanFilter.hpp and Matrix.hpp to the include directories
include_directories(srcs)
add_executable(tests
        tests/mainTests.cpp
        tests/MatrixTests.cpp
        ${SOURCES}
        tests/KFFTGUChapter9.cpp)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

#add_executable(test tests/main.cpp ${SOURCES})
