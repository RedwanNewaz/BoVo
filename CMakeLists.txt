cmake_minimum_required(VERSION 3.22)
project(mpcHRVO)

set(CMAKE_CXX_STANDARD 17)
include(cmake/CPM.cmake)
include_directories(
        include
        /usr/include/python3.8
        )

link_directories(
        /usr/local/lib)
CPMAddPackage("gh:snape/HRVO#main")
add_executable(mpcHRVO main.cpp src/hrvo_agent.cpp include/hrvo_agent.h src/controller.cpp src/controller.cpp include/controller.h include/helper.h)


add_subdirectory(unittest)
add_subdirectory(ompl_integration)
add_subdirectory(bo_integration)

target_link_libraries(${PROJECT_NAME}
        HRVO
        -pthread
        -lpython3.8
        )
