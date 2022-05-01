
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
        ${ompl_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

set(SOURCE_FILES
        ../src/controller.cpp
        ../src/hrvo_agent.cpp
        src/ompl_planner.h
        src/ompl_planner.cpp
        src/polygonal_obstacles.h
        src/polygonal_obstacles.cpp
        src/mrm_planner.cpp src/mrm_planner.h)

add_executable(${PROJECT_NAME}_ompl main_ompl.cpp ${SOURCE_FILES} src/map_parser.cpp src/map_parser.h)
find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME}_ompl
        HRVO
        ${OMPL_LIBRARY}
        ${OpenCV_LIBS}
        -pthread
        -lpython3.8
        Eigen3::Eigen
        )
