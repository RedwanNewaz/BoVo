CPMAddPackage("gh:resibots/limbo#master")
CPMAddPackage("gh:stevengj/nlopt#master")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package( Boost COMPONENTS system filesystem REQUIRED)
find_package(OMPL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        ${Eigen_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}/../_deps/limbo-src/src
        ${ompl_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ../ompl_integration/src
)

add_executable(${PROJECT_NAME}_bo main_bo.cpp
        ../ompl_integration/src/polygonal_obstacles.cpp
        ../ompl_integration/src/ompl_planner.cpp
        )


target_link_libraries(${PROJECT_NAME}_bo
        Eigen3::Eigen
        nlopt
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${OMPL_LIBRARY}
        ${OpenCV_LIBS}
        )