
add_executable(poseController demo_controller.cpp ../src/controller.cpp)

target_link_libraries(poseController
        -pthread
        -lpython3.8)