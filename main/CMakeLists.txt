set(COMPONENT_SRCS
    "main.cpp"
    "WiFi.cpp"
    # "motors.cpp"
    "Flyer.cpp"
    "FlyerCalibration.cpp"
    "Waveshare10DOF.cpp"
    "Remote.cpp"
    "Sonar.cpp"
    "WiFi.cpp"
)

register_component()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

target_link_libraries(${COMPONENT_TARGET} icarus Eigen3::Eigen)

target_include_directories(${COMPONENT_TARGET} PUBLIC ${Boost_INCLUDE_DIRS})
