rock_opencv_autodetect(OPENCVLIBNAME)

rock_library(projection
    DEPS_PKGCONFIG ${OPENCVLIBNAME} eigen3
    SOURCES Homography.cpp Omnidirectional.cpp
    HEADERS Homography.hpp StereoTriangulation.hpp Omnidirectional.hpp OmnidirectionalConfig.hpp)

find_package(Boost COMPONENTS program_options filesystem system)
rock_executable(reproject
    SOURCES reproject.cpp    
    DEPS_PLAIN Boost_PROGRAM_OPTIONS Boost_FILESYSTEM Boost_SYSTEM
    DEPS projection)

