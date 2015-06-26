rock_library(projection
    DEPS_PKGCONFIG opencv eigen3
    SOURCES Homography.cpp Omnidirectional.cpp
    HEADERS Homography.hpp StereoTriangulation.hpp Omnidirectional.hpp)
