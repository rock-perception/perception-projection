rock_library(projection
    DEPS_PKGCONFIG opencv eigen3
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(projection_bin Main.cpp
    DEPS projection)

