rock_library(projection
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(projection_bin Main.cpp
    DEPS projection)

