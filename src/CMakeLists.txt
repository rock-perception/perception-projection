# In earlier versions opencv library was identifies 'opencv'
# without any version number in pkg-config
# Since the released used in Ubuntu 20.04 this changed and the 
# name now is 'opencv4'.
#
# The following lines identify the Ubuntu release version
# and select the library name accordingly.
#
find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(LSB_RELEASE_ID_SHORT STREQUAL 20.04)
    set(OPENCVLIBNAME opencv4)
else()
    set(OPENCVLIBNAME opencv)
endif()

rock_library(projection
    DEPS_PKGCONFIG ${OPENCVLIBNAME} eigen3
    SOURCES Homography.cpp Omnidirectional.cpp
    HEADERS Homography.hpp StereoTriangulation.hpp Omnidirectional.hpp OmnidirectionalConfig.hpp)

find_package(Boost COMPONENTS program_options filesystem system)
rock_executable(reproject
    SOURCES reproject.cpp    
    DEPS_PLAIN Boost_PROGRAM_OPTIONS Boost_FILESYSTEM Boost_SYSTEM
    DEPS projection)

