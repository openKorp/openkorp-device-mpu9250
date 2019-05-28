# Copyright (C) 2018 Björnborg
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

cmake_minimum_required(VERSION 3.7)

project(openkorp-device-mpu9250)

################################################################################
# Defining the relevant versions of OpenDLV Standard Message Set and libcluon.
set(OPENDLV_STANDARD_MESSAGE_SET opendlv-standard-message-set.odvd)
set(OPENKORP_MESSAGE_SET openkorp-message-set.odvd)
set(CLUON_COMPLETE cluon-complete-v0.0.121.hpp)

################################################################################
# This project requires C++14 or newer.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Strip unneeded symbols from binaries.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
# Build a static binary.
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
# Add further warning levels.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -D_XOPEN_SOURCE=700 \
    -D_FORTIFY_SOURCE=2 \
    -O2 \
    -fstack-protector \
    -fomit-frame-pointer \
    -pipe \
    -pedantic -pedantic-errors \
    -Werror \
    -Weffc++ \
    -Wall -Wextra -Wshadow -Wdeprecated \
    -Wdiv-by-zero -Wfloat-equal -Wfloat-conversion -Wsign-compare -Wpointer-arith \
    -Wuninitialized -Wunreachable-code \
    -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-but-set-parameter -Wunused-but-set-variable \
    -Wunused-value -Wunused-variable -Wunused-result \
    -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn")
# Threads are necessary for linking the resulting binaries as UDPReceiver is running in parallel.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_program(CLUON_CMD cluon-msc)

################################################################################
# Find installed libcluon
if(CLUON_CMD)
    message(STATUS "Libcluon: found at " ${CLUON_CMD})
    add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-complete.hpp
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.hpp
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE})
else(CLUON_CMD)
################################################################################
# Extract cluon-msc from cluon-complete.hpp.
    message(STATUS "Libcluon: not found - generating cluon-msc from header file")
    add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-complete.hpp
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.hpp
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
        COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc ${CMAKE_BINARY_DIR}/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE})
    set(CLUON_CMD ${CMAKE_BINARY_DIR}/cluon-msc)
endif(CLUON_CMD)


################################################################################
# Generate opendlv-standard-message-set.hpp from ${OPENDLV_STANDARD_MESSAGE_SET} file.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CLUON_CMD} --cpp --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET} ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CLUON_CMD})

################################################################################
# Generate openkorp-message-set.hpp 
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/openkorp-message-set.hpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CLUON_CMD} --cpp --out=${CMAKE_BINARY_DIR}/openkorp-message-set.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENKORP_MESSAGE_SET}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENKORP_MESSAGE_SET} ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CLUON_CMD})



# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/librobotcontrol/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

################################################################################
# Find ncurse
# find_package( Curses REQUIRED )
# include_directories( ${CURSES_INCLUDE_DIRS} )

################################################################################
# Gather all object code first to avoid double compilation.


file(GLOB THIRDPARTY_SOURCE 
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/librobotcontrol/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/librobotcontrol/src/*/*.c
    )
# message(STATUS ${THIRDPARTY_SOURCE})

add_library(${PROJECT_NAME}-core OBJECT  
    ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp 
    ${CMAKE_BINARY_DIR}/openkorp-message-set.hpp
    ${THIRDPARTY_SOURCE}
    )
set(LIBRARIES Threads::Threads ${CURSES_LIBRARIES} ${Eigen3_LIBS})

################################################################################
# Create executable.
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

################################################################################
# Enable unit testing.
# enable_testing()
# add_executable(${PROJECT_NAME}-runner ${CMAKE_CURRENT_SOURCE_DIR}/test/tests-mpu9250.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
# target_link_libraries(${PROJECT_NAME}-runner ${LIBRARIES})
# add_test(NAME ${PROJECT_NAME}-runner COMMAND ${PROJECT_NAME}-runner)

################################################################################
# Install executable.
install(TARGETS ${PROJECT_NAME} DESTINATION bin COMPONENT ${PROJECT_NAME})
