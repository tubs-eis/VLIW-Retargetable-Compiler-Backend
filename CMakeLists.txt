# Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
#                    Technische Universitaet Braunschweig, Germany
#                    www.tu-braunschweig.de/en/eis
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

# EIS open source compiler Build System
# This cmake project supports the following build types:
#   Release, Debug, Profile
#
# In order to build one of these types:
#   mkdir <Typename>; cd <Typename>
#   cmake -DCMAKE_BUILD_TYPE=<Typename>
#   make

cmake_minimum_required(VERSION 3.10)

project(eis-osc)

####
# Build type checking

if (NOT CMAKE_BUILD_TYPE)
    message("Using default build type 'Release'")
    SET(CMAKE_BUILD_TYPE RELEASE)
endif ()
string(TOLOWER ${CMAKE_BUILD_TYPE} lower_release_type)
if (${lower_release_type} MATCHES "release")
    message("Building Release version")
elseif (${lower_release_type} MATCHES debug)
    message("Building Debug version")
elseif (${lower_release_type} MATCHES profile)
    message("Building Profile version")
else ()
    message(FATAL_ERROR "Invalid build type '${CMAKE_BUILD_TYPE}'")
endif ()

####
# Build flags

set(CMAKE_CXX_FLAGS_RELEASE "-g2 -Wall -O3 -lrt -Werror=return-type")
set(CMAKE_CXX_FLAGS_DEBUG "-pedantic -ggdb -Wall -O0 -lrt -fno-omit-frame-pointer -fsanitize=address -static-libstdc++ -Werror=return-type")
set(CMAKE_CXX_FLAGS_PROFILE "-pedantic -ggdb -Wall -O3 -lrt -fno-omit-frame-pointer -DNDEBUG")

# to install: sudo apt-get install libboost-all-dev
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

#find_package(pybind11 REQUIRED)
#set(PYBIND11_CPP_STANDARD -std=c++1z)

execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
message(STATUS "Using G++ version " ${GCC_VERSION})
if (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)
    message(STATUS "GCC Version >= 4.9 Using c++14 standard")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++14")
    set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_PROFILE} -std=c++14")
else ()
    message(STATUS "GCC Version < 4.9 Using Boost Libraries")
    find_package(Boost REQUIRED COMPONENTS regex)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(eis_osc ${Boost_LIBRARIES})
endif ()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY build)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
set(BUILD_SHARED_LIBRARIES OFF)

####
# OpenMP Support

set(OpenMP_pthread_LIBRARY -lpthread)
find_package(OpenMP)
if (OPENMP_FOUND)
    message("Found OpenMP and will add flags")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_PROFILE} ${OpenMP_CXX_FLAGS}")
endif ()

####
# Source files
include_directories(header)


set(MAIN
        src/main.cpp )

set(SRC
        src/assembler.cpp
        src/functionalunit.cpp
        src/global.cpp
        src/label.cpp
        src/MI.cpp
        src/MO.cpp
        src/listsearch.cpp
        src/operation.cpp
        src/processor.cpp
        src/readFile.cpp
        src/register.cpp
        src/memory.cpp
        src/SLM.cpp
        src/vectorunit.cpp
        src/virtual_reg.cpp
        src/gen_scheduling.cpp
        src/optimizer.cpp

        src/opts.cpp
        src/moai_special.cpp
        src/gen_merging.cpp
        src/gen_register.cpp
        src/sched_stats.cpp
        src/utility.cpp
        src/portOptimalReg.cpp
        src/CompilerContext.cpp
        src/ga_stats.cpp
        src/rdg.cpp
        src/Origin.cpp

        header/EnergyRegisterAllocation/PopulationEnergyAllocation.cpp
        header/EnergyRegisterAllocation/EnergyChromosome.cpp



        header/EnergyRegisterAllocation/RegisterLivelihood.cpp

        header/EnergyRegisterAllocation/EnergyGene.cpp

        header/DoubleRegister.cpp

        header/ML_schedule.h

        src/Program.cpp)






set(EXPORT_SOURCES
        export/ScheduledMOCharacteristics.h)




set(POWER_INC
        powerEstimation/ModelImporter.h
        powerEstimation/TransitionEnergyEstimator.h
        powerEstimation/InstructionModel.h
        powerEstimation/CSVReader.h

        powerEstimation/Estimator.h

        header/EnergyRegisterAllocation/CombineReturn.h)

set(POWER_SRC
        powerEstimation/ModelImporter.cpp
        powerEstimation/TransitionEnergyEstimator.cpp
        powerEstimation/InstructionModel.cpp



        )



set(SRC_DEBIN debinary.c)


foreach (var ${SRC_DEBIN})
    set(SRC2_DEBIN ${SRC2_DEBIN} src/${var})
endforeach (var)

add_executable(eis_osc
        ${MAIN}
        ${POWER_INC}
        ${SRC}
        ${POWER_SRC}
        header/Program.h
        )


add_executable(debin ${SRC2_DEBIN})


add_custom_target(clean_all
        rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Removing cmake files" VERBATIM
        )

find_package(Doxygen)
if (DOXYGEN_FOUND)
    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
            )
endif (DOXYGEN_FOUND)
