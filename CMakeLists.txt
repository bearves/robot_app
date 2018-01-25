cmake_minimum_required(VERSION 2.8)
set( CMAKE_EXPORT_COMPILE_COMMANDS 1)
# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(PREPEND_CPP var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.cpp")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_CPP)

FUNCTION(PREPEND_H var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}.h")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND_H)

project(robot_app)

if(UNIX)
    #set(CMAKE_C_COMPILER "/usr/bin/gcc-4.8")
    #set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")
    set(CMAKE_C_COMPILER "/usr/bin/clang")
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set debug and release options
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CONFIGURATION_TYPES Debug Release)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
endif(UNIX)

# set definitions to disable some warning
if(WIN32)
add_definitions(-DWIN32 -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS)
endif(WIN32)
if(UNIX)
add_definitions(-DUNIX)
endif(UNIX)

# set include and lib folders
if(UNIX)
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread)
set(XENOMAI_INCL_DIR "/usr/Aris_Dependent/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/Aris_Dependent/xenomai/lib")
set(XENOMAI_LINK_LIB native rtdm xenomai)
set(ETHERLAB_INCL_DIR "/usr/Aris_Dependent/etherlab/include")
set(ETHERLAB_LINK_DIR "/usr/Aris_Dependent/etherlab/lib")
set(ETHERLAB_LINK_LIB ethercat_rtdm rt)
set(ARIS_INCL_DIR "/usr/aris/include")
set(ARIS_LINK_DIR "/usr/aris/lib")
set(ARIS_LINK_LIB aris_server aris_model aris_control aris_core)
set(NRT_ARIS_LINK_LIB  aris_core)

set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR})
set(ALL_LINK_DIR ${ARIS_LINK_DIR} ${ETHERLAB_LINK_DIR} ${XENOMAI_LINK_DIR})
set(ALL_LINK_LIB ${ARIS_LINK_LIB} ${ETHERLAB_LINK_LIB} ${XENOMAI_LINK_LIB} ${SYSTEM_LINK_LIB})
set(NRT_LINK_LIB ${NRT_ARIS_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(UNIX)
if(WIN32)
set(GENERAL_INCL_DIR "C:/Aris_Dependent")
set(SYSTEM_LINK_LIB ws2_32 setupapi)
set(XSENS_INCL_DIR "C:/Aris_Dependent/xsens/include")
set(XSENS_LINK_DIR "C:/Aris_Dependent/xsens/lib")
set(XSENS_LINK_LIB xdeviceclass)
set(ARIS_INCL_DIR "C:/Aris/include")
set(ARIS_LINK_DIR "C:/Aris/lib")
set(ARIS_LINK_LIB aris_server aris_sensor aris_model aris_control aris_core)

set(ALL_INCL_DIR  ${ARIS_INCL_DIR} ${XSENS_INCL_DIR} ${ETHERLAB_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR} ${PROJ_INCL_DIR})
set(ALL_LINK_DIR  ${ARIS_LINK_DIR} ${XSENS_LINK_DIR} )
set(ALL_LINK_LIB  ${ARIS_LINK_LIB} ${XSENS_LINK_LIB} ${SYSTEM_LINK_LIB})
endif(WIN32)

include_directories(${ALL_INCL_DIR})
link_directories(${ALL_LINK_DIR})

################################### build server ####################################
set(SOURCE_FILES main.cpp GaitPlanner/RecoverPlanner.cpp GaitPlanner/WalkPlanner.cpp GaitPlanner/MotorSelector.cpp GaitPlanner/Kinematics/LegKinematics.cpp)
PREPEND(FULL_SRC Server ${SOURCE_FILES})
add_executable(Server ${FULL_SRC})
target_link_libraries(Server ${ALL_LINK_LIB})

################################### build client ####################################
set(SOURCE_FILES client.cpp)
PREPEND(FULL_SRC Client ${SOURCE_FILES})
add_executable(rbt ${FULL_SRC})
target_link_libraries(rbt ${NRT_LINK_LIB})

################################### build tests ####################################
set(SOURCE_FILES GaitPlanner/walk2.cpp)
PREPEND(FULL_SRC Server ${SOURCE_FILES})
add_executable(walk2 ${FULL_SRC})
target_link_libraries(walk2 ${NRT_LINK_LIB})

################################### install ####################################
if(UNIX)
	set(CMAKE_INSTALL_PREFIX "/usr/Robots/CMakeDemo/")
endif(UNIX)
if(WIN32)
	set(CMAKE_INSTALL_PREFIX "C:/Robots/CMakeDemo/")
endif(WIN32)

install(TARGETS Server rbt RUNTIME DESTINATION bin)


