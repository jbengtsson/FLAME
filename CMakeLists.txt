cmake_minimum_required(VERSION 3.0.0)

project("flame" LANGUAGES C CXX)

option(USE_WERROR "Treat warnings as errors" OFF)
option(USE_PYTHON "Build python support" OFF)
option(USE_HDF5 "HDF5 output support" OFF)
option(NEED_PYTHON "Require python support" OFF)
option(NEED_EPICS  "Require EPICS support" OFF)
option(NEED_DEMOIOC  "build flamedemo IOC executable" OFF)

message(STATUS "Option PYTHON ${USE_PYTHON}")
message(STATUS "Option HDF5 ${USE_HDF5}")


set(FLAME_VERSION 1.8.6)
if(FLAME_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)")
  set(FLAME_VERSION_MAJOR "${CMAKE_MATCH_1}")
  set(FLAME_VERSION_MINOR "${CMAKE_MATCH_2}")
  set(FLAME_VERSION_PATCH "${CMAKE_MATCH_3}")
  math(EXPR FLAME_VERSION_DECIMAL
    "${FLAME_VERSION_MAJOR}*1000000 + ${FLAME_VERSION_MINOR}*1000 + ${FLAME_VERSION_PATCH}")
else()
  message(FATAL_ERROR "Failed to parse FLAME_VERSION='${FLAME_VERSION}'")
endif()


# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE) # default to debug build
  message(STATUS "Default to DEBUG build")
endif()

if(CMAKE_COMPILER_IS_GNUCC)
  # Turn on extra GCC warnings
  add_definitions(-pedantic -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
  add_definitions(-g -ggdb)
  add_definitions(
  	-Wcast-align
  	-Wcast-qual
  ##	-Wctor-dtor-privacy
  ##	-Wdisabled-optimization
  ##	-Wformat=2
  ##	-Winit-self
  ##	-Wlogical-op
  ##	-Wnoexcept
  ##	# -Wold-style-cast
  	-Woverloaded-virtual
  	-Wredundant-decls
  ##	# -Wshadow
  ##	-Wsign-promo
  ##	-Wstrict-null-sentinel
  ##	-Wstrict-overflow=5
  ##	-Wswitch-default
  ##	-Wundef
  ##	-Wno-unused
  ## 	-Wmissing-declarations
  ## 	#-Weffc++
  )

  # not compatible with boost
  # add_definitions(-D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC -g -ggdb -pedantic -Wall)
  if(USE_WERROR)
    add_definitions(-Werror)
  endif()
endif()
message(STATUS "Build type ${CMAKE_BUILD_TYPE}")

#Default path for rf cavity data
if(DEF_PATH)
  add_definitions(-DDEFPATH=\"${DEF_PATH}\")
  message(STATUS "Set default rf cavity data path: ${DEF_PATH}")
endif()

# see if git submodule is actually checked out
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake4epics/cmake/Modules/FindEPICS.cmake)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake4epics/cmake/Modules)
  set(TRY_EPICS 1)
else()
   message(STATUS "Skip EPICS check.  Checkout cmake4epics submodule")
endif()

find_package(Boost 1.52.0 REQUIRED COMPONENTS
  system
  thread
  filesystem
  regex
  program_options
  prg_exec_monitor unit_test_framework
)

# respects -DHDF5_USE_STATIC_LIBRARIES=YES
# and environment variable HDF5_ROOT
if(USE_HDF5)
  find_package(HDF5 REQUIRED COMPONENTS C CXX)
  add_definitions(-DUSE_HDF5)
endif()

find_package(FLEX)
find_package(BISON 3)

find_program(CPPCHECK_EXECUTABLE cppcheck
  DOC "cppcheck executable"
)
message(STATUS "cppcheck ${CPPCHECK_EXECUTABLE}")

message(STATUS "Boost libs in ${Boost_LIBRARY_DIRS}")

find_package(PythonInterp 3)

if(PYTHONINTERP_FOUND)
  # Introspect this interpreter
  execute_process(COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/pyconfig.py ${CMAKE_CURRENT_BINARY_DIR}/pyconfig.cmake
    RESULT_VARIABLE _PY_CONFIG
  )
  if(_PY_CONFIG)
    message(STATUS "Failed to fetch build information for ${PYTHON_EXECUTABLE}")
  else()
    include(${CMAKE_CURRENT_BINARY_DIR}/pyconfig.cmake)
    message(STATUS "Python version ${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}.${Python_VERSION_PATCH}")
    message(STATUS " headers ${Python_INCLUDE_DIRS}")
    message(STATUS " numpy ${Python_NUMPY_FOUND}")
    #message(STATUS " library ${Python_LIBRARY_DIRS}") # not used
  endif()
endif()

if(Python_FOUND)
  # see if nosetests test runner is available
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m nose.core -V
    OUTPUT_QUIET
    ERROR_QUIET
    RESULT_VARIABLE _HAVE_NOSE
  )
  if(_HAVE_NOSE)
    if(NEED_PYTHON)
      message(SEND_ERROR "Unable to build Python support")
    else()
      message(STATUS "Can't find nosetests, not running tests")
    endif()
    set(HAVE_NOSE NO)
  else()
    set(HAVE_NOSE YES)
  endif()
endif()

find_package(Doxygen)

if(TRY_EPICS)
  find_package(EPICS COMPONENTS IOC)
  find_package(EPICSTools)
endif()

enable_testing()

add_subdirectory(src)
add_subdirectory(examples)

if(USE_PYTHON AND Python_NUMPY_FOUND)
  message(STATUS "Building python support")
  add_subdirectory(python/flame)
elseif(NEED_PYTHON)
  message(SEND_ERROR "Unable to build Python support")
else()
  message(STATUS "Omitting python support")
endif()

add_subdirectory(tools)

if(EPICS_FOUND AND EPICSTOOLS_FOUND)
  message(STATUS "Building virtual accelerator IOC")
  add_subdirectory(vaccel)
elseif(NEED_EPICS)
  message(SEND_ERROR "Unable to build virtual accelerator IOC")
else()
  message(STATUS "Omit virtual accelerator IOC")
endif()

if(DOXYGEN_FOUND)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/documentation)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/documentation/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/documentation/Doxyfile
    @ONLY
  )
  add_custom_target(doc
    COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/documentation
  )
  if(UNIX)
    add_custom_target(doc-commit
      DEPENDS doc
      COMMAND find . -type f -print0 | xargs -0 ${CMAKE_CURRENT_SOURCE_DIR}/documentation/commit-gh.sh ${CMAKE_CURRENT_SOURCE_DIR}/.git
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/documentation/html
    )
  endif()
  message(STATUS "Will generate documentation")
else()
  message(STATUS "Skipping documentation generation (missing doxygen)")
endif()

if(CPPCHECK_EXECUTABLE)
  # Add a target with --enable=all for maximum noise for developers
  add_custom_target(cppcheck
    COMMAND ${CPPCHECK_EXECUTABLE} --enable=all --error-exitcode=1
                -I ${CMAKE_CURRENT_SOURCE_DIR}/src
                -I ${CMAKE_CURRENT_BINARY_DIR}/src
                -I ${CMAKE_CURRENT_BINARY_DIR}/python/flame
                ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
                ${CMAKE_CURRENT_BINARY_DIR}/src/*.c
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/cli.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/python/flame/mod*.cpp
    COMMENT "Running cppcheck static analysis"
  )
  # add a test with fewer checks (those which actually pass)
  add_test(NAME run_cppcheck
    COMMAND ${CPPCHECK_EXECUTABLE} --enable=performance,portability --error-exitcode=1
                -I ${CMAKE_CURRENT_SOURCE_DIR}/src
                -I ${CMAKE_CURRENT_BINARY_DIR}/src
                -I ${CMAKE_CURRENT_BINARY_DIR}/python/flame
                ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
                ${CMAKE_CURRENT_BINARY_DIR}/src/*.c
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/cli.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/python/flame/mod*.cpp
  )
endif()
