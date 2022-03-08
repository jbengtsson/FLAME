add_custom_command(
  OUTPUT glps.par.c glps.par.h
  COMMAND ${YACC_EXECUTABLE}
  ARGS
    -o glps.par.c
    --defines=glps.par.h
    ${CMAKE_CURRENT_SOURCE_DIR}/glps.y
  MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/glps.y
)

add_custom_command(
  OUTPUT glps.tab.c glps.tab.h
  COMMAND ${LEX_EXECUTABLE}
  ARGS
    -o glps.tab.c
    --header-file=glps.tab.h
    ${CMAKE_CURRENT_SOURCE_DIR}/glps.l
  MAIN_DEPENDENCY ${CMAKE_CURRENT_SOURCE_DIR}/glps.l
)

configure_file(flame/version.h.in flame/version.h @ONLY)

set_source_files_properties(
    glps.par.c glps.par.h
    glps.tab.c glps.tab.h
    PROPERTIES
    GENERATED TRUE
)

set(flame_core_HEADERS
  flame/core/util.h
  flame/core/base.h
  flame/core/config.h
)

set(flame_bd_HEADERS
  flame/constants.h
  flame/state/vector.h
  flame/state/matrix.h
  flame/linear.h
  flame/moment.h
  flame/moment_sup.h
  flame/rf_cavity.h
  flame/chg_stripper.h
)

if(USE_HDF5)
  include_directories(${HDF5_CXX_INCLUDE_DIRS})
  set(flame_core_HEADERS ${flame_core_HEADERS}
    flame/core/h5loader.h
    flame/core/h5writer.h
  )
endif()

set(flame_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/flame/version.h
)

set(flame_files
  version.cpp
)

set(flame_core_files
  config.cpp
  base.cpp

  glps_parser.cpp glps_parser.h
  glps_ops.cpp
  glps.par.c glps.par.h
  glps.tab.c glps.tab.h

  util.cpp
)

set(flame_bd_files
  linear.cpp
  moment.cpp
  moment_sup.cpp
  rf_cavity.cpp
  chg_stripper.cpp
)

if(USE_HDF5)
  set(flame_core_files ${flame_core_files}
    h5loader.cpp
    h5writer.cpp
  )
endif()


if(MSYS)
   # msys / m$ requires the use of dllexport when using the library
   # but not during build
   add_definitions(-DFLAME_BUILD_LIB)
endif()
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(flame SHARED
  ${flame_files}
  ${flame_HEADERS}
 )

add_library(flame_core SHARED
  ${flame_core_files}
  ${flame_core_HEADERS}
)



add_library(flame::core ALIAS flame_core)
target_include_directories(flame_core
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)
target_link_libraries(flame_core
  ${Boost_THREAD_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
)

if(USE_HDF5)
  target_link_libraries(flame_core
    ${HDF5_LIBRARIES}
  )
endif()

set_target_properties(flame_core
  PROPERTIES
    SOVERSION 0
)


if(NOT MSYS)
    add_library(flame_bd SHARED
      ${flame_bd_files}
      ${flame_bd_HEADERS}
    )
    add_library(flame::bd ALIAS flame_bd)

    target_include_directories(flame_bd
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    )
    target_link_libraries(flame_bd
      ${Boost_FILESYSTEM_LIBRARY}
    )
    set_target_properties(flame_bd
      PROPERTIES
        SOVERSION 0
    )
endif()

set(FLAME_CMAKE_CONFIG_DIR "${CMAKE_INSTALL_DATADIR}/flame/cmake")
set(FLAME_CMAKE_LIB_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/flame")

# -- core --library part
install(FILES ${flame_core_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/flame/core)
install(TARGETS flame_core
        EXPORT flame_coreTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# generate and install export file
install(EXPORT flame_coreTargets
        FILE flamecoreTargets.cmake
        NAMESPACE flame::
        DESTINATION ${FLAME_CMAKE_LIB_DIR}
)
# -- end core --library part

# -- bd --library part
if(NOT MSYS)
    install(FILES ${flame_bd_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/flame/)
    install(TARGETS flame_bd
            EXPORT flamebdTargets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    # generate and install export file
    install(EXPORT flamebdTargets
            FILE flamebdTargets.cmake
            NAMESPACE flame::
            DESTINATION ${FLAME_CMAKE_LIB_DIR}
    )
endif()
# -- end bd --library part


configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/flameConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/flameConfig.cmake"
    INSTALL_DESTINATION ${FLAME_CMAKE_LIB_DIR}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/flameConfigVersion.cmake
    VERSION "${FLAME_VERSION}"
    COMPATIBILITY SameMajorVersion
)



# install(DIRECTORY include/flame DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${flame_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS flame
        EXPORT flameTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT flameTargets
    FILE flameTargets.cmake
    DESTINATION ${FLAME_CMAKE_LIB_DIR}
    NAMESPACE flame::
)

install(FILES
         "${CMAKE_CURRENT_BINARY_DIR}/flameConfig.cmake"
         "${CMAKE_CURRENT_BINARY_DIR}/flameConfigVersion.cmake"
	 DESTINATION ${FLAME_CMAKE_LIB_DIR}
)


add_executable(test_lex
  test_lex.cpp
)
target_link_libraries(test_lex
  flame_core
)

if(NOT MSYS)
    add_executable(test_jb_2
      test_jb_2.cpp
    )
    target_link_libraries(test_jb_2
      flame_core flame_bd
    )
endif()

add_executable(test_parse
  test_parse.cpp
)
target_link_libraries(test_parse
  flame_core
)

add_executable(test_config
  test_config.cpp
)
add_test(config test_config)
target_link_libraries(test_config
  flame_core
  ${Boost_PRG_EXEC_MONITOR_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

add_executable(test_util
  test_util.cpp
)
add_test(util test_util)
target_link_libraries(test_util
  flame_core
  ${Boost_PRG_EXEC_MONITOR_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

if(USE_HDF5)
  add_executable(h5_loader
    h5loadertest.cpp
  )
  target_link_libraries(h5_loader
    flame_core
  )
endif()

if(UNIX)

  add_test(recurse1
    /bin/sh ${CMAKE_CURRENT_SOURCE_DIR}/recurse-test.sh
      ./test_parse
      ${CMAKE_CURRENT_SOURCE_DIR}/data/tracy_1.lat
      ${CMAKE_CURRENT_SOURCE_DIR}/data/tracy_1_out.lat
  )

  add_test(recurse2
    /bin/sh ${CMAKE_CURRENT_SOURCE_DIR}/recurse-test.sh
      ./test_parse
      ${CMAKE_CURRENT_SOURCE_DIR}/data/tracy_2.lat
      ${CMAKE_CURRENT_SOURCE_DIR}/data/tracy_2_out.lat
  )

endif()
