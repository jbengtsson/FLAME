include(FindEPICSModule)

find_epics_module(
  NAME pvData
  IDFILES include/pv/pvData.h
  LIBS pvData
)
find_epics_module(
  NAME pvAccess
  IDFILES include/pv/pvAccess.h
  DBDS PVAServerRegister.dbd
  LIBS pvAccess pvAccessIOC
)
find_epics_module(
  NAME pva2pva
  IDFILES dbd/qsrv.dbd
  DBDS qsrv.dbd columnarinRecord.dbd
  LIBS qsrv
)

include_directories(
  ${EPICS_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)

list(APPEND flame_vaccel_SRCS
  flame.cpp
  flamedev.cpp
  flameelem.cpp
  flameset.cpp
  flamemeasure.cpp
  flameorbit.cpp
)
if(pvData_FOUND AND pvAccess_FOUND AND pva2pva_FOUND)
  list(APPEND flame_SRCS bpmtable.cpp)
  include_directories(
    ${pva2pva_INCLUDE_DIRS}
    ${pvData_INCLUDE_DIRS}
  )
endif()

add_library(flame_vaccel ${flame_vaccel_SRCS})
add_library(flame::vaccel ALIAS flame_vaccel)

target_link_libraries(flame_vaccel
  flame flame_core flame_bd
  ${Boost_REGEX_LIBRARY}
  ${EPICS_LIBRARIES}
)
if(pvData_FOUND AND pvAccess_FOUND AND pva2pva_FOUND)
  target_link_libraries(flame
    ${pva2pva_LIBRARIES} ${pvAccess_LIBRARIES} ${pvData_LIBRARIES}
  )
endif()

epics_install(
  PREFIX lib/epics/
  LIBS flame_vaccel flame_core flame_bd
  DBDS flameSupport.dbd
)

if(NEED_DEMOIOC)
  message(STATUS "Building flamedemo IOC")

  find_epics_module(
    NAME autosave
    IDFILES dbd/asSupport.dbd
    DBDS asSupport.dbd
    LIBS autosave
  )
  if(autosave_FOUND)
    message(STATUS "flamedemo will include autosave")
    list(APPEND EXTRA_DBDS ${autosave_DBDS})
    list(APPEND EXTRA_LIBS ${autosave_LIBRARIES})
  endif()

  find_epics_module(
    NAME iocstats
    IDFILES dbd/devIocStats.dbd
    DBDS devIocStats.dbd
    LIBS devIocStats
  )
  if(iocstats_FOUND)
    message(STATUS "flamedemo will include iocstats")
    list(APPEND EXTRA_DBDS ${iocstats_DBDS})
    list(APPEND EXTRA_LIBS ${iocstats_LIBRARIES})
  endif()

  find_epics_module(
    NAME caputlog
    IDFILES dbd/caPutLog.dbd
    DBDS caPutLog.dbd
    LIBS caPutLog
  )
  if(caputlog_FOUND)
    message(STATUS "flamedemo will include caputlog")
    list(APPEND EXTRA_DBDS ${caputlog_DBDS})
    list(APPEND EXTRA_LIBS ${caputlog_LIBRARIES})
  endif()

  if(pvData_FOUND AND pvAccess_FOUND AND pva2pva_FOUND)
    message(STATUS "flamedemo will include PVA integration")
    list(APPEND EXTRA_DBDS ${pvAccess_DBDS} ${pva2pva_DBDS})
    list(APPEND EXTRA_LIBS ${pva2pva_LIBRARIES} ${pvAccess_LIBRARIES} ${pvData_LIBRARIES})
  endif()

  epics_add_ioc(flamedemo
    DBDS flameSupport.dbd
         ${EXTRA_DBDS}
    LIBS flame
         ${EXTRA_LIBS}
    INSTALL_PREFIX lib/epics/
  )
endif()
