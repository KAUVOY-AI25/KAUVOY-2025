# Install script for directory: /home/kauvoy/Race/src/ublox_f9p/ublox_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kauvoy/Race/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/msg" TYPE FILE FILES
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/Ack.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/AidALM.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/AidEPH.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/AidHUI.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgANT.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgCFG.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgDAT.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgDGNSS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgGNSS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgGNSS_Block.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgHNR.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgINF.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgINF_Block.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgMSG.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgNAV5.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgNAVX5.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgNMEA.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgNMEA6.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgNMEA7.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgPRT.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgRATE.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgRST.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgSBAS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgTMODE3.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/CfgUSB.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/EsfINS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/EsfMEAS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/EsfRAW.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/EsfRAW_Block.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/EsfSTATUS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/EsfSTATUS_Sens.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/HnrPVT.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/Inf.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/MgaGAL.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/MonGNSS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/MonHW.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/MonHW6.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/MonVER.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/MonVER_Extension.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavATT.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavCLOCK.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavDGPS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavDGPS_SV.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavDOP.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavPOSECEF.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavPOSLLH.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavPVT.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavPVT7.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavRELPOSNED.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSAT.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSAT_SV.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSBAS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSBAS_SV.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSOL.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSTATUS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSVIN.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSVINFO.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavSVINFO_SV.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavTIMEGPS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavTIMEUTC.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavVELECEF.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/NavVELNED.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmALM.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmEPH.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmRAW.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmRAWX.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmRAWX_Meas.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmRAW_SV.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmRTCM.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmSFRB.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmSFRBX.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmSVSI.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/RxmSVSI_SV.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/TimTM2.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/UpdSOS.msg"
    "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/msg/UpdSOS_Ack.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/cmake" TYPE FILE FILES "/home/kauvoy/Race/build/ublox_f9p/ublox_msgs/catkin_generated/installspace/ublox_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kauvoy/Race/devel/include/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kauvoy/Race/devel/share/roseus/ros/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kauvoy/Race/devel/share/common-lisp/ros/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kauvoy/Race/devel/share/gennodejs/ros/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/kauvoy/.conda/envs/main/bin/python3" -m compileall "/home/kauvoy/Race/devel/lib/python3/dist-packages/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/kauvoy/Race/devel/lib/python3/dist-packages/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kauvoy/Race/build/ublox_f9p/ublox_msgs/catkin_generated/installspace/ublox_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/cmake" TYPE FILE FILES "/home/kauvoy/Race/build/ublox_f9p/ublox_msgs/catkin_generated/installspace/ublox_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/cmake" TYPE FILE FILES
    "/home/kauvoy/Race/build/ublox_f9p/ublox_msgs/catkin_generated/installspace/ublox_msgsConfig.cmake"
    "/home/kauvoy/Race/build/ublox_f9p/ublox_msgs/catkin_generated/installspace/ublox_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs" TYPE FILE FILES "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/kauvoy/Race/devel/lib/libublox_msgs.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kauvoy/Race/src/ublox_f9p/ublox_msgs/include/" REGEX "/\\.svn$" EXCLUDE)
endif()

