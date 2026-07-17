cmake_minimum_required(VERSION 3.15)
project(Devourer)

if(WIN32 AND DEFINED ENV{VCPKG_ROOT} AND NOT "$ENV{VCPKG_ROOT}" STREQUAL "")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

# The libusb RTL8812AU driver + Station L2 + crypto live in the WiFiDriver submodule.
add_subdirectory(extern/WiFiDriver)

# ---------------------------------------------------------------------------
#  devourer — layered ON TOP of WiFiDriver. Adds the IP layer (DHCP / static-IP
#  / ARP), the FPV + session glue (RTP relay, LQ feedback, RxDeframe demux,
#  ApfpvStation orchestration + auto-reconnect), and the AP-side WPA2
#  authenticator (WIP). The JNI lives with the app's Android build.
# ---------------------------------------------------------------------------
add_library(devourer
    src/ApfpvStation.cpp      src/ApfpvStation.h
    src/ApfpvDhcp.cpp         src/ApfpvDhcp.h
    src/RxDeframe.cpp         src/RxDeframe.h
    src/LqFeedback.cpp        src/LqFeedback.h
    src/Wpa2Authenticator.cpp src/Wpa2Authenticator.h)
target_compile_features(devourer PUBLIC cxx_std_20)
target_include_directories(devourer PUBLIC src)
# WHOLE_ARCHIVE: pull EVERY WiFiDriver .o (+ its static initializers) into dependents — splitting the
# driver into a separate static lib otherwise drops/reorders static-init .o and silently breaks TX
# (auth frames stop radiating). Equivalent to the pre-refactor single-lib link.
target_link_libraries(devourer PUBLIC "$<LINK_LIBRARY:WHOLE_ARCHIVE,WiFiDriver>")

# Host test/demo targets (Linux/Windows): the APFPV behavior suite + ACK-survival probe.
add_executable(ApfpvCompliance demo/apfpv_compliance.cpp)
target_link_libraries(ApfpvCompliance PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,devourer>" PkgConfig::libusb)

add_executable(ApfpvProbe demo/apfpv_probe.cpp)
target_link_libraries(ApfpvProbe PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,devourer>" PkgConfig::libusb)
if(WIN32)
    target_link_libraries(ApfpvProbe PRIVATE ws2_32)
endif()
