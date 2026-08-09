################################################################################
#
# vcgencmd
#
################################################################################

# No upstream release tags -- pin to a specific commit for reproducibility.
VCGENCMD_VERSION = a54a0dbb2b8dcf9bafdddfc9a9374fb51d97e976
VCGENCMD_SITE = https://github.com/raspberrypi/userland.git
VCGENCMD_SITE_METHOD = git
VCGENCMD_LICENSE = BSD-3-Clause
VCGENCMD_LICENSE_FILES = LICENCE

# Only ever built for aarch64 here (BR2_cortex_a53, see openmower_defconfig).
# Upstream's own top-level CMakeLists already gates the GL/EGL/OpenMAX/MMAL/
# raspicam side of the tree behind `if(NOT ARM64)`/`if(BUILD_MMAL)` (the
# latter forced off by the former), leaving just vcos/vchiq_arm/vchostif
# (support libs, needed by the tools below) and host_applications/linux/apps'
# gencmd/tvservice/vcmailbox/dtoverlay/dtmerge -- no manual trimming needed.
#
# VMCS_INSTALL_PREFIX overrides makefiles/cmake/vmcs.cmake's own hardcoded
# /opt/vc default: it FORCE-sets CMAKE_INSTALL_PREFIX from this variable,
# ignoring whatever prefix Buildroot's cmake-package infra would otherwise
# pass on the command line -- has to be pre-set to override it.
#
# CMAKE_POLICY_VERSION_MINIMUM works around this long-untouched tree's own
# `cmake_minimum_required(VERSION 2.8)`, which current CMake (>= 4.0)
# refuses outright ("Compatibility with CMake < 3.5 has been removed").
VCGENCMD_CONF_OPTS = -DARM64=ON -DVMCS_INSTALL_PREFIX=/usr -DCMAKE_POLICY_VERSION_MINIMUM=3.5

$(eval $(cmake-package))
