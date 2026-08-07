################################################################################
#
# improv-serial
#
################################################################################

IMPROV_SERIAL_VERSION = 1.0
IMPROV_SERIAL_SITE = $(BR2_EXTERNAL_OPENMOWER_PATH)/package/improv-serial/files
IMPROV_SERIAL_SITE_METHOD = local
IMPROV_SERIAL_LICENSE = Proprietary
IMPROV_SERIAL_DEPENDENCIES = python3 python-serial

define IMPROV_SERIAL_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/improv-serial \
		$(TARGET_DIR)/usr/bin/improv-serial
	$(INSTALL) -D -m 0644 $(@D)/improv-serial.service \
		$(TARGET_DIR)/usr/lib/systemd/system/improv-serial.service
endef

$(eval $(generic-package))
