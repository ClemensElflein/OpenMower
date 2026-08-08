################################################################################
#
# improv-ble
#
################################################################################

IMPROV_BLE_VERSION = 1.0
IMPROV_BLE_SITE = $(BR2_EXTERNAL_OPENMOWER_PATH)/package/improv-ble/files
IMPROV_BLE_SITE_METHOD = local
IMPROV_BLE_LICENSE = Proprietary
IMPROV_BLE_DEPENDENCIES = python3 python-dbus-fast

define IMPROV_BLE_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/improv-ble \
		$(TARGET_DIR)/usr/bin/improv-ble
	$(INSTALL) -D -m 0644 $(@D)/improv-ble.service \
		$(TARGET_DIR)/usr/lib/systemd/system/improv-ble.service
endef

$(eval $(generic-package))
