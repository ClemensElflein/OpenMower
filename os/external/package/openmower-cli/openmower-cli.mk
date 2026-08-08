################################################################################
#
# openmower-cli
#
################################################################################

# Always the latest GitHub release, resolved and staged into .cache/openmower-cli
# by build.sh (see its own comment for why this isn't Buildroot's normal
# pinned-URL + static .hash download) -- SITE_METHOD=local like
# openmower-ros.mk/improv-ble.mk, neither of which goes
# through Buildroot's hash mechanism either.
OPENMOWER_CLI_VERSION = 1.0
OPENMOWER_CLI_SITE = $(BR2_EXTERNAL_OPENMOWER_PATH)/../.cache/openmower-cli
OPENMOWER_CLI_SITE_METHOD = local
# No LICENSE_FILES: the release zip ships only openmower + provenance.json,
# not the repo's LICENSE file -- MIT per the upstream repo's own LICENSE.
OPENMOWER_CLI_LICENSE = MIT

define OPENMOWER_CLI_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/openmower $(TARGET_DIR)/usr/lib/openmower/openmower.pyz
	$(INSTALL) -D -m 0755 $(OPENMOWER_CLI_PKGDIR)/files/openmower-wrapper \
		$(TARGET_DIR)/usr/bin/openmower
endef

$(eval $(generic-package))
