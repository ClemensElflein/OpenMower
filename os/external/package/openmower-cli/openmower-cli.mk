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

# openmower is a shiv-built Python zipapp -- self-executable (its own
# #!/usr/bin/env python3 shebang), no wrapper needed to run it. Real
# payload under /usr/lib, /usr/bin/openmower a plain symlink to it -- shiv
# unpacks its bundled site-packages into $HOME/.shiv on first run per
# distinct build (content-hash-keyed extraction dir, so an update's new
# zipapp gets its own fresh dir rather than colliding with an old one) and
# reuses the extraction on later runs; now that root's $HOME persists on
# /data (see post-build.sh), that default just works, no SHIV_ROOT
# override needed to redirect it away from the once-read-only /root.
define OPENMOWER_CLI_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/openmower $(TARGET_DIR)/usr/lib/openmower/openmower.pyz
	ln -sf ../lib/openmower/openmower.pyz $(TARGET_DIR)/usr/bin/openmower
endef

$(eval $(generic-package))
