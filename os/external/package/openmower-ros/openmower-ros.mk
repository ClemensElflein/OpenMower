################################################################################
#
# openmower-ros
#
################################################################################

OPENMOWER_ROS_VERSION = 1.0
OPENMOWER_ROS_SITE = $(BR2_EXTERNAL_OPENMOWER_PATH)/../.cache/openmower-rootfs
OPENMOWER_ROS_SITE_METHOD = local
OPENMOWER_ROS_LICENSE = Proprietary

define OPENMOWER_ROS_INSTALL_TARGET_CMDS
	rm -rf $(TARGET_DIR)/opt/openmower-root
	mkdir -p $(TARGET_DIR)/opt
	cp -al $(@D)/. $(TARGET_DIR)/opt/openmower-root/
	rm -f $(TARGET_DIR)/opt/openmower-root/.br-content-hash
	# Defensive pre-create of the resolv.conf mount point nspawn's
	# --resolv-conf=bind-host binds onto. /opt/openmower-root is read-only
	# squashfs at runtime; nspawn is supposed to handle a missing
	# destination itself, but cheap enough to not depend on that.
	rm -f $(TARGET_DIR)/opt/openmower-root/etc/resolv.conf
	touch $(TARGET_DIR)/opt/openmower-root/etc/resolv.conf
	# Same deal for /etc/hosts, which openmower-nspawn-start/openmower-shell
	# bind the host's own copy onto (--bind-ro=/etc/hosts) -- Docker
	# deliberately excludes /etc/hosts (like resolv.conf/hostname) from
	# committed image layers, injecting it fresh per-container at runtime
	# instead, so the vendored tree never had one to begin with. Without
	# this, roslaunch/roscore can't even resolve "localhost" (observed:
	# "RLException: cannot resolve host address for machine [localhost]").
	rm -f $(TARGET_DIR)/opt/openmower-root/etc/hosts
	touch $(TARGET_DIR)/opt/openmower-root/etc/hosts
	# openmower-start runs INSIDE the container via nspawn's trailing
	# command argument -- lives inside /opt/openmower-root.
	$(INSTALL) -D -m 0755 $(OPENMOWER_ROS_PKGDIR)/files/openmower-start \
		$(TARGET_DIR)/opt/openmower-root/usr/local/bin/openmower-start
	# openmower-check-config, openmower-nspawn-start and openmower-shell all
	# run on the HOST (the first as openmower.service's ExecCondition=,
	# before nspawn even starts; the second is openmower.service's actual
	# ExecStart=, invokes nspawn itself; the third invokes nspawn for an
	# interactive debug shell) -- all three live in the base rootfs, not
	# the vendored tree.
	$(INSTALL) -D -m 0755 $(OPENMOWER_ROS_PKGDIR)/files/openmower-check-config \
		$(TARGET_DIR)/usr/bin/openmower-check-config
	$(INSTALL) -D -m 0755 $(OPENMOWER_ROS_PKGDIR)/files/openmower-nspawn-start \
		$(TARGET_DIR)/usr/bin/openmower-nspawn-start
	$(INSTALL) -D -m 0755 $(OPENMOWER_ROS_PKGDIR)/files/openmower-shell \
		$(TARGET_DIR)/usr/bin/openmower-shell
	$(INSTALL) -D -m 0644 $(OPENMOWER_ROS_PKGDIR)/files/openmower.service \
		$(TARGET_DIR)/usr/lib/systemd/system/openmower.service
	# openmower.conf.example ships in two places: the host copy is what
	# openmower-check-config's EnvironmentFile= reads (ExecCondition= is a
	# host-side process); the copy inside /opt/openmower-root is what
	# openmower-start sources directly once it's running in the container
	# (see openmower-start -- doesn't rely on nspawn env-passthrough).
	$(INSTALL) -D -m 0644 $(OPENMOWER_ROS_PKGDIR)/files/openmower.conf.example \
		$(TARGET_DIR)/etc/openmower/openmower.conf
	$(INSTALL) -D -m 0644 $(OPENMOWER_ROS_PKGDIR)/files/openmower.conf.example \
		$(TARGET_DIR)/opt/openmower-root/etc/openmower/openmower.conf
	# Default mower_params.yaml (safe generic values, enable_mower: false) --
	# copied to /data/openmower/params/ on first boot if missing (see
	# rootfs-overlay/etc/tmpfiles.d/openmower.conf's
	# "C" line) so openmower-check-config doesn't block startup on a
	# freshly-flashed device, at the cost of the mower being unconfigured
	# (enable_mower: false) until someone edits it for their actual robot.
	$(INSTALL) -D -m 0644 $(OPENMOWER_ROS_PKGDIR)/files/mower_params.yaml.default \
		$(TARGET_DIR)/etc/openmower/mower_params.yaml.default
endef

$(eval $(generic-package))
