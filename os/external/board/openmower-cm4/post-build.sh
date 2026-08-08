#!/bin/bash
# Runs after target finalization, before rootfs image creation.
set -eu

BOARD_DIR="$(cd "$(dirname "$0")" && pwd)"
OS_DIR="$(cd "$BOARD_DIR/../../.." && pwd)"   # .../os (external tree parent)

OPENMOWER_VERSION="${OPENMOWER_VERSION:-$(date -u +%Y%m%d%H%M%S)}"
OPENMOWER_GIT_REV="${OPENMOWER_GIT_REV:-unknown}"

# Own os-release: VERSION_ID is the numeric build version the OTA poller compares.
cat > "$TARGET_DIR/usr/lib/os-release" <<EOF
NAME="OpenMower OS"
ID=openmower-os
VERSION_ID=$OPENMOWER_VERSION
PRETTY_NAME="OpenMower OS $OPENMOWER_VERSION ($OPENMOWER_GIT_REV)"
VARIANT=cm4
EOF
ln -sf ../usr/lib/os-release "$TARGET_DIR/etc/os-release"

# MOTD: shown by dropbear/sshd on login (no PAM in this image -- both print
# /etc/motd directly by default, no pam_motd wiring needed). Baked at build
# time like os-release above, so it's static per image build -- includes the
# ROS/open_mower_ros software version too, pulled from the vendored tree's
# own version_info.env (written by open_mower_ros/docker/Dockerfile from
# `git describe`), not just this OS image's own version.
OM_VERSION_ENV="$TARGET_DIR/opt/openmower-root/opt/open_mower_ros/version_info.env"
if [ -f "$OM_VERSION_ENV" ]; then
    OM_SOFTWARE_VERSION="$(sed -n 's/^export OM_SOFTWARE_VERSION=//p' "$OM_VERSION_ENV" | head -n1)"
fi
OM_SOFTWARE_VERSION="${OM_SOFTWARE_VERSION:-unknown}"

cat > "$TARGET_DIR/etc/motd" <<EOF

  OpenMower OS
  ------------
  OS version    : $OPENMOWER_VERSION ($OPENMOWER_GIT_REV)
  ROS software  : $OM_SOFTWARE_VERSION

EOF

# /etc/issue: shown by getty on the serial console BEFORE login (unlike
# motd above, which only prints after auth succeeds) -- so a technician at
# the console can identify the running build without credentials.
# Overwrites whatever BR2_TARGET_GENERIC_ISSUE put here at
# rootfs-skeleton time (this script runs later); reuses the same version
# variables already computed above instead of a second mechanism.
echo "OpenMower OS $OPENMOWER_VERSION ($OPENMOWER_GIT_REV)" > "$TARGET_DIR/etc/issue"

# Make bash the interactive login shell for root (buildroot's skeleton
# hardcodes /bin/sh -- makeusers' syntax explicitly forbids overriding
# "root" via a users_table, so this is the only place to change it). Boot-
# critical scripts stay POSIX sh regardless (openmower-nspawn-start,
# openmower-shell, openmower-check-config) -- this is purely for interactive
# comfort, see rootfs-overlay/root/.bashrc for the Raspberry-Pi-OS-style
# prompt/aliases that go with it.
sed -i 's#^root:\(.*\):/bin/sh$#root:\1:/bin/bash#' "$TARGET_DIR/etc/passwd"

# RAUC keyring = dev signing cert (production: real CA cert here instead).
install -D -m 0644 "$OS_DIR/keys/dev-cert.pem" "$TARGET_DIR/etc/rauc/keyring.pem"

# Dropbear host keys must survive updates and reboots -> /data.
rm -rf "$TARGET_DIR/etc/dropbear"
ln -s /data/dropbear "$TARGET_DIR/etc/dropbear"

# Root's password persistence lives entirely at runtime (see
# openmower-persist-etc / openmower-etc-overlay.service), NOT here or in
# post-fakeroot.sh -- a build-time symlink over /etc/shadow looks tempting
# (same shape as dropbear's host keys above) but is actually unsafe: without
# BR2_PER_PACKAGE_DIRECTORIES (not enabled here), /etc/shadow is written
# exactly ONCE, by skeleton-init-common's own install step, into the shared
# $(TARGET_DIR) that Buildroot reuses incrementally across builds -- nothing
# ever recreates it. A symlink placed here (or in post-fakeroot.sh, or via
# the rootfs overlay -- tried all three, confirmed against a real build)
# breaks EVERY future rebuild permanently: Buildroot's own root-password
# baking (BR2_TARGET_GENERIC_ROOT_PASSWD, skeleton-init-common's
# SET_ROOT_PASSWD hook, part of TARGET_FINALIZE_HOOKS) runs at the very
# start of target-finalize, before any of our own hooks get a chance to
# fix it back, and fails with "shadow: No such file or directory" forever
# after.
#
# A single-file bind-mount at boot (tried second, also reverted) doesn't
# work either, for a completely different reason: `passwd` doesn't write
# /etc/shadow directly, it creates /etc/shadow+ next to it and renames it
# into place -- confirmed on real hardware ("can't create '/etc/shadow+':
# Read-only file system"). Bind-mounting one file doesn't make its
# ENCLOSING directory (/etc, on read-only squashfs) able to hold a new
# entry. openmower-persist-etc overlays the whole of /etc instead --
# verified against a real kernel (privileged container) before shipping
# it, not just reasoned about.

# Same reasoning for the auxiliary Docker stack (Mosquitto, OpenMowerApp,
# Dockge, ttyd): /opt/stacks and /opt/dockge are the paths Dockge's own
# compose.yaml hard-requires (host path == container path for its bind
# mount), but root is read-only squashfs, so both are symlinks onto /data
# (seeded on first boot by rootfs-overlay/etc/tmpfiles.d/docker-stacks.conf).
rm -rf "$TARGET_DIR/opt/stacks" "$TARGET_DIR/opt/dockge"
ln -s /data/stacks "$TARGET_DIR/opt/stacks"
ln -s /data/dockge "$TARGET_DIR/opt/dockge"

# Bake usercfg.txt.default's hardware config.txt addendum (UART, antenna,
# fan) onto config.txt itself, instead of keeping two copies of the same
# directives in the repo -- single source of truth is usercfg.txt.default
# (see its own comments), which also ships as-is to /data/boot/usercfg.txt
# as the live editable copy of those same defaults. Applied to both the
# real boot-partition config.txt (already installed by the rpi-firmware
# package, from config.txt.default) and the /data reference copy in
# TARGET_DIR (tmpfiles seeds /data/boot/config.txt from it, see
# rootfs-overlay/etc/tmpfiles.d/openmower.conf) so the two never drift
# apart. `include usercfg.txt` must come LAST, after the baked-in defaults
# it's appended alongside -- config.txt semantics are "later directives
# win", so an actual on-device edit to usercfg.txt (included at that same
# point once synced) overrides the baked default for the same setting
# rather than being overridden by it.
USERCFG_DEFAULT="$TARGET_DIR/etc/openmower/usercfg.txt.default"
for CONFIG_TXT in \
    "$BINARIES_DIR/rpi-firmware/config.txt" \
    "$TARGET_DIR/etc/openmower/config.txt.default"
do
    cat "$USERCFG_DEFAULT" >> "$CONFIG_TXT"
    echo "include usercfg.txt" >> "$CONFIG_TXT"
done
