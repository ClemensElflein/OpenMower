#!/bin/bash
# Assemble sdcard.img (A/B layout) and the signed RAUC update bundle.
set -eu

BOARD_DIR="$(cd "$(dirname "$0")" && pwd)"
OS_DIR="$(cd "$BOARD_DIR/../../.." && pwd)"

OPENMOWER_VERSION="${OPENMOWER_VERSION:-$(date -u +%Y%m%d%H%M%S)}"
# Set via BR2_ROOTFS_POST_IMAGE_SCRIPT_ARGS in openmower_dev_defconfig
# only ("$1" is BINARIES_DIR, unused positionally below -- we read it from
# the env var Buildroot also exports).
OPENMOWER_VARIANT="${2:-prod}"

# --- Stage both boards' kernels/DTBs -----------------------------------------
# No kernel built in this tree (see post-build.sh) -- both come from the
# kernel-cm4/kernel-cm5 satellite builds. Filenames matter here: kernel8.img/
# kernel_2712.img and each board's *.dtb must exactly match what
# config.txt.default's per-board [cm4]/[pi4]/[cm5] sections name explicitly
# (kernel=/device_tree=) -- NOT relying on Pi firmware auto-detecting by
# omission. Auto-detect is real, documented firmware behavior, but depends
# on the specific pinned firmware blob version (rpi-firmware.mk) actually
# supporting it -- an earlier version of this file left kernel=/device_tree=
# unset entirely and that shipped a CM4 build that didn't boot on real
# hardware. Explicit per-board directives sidestep the question rather than
# depend on it.
KERNEL_CM4_DIR="$OS_DIR/output-kernel-cm4"
KERNEL_CM5_DIR="$OS_DIR/output-kernel-cm5"
install -m 0644 "$KERNEL_CM4_DIR/images/Image" "$BINARIES_DIR/rpi-firmware/kernel8.img"
install -m 0644 "$KERNEL_CM5_DIR/images/Image" "$BINARIES_DIR/rpi-firmware/kernel_2712.img"
cp -f "$KERNEL_CM4_DIR/images/"*.dtb "$KERNEL_CM5_DIR/images/"*.dtb "$BINARIES_DIR/rpi-firmware/"

# --- Build boot filesystem images (differ only in cmdline.txt) ---------------
# Contents: Pi firmware (incl. our config.txt), both kernels, both boards'
# DTBs -- everything now lives under rpi-firmware/ (post-build.sh purged the
# stale top-level DTBs rpi-firmware's own INSTALL_DTBS produced; this build
# has no kernel package of its own, so there's no separate top-level
# Image/*.dtb to glob the way a combined single-kernel build would have).
BOOT_FILES=("$BINARIES_DIR"/rpi-firmware/*)

# Dev image only: show a console on the default UART hardware pins (GPIO14/
# 15, the mini-UART -- ttyS0/serial0, already clocked correctly via config.
# txt's enable_uart=1) in addition to the USB gadget one. Resolved into
# BINARIES_DIR rather than editing cmdline-{a,b}.txt in place: those are
# shared with prod, where those same pins carry real mower hardware
# traffic (LL board or similar), not a debug terminal -- see config.txt's
# own enable_uart=1 comment for that reasoning.
CMDLINE_A="$BOARD_DIR/cmdline-a.txt"
CMDLINE_B="$BOARD_DIR/cmdline-b.txt"
if [ "$OPENMOWER_VARIANT" = "dev" ]; then
    CMDLINE_A="$BINARIES_DIR/cmdline-a.txt"
    CMDLINE_B="$BINARIES_DIR/cmdline-b.txt"
    sed 's/$/ console=ttyS0,115200/' "$BOARD_DIR/cmdline-a.txt" >"$CMDLINE_A"
    sed 's/$/ console=ttyS0,115200/' "$BOARD_DIR/cmdline-b.txt" >"$CMDLINE_B"
fi

make_boot_vfat() {
    local out="$1" cmdline="$2"
    rm -f "$out"
    # Bumped from 128M: this partition now carries TWO kernels + both
    # boards' DTBs instead of one (see the kernel-cm4/kernel-cm5 staging
    # above). Starting budget, not measured yet -- re-tune (`mdir -i
    # boot-a.vfat -/` or similar) after the first real build.
    mkfs.vfat -C -n "$3" "$out" $((192 * 1024))   # size in 1K blocks
    mcopy -i "$out" -s -Q "${BOOT_FILES[@]}" ::/
    mcopy -i "$out" -o -Q "$cmdline" ::/cmdline.txt
    # config.txt's `include usercfg.txt` tolerates a missing target, but bake
    # the shipped default in anyway rather than lean on that -- real content
    # gets synced on top of this from /data on first boot (see
    # openmower-sync-bootcfg) and on every OTA install (see rauc-hook.sh).
    mcopy -i "$out" -o -Q "$TARGET_DIR/etc/openmower/usercfg.txt.default" ::/usercfg.txt
}

make_boot_vfat "$BINARIES_DIR/boot-a.vfat" "$CMDLINE_A" BOOT-A
make_boot_vfat "$BINARIES_DIR/boot-b.vfat" "$CMDLINE_B" BOOT-B

cp -f "$BOARD_DIR/autoboot.txt" "$BINARIES_DIR/autoboot.txt"

# --- Assemble the disk image -------------------------------------------------
# Skippable (OPENMOWER_BUILD_SDCARD=0): the RAUC bundle below needs only
# boot-a.vfat + rootfs.squashfs, both already built above/by Buildroot
# itself -- genimage additionally allocates+writes rootfs-b (3584M),
# data.ext4 (4096M) and config.vfat, ~8GB of pure image-assembly work
# nothing in the bundle depends on. CI skips it for ordinary PR/branch
# builds (bundle-only is enough to validate the build); tagged releases
# still get the full flashable sdcard.img, default on for everyone else
# (a bare `./build.sh image` / `make image` is unaffected).
if [ "${OPENMOWER_BUILD_SDCARD:-1}" = "1" ]; then
    GENIMAGE_TMP="$BUILD_DIR/genimage.tmp"
    ROOTPATH_TMP="$(mktemp -d)"
    trap 'rm -rf "$ROOTPATH_TMP"' EXIT
    rm -rf "$GENIMAGE_TMP"

    genimage \
        --rootpath "$ROOTPATH_TMP" \
        --tmppath "$GENIMAGE_TMP" \
        --inputpath "$BINARIES_DIR" \
        --outputpath "$BINARIES_DIR" \
        --config "$BOARD_DIR/genimage.cfg"

    # Release asset scripts/migrate-to-openmower.sh downloads directly over
    # HTTPS (a GitHub Releases upload, see that script's own header -- no
    # manifest, no update server of our own). gzipped: sdcard.img's 4096M
    # data partition is freshly formatted and empty (see genimage.cfg), so
    # it's mostly sparse zero bytes on disk -- gzip shrinks the image back
    # down to roughly its real unique content instead of shipping the full
    # nominal size. Sidecar is a bare hex digest (not `sha256sum` output
    # format) so the migrate script can use it directly without parsing.
    IMG_GZ="$BINARIES_DIR/openmower-$OPENMOWER_VERSION.img.gz"
    rm -f "$BINARIES_DIR"/openmower-*.img.gz "$BINARIES_DIR"/openmower-*.img.gz.sha256
    gzip -c "$BINARIES_DIR/sdcard.img" > "$IMG_GZ"
    sha256sum "$IMG_GZ" | cut -d' ' -f1 > "$IMG_GZ.sha256"
fi

# --- Build the RAUC bundle ---------------------------------------------------
# The bundle carries the A-variant boot image; a post-install hook rewrites
# cmdline.txt when the target is slot B.
BUNDLE_DIR="$(mktemp -d)"
trap 'rm -rf "${ROOTPATH_TMP:-}" "$BUNDLE_DIR"' EXIT

cp "$BINARIES_DIR/boot-a.vfat" "$BUNDLE_DIR/boot.vfat"
cp "$BINARIES_DIR/rootfs.squashfs" "$BUNDLE_DIR/rootfs.squashfs"
cp "$BOARD_DIR/rauc-hook.sh" "$BUNDLE_DIR/hook.sh"
chmod +x "$BUNDLE_DIR/hook.sh"

cat > "$BUNDLE_DIR/manifest.raucm" <<EOF
[update]
# No per-board suffix -- one bundle now legitimately installs on either
# CM4 or CM5 (see openmower-detect-hardware), matching
# rootfs-overlay/etc/rauc/system.conf's own compatible=. Trade-off: RAUC no
# longer acts as a hardware-mismatch flash interlock (accepted, see
# os/README.md).
compatible=openmower
version=$OPENMOWER_VERSION

[bundle]
format=plain

[hooks]
filename=hook.sh

[image.boot]
filename=boot.vfat
hooks=post-install

[image.rootfs]
filename=rootfs.squashfs
EOF

BUNDLE_OUT="$BINARIES_DIR/openmower-$OPENMOWER_VERSION.raucb"
rm -f "$BINARIES_DIR"/openmower-*.raucb
rauc bundle \
    --cert "$OS_DIR/keys/dev-cert.pem" \
    --key "$OS_DIR/keys/dev-key.pem" \
    "$BUNDLE_DIR" "$BUNDLE_OUT"

if [ "${OPENMOWER_BUILD_SDCARD:-1}" = "1" ]; then
    echo ">> Image:  $BINARIES_DIR/sdcard.img"
    echo ">> Release asset: $IMG_GZ (+ .sha256)"
fi
echo ">> Bundle: $BUNDLE_OUT"
