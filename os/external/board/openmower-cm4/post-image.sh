#!/bin/bash
# Assemble sdcard.img (A/B layout) and the signed RAUC update bundle.
set -eu

BOARD_DIR="$(cd "$(dirname "$0")" && pwd)"
OS_DIR="$(cd "$BOARD_DIR/../../.." && pwd)"

OPENMOWER_VERSION="${OPENMOWER_VERSION:-$(date -u +%Y%m%d%H%M%S)}"

# --- Build boot filesystem images (differ only in cmdline.txt) ---------------
# Contents: Pi firmware (incl. our config.txt), kernel, DTBs.
BOOT_FILES=("$BINARIES_DIR"/rpi-firmware/* "$BINARIES_DIR"/Image "$BINARIES_DIR"/*.dtb)

make_boot_vfat() {
    local out="$1" cmdline="$2"
    rm -f "$out"
    mkfs.vfat -C -n "$3" "$out" $((128 * 1024))   # size in 1K blocks
    mcopy -i "$out" -s -Q "${BOOT_FILES[@]}" ::/
    mcopy -i "$out" -o -Q "$cmdline" ::/cmdline.txt
    # config.txt's `include usercfg.txt` tolerates a missing target, but bake
    # the shipped default in anyway rather than lean on that -- real content
    # gets synced on top of this from /data on first boot (see
    # openmower-sync-bootcfg) and on every OTA install (see rauc-hook.sh).
    mcopy -i "$out" -o -Q "$TARGET_DIR/etc/openmower/usercfg.txt.default" ::/usercfg.txt
}

make_boot_vfat "$BINARIES_DIR/boot-a.vfat" "$BOARD_DIR/cmdline-a.txt" BOOT-A
make_boot_vfat "$BINARIES_DIR/boot-b.vfat" "$BOARD_DIR/cmdline-b.txt" BOOT-B

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
compatible=openmower-cm4
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
