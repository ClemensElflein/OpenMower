#!/bin/bash
# Assemble the self-extracting migration installer: scripts/migrate-to-openmower.sh
# (the header/logic) + a tar payload (kernel, both dtbs, and this migration
# initramfs) appended after its marker line.
#
# Deliberately just these three -- a few tens of MB, rarely changes. The
# actual OS (sdcard.img, easily 1GB+ compressed) is NOT bundled here:
# scripts/migrate-to-openmower.sh fetches that over the network at run time
# from a static HTTPS URL (a GitHub Releases asset -- see
# external/board/openmower/post-image.sh for how it's built), which is
# what lets this installer script stay small and not need
# rebuilding/republishing every time the OS/app changes -- only when the
# kernel/dtbs/this initramfs itself changes.
#
# Needs `make image` to have already produced
# output/images/rpi-firmware/{kernel8.img,*.dtb} -- this defconfig builds no
# kernel itself, just rootfs.cpio.gz (see that defconfig's own header for
# why it gets its own output-migration/ dir instead of sharing output/ with
# prod).
#
# CM4-only, deliberately not updated for the unified CM4+CM5 image's
# multi-kernel layout -- see os/README.md. The prod build now ships CM4's
# kernel as kernel8.img (Pi-firmware auto-detect convention, alongside
# CM5's kernel_2712.img), but this migration tool's own tryboot.txt
# (scripts/migrate-to-openmower.sh) still hardcodes the plain "Image"
# filename throughout its own extraction/staging logic -- it's a
# self-contained, CM4-only one-shot boot config that never needed
# auto-detect in the first place, so kernel8.img is renamed back to Image
# when staged into the payload below, rather than touching
# migrate-to-openmower.sh's own naming at all.
set -eu

BOARD_DIR="$(cd "$(dirname "$0")" && pwd)"
OS_DIR="$(cd "$BOARD_DIR/../../.." && pwd)"
PROD_IMAGES="$OS_DIR/output/images/rpi-firmware"

OPENMOWER_VERSION="${OPENMOWER_VERSION:-$(date -u +%Y%m%d%H%M%S)}"

PROD_FILES="kernel8.img bcm2711-rpi-cm4.dtb bcm2711-rpi-4-b.dtb"
for f in $PROD_FILES; do
    if [ ! -f "$PROD_IMAGES/$f" ]; then
        echo ">> ERROR: $PROD_IMAGES/$f missing -- run 'make image' before 'make image-migration'" >&2
        exit 1
    fi
done

PAYLOAD_TAR="$(mktemp)"
trap 'rm -f "$PAYLOAD_TAR"' EXIT

# shellcheck disable=SC2086 -- $PROD_FILES is a deliberate word-split list
tar -cf "$PAYLOAD_TAR" \
    --transform 's/^kernel8\.img$/Image/' \
    -C "$PROD_IMAGES" $PROD_FILES \
    -C "$BINARIES_DIR" rootfs.cpio.gz

PAYLOAD_SHA256="$(sha256sum "$PAYLOAD_TAR" | cut -d' ' -f1)"

OUT="$BINARIES_DIR/openmower-migrate-$OPENMOWER_VERSION.sh"
rm -f "$BINARIES_DIR"/openmower-migrate-*.sh
# 0,/re/ restricts the substitution to (at most) the first matching line in
# the whole file, not "first match per line" like a plain s/// would -- the
# placeholder is only supposed to appear once, but a bare s/// silently
# replaces it everywhere it appears instead of erroring if that ever stops
# being true, which is exactly how this shipped broken once already (see
# migrate-to-openmower.sh's own PAYLOAD_SHA256 validation for the other
# half of that fix).
sed "0,/@@PAYLOAD_SHA256@@/{s/@@PAYLOAD_SHA256@@/$PAYLOAD_SHA256/}" "$OS_DIR/scripts/migrate-to-openmower.sh" > "$OUT"
cat "$PAYLOAD_TAR" >> "$OUT"
chmod +x "$OUT"

echo ">> Installer: $OUT ($(du -h "$OUT" | cut -f1))"
