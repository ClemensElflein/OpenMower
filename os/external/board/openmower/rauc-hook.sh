#!/bin/sh
# RAUC bundle hook. The bundle always ships slot-A's boot files (see
# post-image.sh); installing into slot B needs cmdline.txt/cmdline-cm4.txt/
# cmdline-cm5.txt (root=/rauc.slot=) rewritten to point at B's rootfs
# partition, or whichever board's file gets missed boots the *other*
# slot's rootfs after the next OTA. Installing into slot A needs no
# rewrite -- the shipped files already match A.
set -eu

case "$1" in
    slot-post-install)
        [ "$RAUC_SLOT_CLASS" = "boot" ] || exit 0
        if [ "$RAUC_SLOT_BOOTNAME" = "B" ]; then
            for CMDLINE in "$RAUC_SLOT_MOUNT_POINT"/cmdline.txt "$RAUC_SLOT_MOUNT_POINT"/cmdline-cm4.txt "$RAUC_SLOT_MOUNT_POINT"/cmdline-cm5.txt; do
                [ -f "$CMDLINE" ] || continue
                sed -i 's|/dev/mmcblk0p5|/dev/mmcblk0p6|; s|rauc\.slot=A|rauc.slot=B|' "$CMDLINE"
            done
        fi
        # Re-apply the user's persistent boot-config override (config.txt's
        # `include usercfg.txt`) -- the bundle only carries the baked-in
        # default, not this device's local customizations (antenna
        # selection, etc.). Same copy openmower-sync-bootcfg applies live,
        # out-of-band from updates; RAUC (an already-running host service)
        # has /data mounted here same as any other point in normal runtime.
        [ -f /data/boot/usercfg.txt ] && cp /data/boot/usercfg.txt "$RAUC_SLOT_MOUNT_POINT/usercfg.txt"
        ;;
esac
exit 0
