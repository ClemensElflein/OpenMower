#!/bin/sh
# RAUC bundle hook. The bundle ships the slot-A boot image; when installing
# into slot B, patch cmdline.txt to point at B's rootfs partition.
set -e

case "$1" in
    slot-post-install)
        [ "$RAUC_SLOT_CLASS" = "boot" ] || exit 0
        CMDLINE="$RAUC_SLOT_MOUNT_POINT/cmdline.txt"
        if [ "$RAUC_SLOT_BOOTNAME" = "B" ]; then
            sed -i 's|/dev/mmcblk0p5|/dev/mmcblk0p6|; s|rauc\.slot=A|rauc.slot=B|' "$CMDLINE"
        else
            sed -i 's|/dev/mmcblk0p6|/dev/mmcblk0p5|; s|rauc\.slot=B|rauc.slot=A|' "$CMDLINE"
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
