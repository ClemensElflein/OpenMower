#!/bin/sh
# RAUC bundle hook. The bundle ships the slot-A boot image; when installing
# into slot B, patch cmdline.txt to point at B's rootfs partition.
set -e

case "$1" in
    slot-post-install)
        [ "$RAUC_SLOT_CLASS" = "boot" ] || exit 0
        # Three cmdline files now (cmdline.txt for [pi4]'s default,
        # cmdline-cm4.txt/cmdline-cm5.txt for those boards' console=ttyS0
        # variant -- see post-image.sh/config.txt.default) -- all three
        # carry their own root=/rauc.slot= and all three need patching, or
        # whichever board's file gets missed boots the *other* slot's
        # rootfs after the next OTA.
        for CMDLINE in "$RAUC_SLOT_MOUNT_POINT"/cmdline.txt "$RAUC_SLOT_MOUNT_POINT"/cmdline-cm4.txt "$RAUC_SLOT_MOUNT_POINT"/cmdline-cm5.txt; do
            [ -f "$CMDLINE" ] || continue
            if [ "$RAUC_SLOT_BOOTNAME" = "B" ]; then
                sed -i 's|/dev/mmcblk0p5|/dev/mmcblk0p6|; s|rauc\.slot=A|rauc.slot=B|' "$CMDLINE"
            else
                sed -i 's|/dev/mmcblk0p6|/dev/mmcblk0p5|; s|rauc\.slot=B|rauc.slot=A|' "$CMDLINE"
            fi
        done
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
