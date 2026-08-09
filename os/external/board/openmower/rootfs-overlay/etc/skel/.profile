# bash only reads ~/.bashrc for non-login shells; SSH sessions are login
# shells, so without this bridge (same pattern Debian/Raspberry Pi OS's own
# skel uses) the colored prompt/aliases in .bashrc never get sourced over
# SSH, even though /bin/bash is correctly the shell.
#
# Lives in /etc/skel, not directly at /root/.profile -- see .bashrc's own
# comment for why (seeded onto persistent /data/.openmower-os/root once,
# not baked directly into the read-only rootfs).
if [ -n "$BASH_VERSION" ] && [ -f "$HOME/.bashrc" ]; then
    . "$HOME/.bashrc"
fi
