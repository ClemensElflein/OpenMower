# Default ~/.bashrc for root, styled after stock Raspberry Pi OS's own
# (color prompt, ls/grep colors, common aliases) so this feels familiar
# over SSH. Buildroot's bash package ships plain upstream bash, no Debian
# skel to inherit this from, hence writing it out here instead.
#
# Lives in /etc/skel (not directly at /root/.bashrc): /root is a symlink
# onto /data/root (see post-build.sh) so root's home persists across
# reboots/updates, and this file is only the seed a tmpfiles.d "C" line
# copies onto /data/root/.bashrc the first time -- never overwriting an
# already-persisted copy, so on-device edits survive.

case $- in
    *i*) ;;
      *) return ;;  # non-interactive, nothing to do
esac

# Red prompt for root -- matches Debian/Raspberry Pi OS's convention of an
# unmistakable "you are root" visual cue (normal users would get green,
# but there are no other interactive users on this image).
PS1='\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]# '

HISTCONTROL=ignoreboth
HISTSIZE=1000
HISTFILESIZE=2000
shopt -s histappend
shopt -s checkwinsize

if ls --color=auto >/dev/null 2>&1; then
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
fi
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
