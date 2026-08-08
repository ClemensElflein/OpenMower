#!/usr/bin/env bash
# Generate DEVELOPMENT-only keys: a self-signed RAUC signing certificate,
# and an SSH host key for the dev image's openssh (baked into the image at
# build time -- see post-build.sh -- so CLion/ssh see the same host key
# across rebuilds AND reflashes, not just reboots). Both stay on this
# machine (keys/ is gitignored, never committed) -- same trade-off as the
# dev root password: convenience for local dev, not for sharing/production.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
KEYDIR="$HERE/keys"
mkdir -p "$KEYDIR"

# Each artifact generated independently (not "refuse if anything exists"):
# an existing checkout only has dev-cert.pem/dev-key.pem from before the
# SSH host key existed here, and re-running this script for everyone would
# otherwise be required just to pick up the new artifact.
if [ -f "$KEYDIR/dev-cert.pem" ]; then
    echo "keys/dev-cert.pem already exists, skipping RAUC dev cert"
else
    openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 \
        -keyout "$KEYDIR/dev-key.pem" -out "$KEYDIR/dev-cert.pem" \
        -days 3650 -nodes \
        -subj "/O=xtech/CN=openmower dev signing"
    echo "Dev signing key written to keys/dev-{cert,key}.pem (gitignored)."
    echo "The certificate is baked into the image as the RAUC keyring."
fi

if [ -f "$KEYDIR/dev-ssh-host-key" ]; then
    echo "keys/dev-ssh-host-key already exists, skipping SSH host key"
else
    ssh-keygen -q -t ed25519 -N "" -f "$KEYDIR/dev-ssh-host-key"
    echo "Dev SSH host key written to keys/dev-ssh-host-key{,.pub} (gitignored)."
    echo "Baked into the dev image's /etc/ssh/ssh_host_ed25519_key."
fi
