#!/usr/bin/env bash
# Generate a self-signed RAUC signing certificate for DEVELOPMENT.
# Production bundles must be signed by an offline CA instead — see README.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
KEYDIR="$HERE/keys"
mkdir -p "$KEYDIR"

if [ -f "$KEYDIR/dev-cert.pem" ]; then
    echo "keys/dev-cert.pem already exists, refusing to overwrite" >&2
    exit 1
fi

openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 \
    -keyout "$KEYDIR/dev-key.pem" -out "$KEYDIR/dev-cert.pem" \
    -days 3650 -nodes \
    -subj "/O=xtech/CN=openmower dev signing"

echo "Dev signing key written to keys/ (gitignored)."
echo "The certificate is baked into the image as the RAUC keyring."
