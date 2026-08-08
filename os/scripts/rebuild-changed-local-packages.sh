#!/usr/bin/env bash
# Rebuild enabled BR2_EXTERNAL packages that use SITE_METHOD = local when their
# local source tree has changed. Buildroot intentionally treats such packages
# as OVERRIDE_SRCDIR development trees and otherwise requires <pkg>-rebuild.
set -euo pipefail

BUILDROOT_DIR=$1
OUTPUT_DIR=$2
EXTERNAL_DIR=$3

state_file="$OUTPUT_DIR/.local-package-source-hashes"
state_tmp=$(mktemp "$OUTPUT_DIR/.local-package-source-hashes.XXXXXX")
trap 'rm -f "$state_tmp"' EXIT

source_hash() {
    local source_dir=$1

    # Fast path: a package's source dir can carry a precomputed content-hash
    # marker (written by whatever produced it, e.g. build.sh's docker buildx
    # export step) instead of paying for a full per-file walk here. Needed
    # for local packages whose source tree is too large to hash on every
    # invocation (tens/hundreds of thousands of files) — the two-file
    # improv-ble package never sets this and falls through to the walk below
    # unchanged.
    if [ -f "$source_dir/.br-content-hash" ]; then
        cat "$source_dir/.br-content-hash"
        return
    fi

    (
        cd "$source_dir"
        while IFS= read -r -d '' file; do
            printf '%s %s ' "$(stat -c '%a' "$file")" "$file"
            if [ -L "$file" ]; then
                readlink "$file"
            else
                sha256sum "$file"
            fi
        done < <(
            find . \
                \( -name .git -o -name .hg -o -name .svn -o -name .bzr \) -prune -o \
                \( -type f -o -type l \) -print0 | LC_ALL=C sort -z
        )
    ) | sha256sum | awk '{print $1}'
}

previous_hash() {
    [ -f "$state_file" ] || return 0
    awk -v package="$1" '$1 == package { print $2; exit }' "$state_file"
}

rebuild_targets=()

while IFS= read -r -d '' package_mk; do
    package_target=$(basename "$package_mk" .mk)
    package_prefix=$(sed -nE 's/^([A-Z0-9_]+)_SITE_METHOD[[:space:]]*=[[:space:]]*local[[:space:]]*$/\1/p' "$package_mk")
    [ -n "$package_prefix" ] || continue

    # generic-package convention: BR2_PACKAGE_FOO_BAR enables FOO_BAR.
    grep -qx "BR2_PACKAGE_${package_prefix}=y" "$OUTPUT_DIR/.config" || continue

    source_dir=$(sed -nE "s|^${package_prefix}_SITE[[:space:]]*=[[:space:]]*||p" "$package_mk" | head -n1)
    source_dir=${source_dir//\$\(BR2_EXTERNAL_OPENMOWER_PATH\)/$EXTERNAL_DIR}
    if [ -z "$source_dir" ] || [ ! -d "$source_dir" ]; then
        echo "error: cannot resolve local source for $package_target" >&2
        exit 1
    fi

    # Also hash the package's own directory (Config.in, *.mk, and any files/
    # it installs itself), not just SITE. For packages where SITE *is* their
    # files/ dir (improv-ble) this is redundant but harmless. It's not
    # redundant for packages like openmower-ros, whose SITE points at an
    # external vendored tree (.cache/openmower-rootfs) --
    # without this, edits to its own files/openmower.service etc. were
    # invisible to this script entirely, so a real source change never
    # triggered a rebuild.
    package_dir=$(dirname "$package_mk")
    current_hash=$(printf '%s %s' "$(source_hash "$package_dir")" "$(source_hash "$source_dir")" | sha256sum | awk '{print $1}')
    printf '%s %s\n' "$package_target" "$current_hash" >> "$state_tmp"
    if [ "$current_hash" != "$(previous_hash "$package_target")" ]; then
        rebuild_targets+=("${package_target}-rebuild")
    fi
done < <(find "$EXTERNAL_DIR/package" -mindepth 2 -maxdepth 2 -name '*.mk' -print0 | LC_ALL=C sort -z)

if [ "${#rebuild_targets[@]}" -gt 0 ]; then
    printf '>> Rebuilding changed local packages:'
    printf ' %s' "${rebuild_targets[@]%-rebuild}"
    printf '\n'
    make -C "$BUILDROOT_DIR" O="$OUTPUT_DIR" "BR2_EXTERNAL=$EXTERNAL_DIR" "${rebuild_targets[@]}"
fi

mv "$state_tmp" "$state_file"
trap - EXIT
