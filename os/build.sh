#!/usr/bin/env bash
# Run Buildroot inside the toolchain container.
#
# Usage:
#   ./build.sh                    # full image build (CM4+CM5 kernels + main rootfs, see below)
#   ./build.sh image-migration    # migration installer, see external/configs/openmower_cm4_migration_defconfig
#   ./build.sh menuconfig         # any Buildroot make target, against the MAIN (rootfs) build -- auto-persists to the checked-in defconfig on exit, see below
#   ./build.sh menuconfig-kernel-cm4   # same, against the cm4 kernel-only satellite build
#   ./build.sh menuconfig-kernel-cm5   # same, against the cm5 kernel-only satellite build
#   ./build.sh savedefconfig      # persist output/.config by hand (e.g. after editing it some other way)
#   ./build.sh shell              # interactive shell in the container
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
# Mount the whole repo, not just os/, so the container's layout matches the
# host's — REPO_ROOT is the open_mower_ros checkout itself (os/ lives inside
# it), and the openmower-ros package's local-package SITE points at .cache/
# (also here).
REPO_ROOT="$(cd "$HERE/.." && pwd)"
IMAGE_TAG=openmower-buildroot
DEFCONFIG="${DEFCONFIG:-openmower_defconfig}"
SUBCOMMAND="${1:-image}"

# The main build (openmower_defconfig/openmower_dev_defconfig) has no
# kernel of its own (BR2_LINUX_KERNEL=n) -- one unified image needs both
# CM4/bcm2711 and CM5/bcm2712 kernels, and Buildroot only ever builds one
# kernel per output tree, so each comes from its own tiny satellite build
# instead, in its own output dir, merged into the main rootfs by
# external/board/openmower/post-build.sh + post-image.sh. See
# openmower_kernel-cm4_defconfig/openmower_kernel-cm5_defconfig and
# os/README.md for the full why.
KERNEL_CM4_OUTPUT_DIR=/work/os/output-kernel-cm4
KERNEL_CM5_OUTPUT_DIR=/work/os/output-kernel-cm5
KERNEL_CM4_HOST_OUTPUT_DIR="$HERE/output-kernel-cm4"
KERNEL_CM5_HOST_OUTPUT_DIR="$HERE/output-kernel-cm5"

# Monotonic, numerically comparable version; git sha for traceability.
OPENMOWER_VERSION="$(date -u +%Y%m%d%H%M%S)"
OPENMOWER_GIT_REV="$(git -C "$HERE" describe --always --dirty 2>/dev/null || echo unknown)"
OPENMOWER_GIT_HASH_FULL="$(git -C "$HERE" rev-parse HEAD 2>/dev/null || echo unknown)"
OPENMOWER_GIT_HASH="$(git -C "$HERE" rev-parse --short=8 HEAD 2>/dev/null || echo unknown)"

# Branch name for /usr/share/openmoweros/version.* (see post-build.sh) --
# best-effort only, CI builds commonly run on a detached HEAD where plain
# rev-parse just says "HEAD". Checked in this order: CI-provided env vars
# (covers GitHub Actions/GitLab/Azure DevOps/Jenkins, whichever actually
# set the job up), then rev-parse (works for a genuine local branch
# checkout), then a remote ref pointing at the same commit (covers a CI
# checkout that left a detached HEAD but still fetched named refs), then
# give up with a hash-labeled placeholder rather than a bare "unknown".
detect_branch() {
    local var val rp commit remote_head
    for var in GITHUB_HEAD_REF GITHUB_REF_NAME GIT_BRANCH CI_COMMIT_REF_NAME CI_BUILD_REF_NAME BUILD_SOURCEBRANCHNAME; do
        val="${!var:-}"
        if [ -n "$val" ]; then
            case "$val" in
                refs/heads/*) val="${val#refs/heads/}" ;;
                refs/tags/*) val="${val#refs/tags/}" ;;
            esac
            printf '%s' "$val"
            return 0
        fi
    done
    rp="$(git -C "$HERE" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
    if [ -n "$rp" ] && [ "$rp" != "HEAD" ]; then
        printf '%s' "$rp"
        return 0
    fi
    commit="$(git -C "$HERE" rev-parse HEAD 2>/dev/null || true)"
    if [ -n "$commit" ]; then
        remote_head="$(git -C "$HERE" for-each-ref --format='%(objectname) %(refname:short)' refs/remotes 2>/dev/null | awk -v c="$commit" '$1==c {print $2; exit}')"
        if [ -n "$remote_head" ]; then
            printf '%s' "${remote_head#origin/}"
            return 0
        fi
        printf 'detached-%s' "${commit:0:8}"
        return 0
    fi
    printf 'unknown'
}
OPENMOWER_GIT_BRANCH="$(detect_branch)"

# The migration initramfs gets its OWN output dir (output-migration), not
# output/. It shares no packages with the prod/dev rootfs (systemd vs.
# no-init, squashfs vs. cpio, ...), and Buildroot has no mechanism to prune
# a package's installed files from target/ when a defconfig switch disables
# it -- sharing output/ with prod produced a rootfs.cpio with the *entire*
# leftover prod target/ (vendored ROS tree included, 5GB+) baked in instead
# of the tiny busybox image it's supposed to be. Separate output dirs sidesteps
# the problem entirely. .cache/dl and .cache/ccache are still shared (below),
# so this doesn't mean rebuilding the toolchain from scratch.
if [ "$SUBCOMMAND" = "image-migration" ] || [ "$SUBCOMMAND" = "menuconfig-migration" ] || [ "$SUBCOMMAND" = "savedefconfig-migration" ]; then
    OUTPUT_DIR=/work/os/output-migration
    DEFCONFIG=openmower_cm4_migration_defconfig
elif [ "$SUBCOMMAND" = "menuconfig-kernel-cm4" ] || [ "$SUBCOMMAND" = "savedefconfig-kernel-cm4" ]; then
    OUTPUT_DIR="$KERNEL_CM4_OUTPUT_DIR"
    DEFCONFIG=openmower_kernel-cm4_defconfig
elif [ "$SUBCOMMAND" = "menuconfig-kernel-cm5" ] || [ "$SUBCOMMAND" = "savedefconfig-kernel-cm5" ]; then
    OUTPUT_DIR="$KERNEL_CM5_OUTPUT_DIR"
    DEFCONFIG=openmower_kernel-cm5_defconfig
else
    OUTPUT_DIR=/work/os/output
fi

mkdir -p "$HERE/.cache/dl" "$HERE/.cache/ccache" "$HERE/output" "$HERE/output-migration" \
    "$KERNEL_CM4_HOST_OUTPUT_DIR" "$KERNEL_CM5_HOST_OUTPUT_DIR"

# Every image/image-migration build forces a fresh target/ + staging/ --
# Buildroot's own incremental build never deletes a target-dir file whose
# source (rootfs-overlay entry, or a line removed from some package's
# INSTALL_TARGET_CMDS) disappeared; it only adds/updates. A stale unit file
# left behind that way, with no matching system-preset line, silently falls
# through to that preset's own default and ships live regardless -- that's
# how a .service deleted from this repo weeks earlier was still running,
# and boot-blocking, on a real device.
#
# This is NOT a full `rm -rf $(O)` (that was the first fix -- correct, but
# needlessly paid for re-running configure on every one of ~80 packages
# every time, since Buildroot's own compiled-output stamps live in the
# same tree it was wiping). Instead: wipe only target/ and staging/, and
# delete every package's .stamp_target_installed/.stamp_staging_installed
# -- unconditionally, no "did this package change" logic, so there's no
# detector here that can be wrong the way an actual staleness-detector
# could be. Make's own dependency graph then reinstalls EVERY package
# (compiled or not) into the now-empty target/, which is what actually
# guarantees no stale file survives -- while leaving .stamp_built alone
# means a package whose source didn't change skips straight past
# extract/patch/configure/build entirely, using Buildroot's own mature
# incremental logic for that part, unchanged and untouched by us.
#
# The one gap that logic doesn't cover on its own: SITE_METHOD=local packages
# (openmower-ros, improv-ble), where Buildroot doesn't auto-detect the local
# source tree changing underneath an already-built .stamp_built --
# scripts/rebuild-changed-local-packages.sh (below) exists specifically for
# that.
HOST_OUTPUT_DIR="$HERE${OUTPUT_DIR#/work/os}"

# Found by testing, not by inspection: a package can also install straight
# into images/ (.stamp_images_installed, pkg-generic.mk's
# TARGET_INSTALL_IMAGES -- e.g. rpi-firmware) as a THIRD install stage
# distinct from target/staging. Missing it here didn't corrupt anything
# silently -- Buildroot considered rpi-firmware already "installed" (stamp
# present) while its actual output sat in the images/ dir this script had
# just deleted, so post-image.sh failed loudly and immediately on the
# missing files. Loud-and-immediate is exactly the failure mode this whole
# approach depends on staying in -- but still worth getting right rather
# than relying on it.
wipe_output_dir() {
    local dir="$1"
    rm -rf "$dir/target" "$dir/staging" "$dir/images"
    mkdir -p "$dir/target" "$dir/staging" "$dir/images"
    if [ -d "$dir/build" ]; then
        find "$dir/build" -maxdepth 2 \
            \( -name '.stamp_target_installed' -o -name '.stamp_staging_installed' -o -name '.stamp_images_installed' \) \
            -delete
    fi
}

if [ "$SUBCOMMAND" = "image" ]; then
    # All three builds (both kernel satellites + the main rootfs build) --
    # same staleness reasoning applies to each; the satellites are cheap
    # relative to the main build but not exempt from the same class of bug.
    wipe_output_dir "$KERNEL_CM4_HOST_OUTPUT_DIR"
    wipe_output_dir "$KERNEL_CM5_HOST_OUTPUT_DIR"
    wipe_output_dir "$HOST_OUTPUT_DIR"
elif [ "$SUBCOMMAND" = "image-migration" ]; then
    wipe_output_dir "$HOST_OUTPUT_DIR"
fi

# Keep submodules (os/buildroot, src/lib/xbot_driver_gps, ...) in sync; an
# uninitialized/stale submodule leaves buildroot/ empty and make fails with
# cryptic "No rule to make target" errors, and OMR_SOURCE=local below would
# docker-build an incomplete open_mower_ros tree.
git -C "$HERE" submodule update --init --recursive

# Buildroot's own incremental build only tracks a package's own SOURCE
# changing -- not some OTHER package's Kconfig `select` newly pulling in a
# suboption it didn't have before (e.g. wavemon selecting
# BR2_PACKAGE_LIBNL_TOOLS onto an already-built libnl). .stamp_configured
# survives untouched and the old build gets silently re-staged instead of
# reconfigured: same bug class as the target/staging wipe above, one stage
# earlier, which is exactly why wipe_output_dir deliberately leaves
# .stamp_configured/.stamp_built alone (that's the whole point of trusting
# Buildroot's own incremental logic for those, per the comment above it).
# Mirrors the content-hash-keyed CI cache split (os-build-output): hash the
# defconfigs + external/ Kconfig inputs + the pinned Buildroot commit, and
# only when that changed since the last local build, force every package
# back through its full extract/patch/configure/build cycle with the
# current .config. Main build only -- the migration/kernel satellites don't
# have this class of cross-package select dependency.
#
# Removes each package's WHOLE directory, not just the .stamp_configured/
# .stamp_built files: an earlier version of this only deleted those two
# stamps, which broke any Autotools package with AUTORECONF=YES (e.g.
# e2fsprogs). For those, Buildroot's own pkg-autotools.mk puts
# LIBTOOL_PATCH_HOOK on PRE_CONFIGURE_HOOKS instead of POST_PATCH_HOOKS, so
# it's gated by .stamp_configured, not .stamp_patched -- deleting only
# .stamp_configured left the already-libtool-patched source on disk and
# reran the (non-idempotent, no -N/-t) `patch -i .../ltmain.sh` against it
# on the forced reconfigure, which is exactly the "Reversed (or previously
# applied) patch detected!" failure this was hitting on every single run
# where the hash actually changed. Re-extracting from .cache/dl (still
# warm) is cheap; ccache still makes the recompiles fast, so the only real
# cost of doing this instead is redoing configure/autoreconf.
if [ "$OUTPUT_DIR" = "/work/os/output" ]; then
    BUILDROOT_REV="$(git -C "$HERE/buildroot" rev-parse HEAD 2>/dev/null || echo unknown)"
    CONFIG_INPUT_FILES="$(find "$HERE/external/configs/openmower_defconfig" "$HERE/external/configs/openmower_dev_defconfig" "$HERE/external/Config.in" "$HERE/external/package" -type f 2>/dev/null | sort)"
    CONFIG_HASH="$( { printf '%s\n' "$BUILDROOT_REV"; sha256sum $CONFIG_INPUT_FILES; } | sha256sum | cut -d' ' -f1)"
    CONFIG_HASH_FILE="$HERE/.cache/output-build.hash"
    if [ ! -f "$CONFIG_HASH_FILE" ] || [ "$(cat "$CONFIG_HASH_FILE")" != "$CONFIG_HASH" ]; then
        if [ -d "$HOST_OUTPUT_DIR/build" ]; then
            echo ">> Buildroot config inputs changed, forcing full package re-extract/reconfigure"
            find "$HOST_OUTPUT_DIR/build" -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} +
        fi
        echo -n "$CONFIG_HASH" > "$CONFIG_HASH_FILE"
    fi
fi

if [ ! -f "$HERE/keys/dev-cert.pem" ]; then
    echo ">> No RAUC dev keys found, generating (keys/)"
    "$HERE/keys-gen-dev.sh"
fi

# --- Vendor the open_mower_ros ROS Noetic install tree (arm64) --------------
# Produced on the HOST (not nested inside the toolchain container below —
# no docker-in-docker). The buildroot package external/package/openmower-ros
# vendors the resulting tree wholesale into the target rootfs; see its
# Config.in help text for the runtime design (systemd RootDirectory=
# isolation, no container runtime on-device).
#
# Three sources, picked by OMR_SOURCE:
#   ghcr        (default) — pull the prebuilt multi-arch image CI already
#                            publishes, just extract it. No arm64 emulation
#                            needed (no code runs, just image layers).
#   local-image            — OMR_IMAGE is already sitting in the local
#                             docker daemon, no pull. What CI uses: the
#                             open_mower_ros "Build" workflow already builds
#                             the real arm64 image from this exact PR/commit
#                             (native arm64 runner, no emulation there
#                             either) and exports it as an artifact; this OS
#                             build just docker-loads and vendors that,
#                             instead of pulling an unrelated ghcr tag that
#                             may not even reflect this change yet.
#   local                  — cross-build docker/Dockerfile locally via
#                             docker buildx --platform linux/arm64
#                             (emulated, slow) — use this to test
#                             open_mower_ros changes on your own machine
#                             before they're pushed/built in CI.
# Override the image with OMR_IMAGE, e.g. a specific sha/PR/version tag.
#
# Either way this is gated behind a content-hash key so a normal
# ./build.sh doesn't pay for a re-pull/rebuild every time — only when the
# resolved image (ghcr) or this repo's HEAD (local) changed.
#
# Skipped entirely for the migration and kernel-satellite build variants:
# the migration initramfs is a tiny busybox image with no openmower-ros
# package in it, and the kernel-cm4/kernel-cm5 satellites build no rootfs
# at all (BR2_LINUX_KERNEL=y and nothing else) -- pulling multi-GB of
# ROS/Ubuntu for either would be pure waste. Only the main build (output/)
# actually vendors this.
if [ "$OUTPUT_DIR" = "/work/os/output" ]; then
    OMR_SOURCE="${OMR_SOURCE:-ghcr}"
    OMR_IMAGE="${OMR_IMAGE:-ghcr.io/clemenselflein/open_mower_ros:edge}"
    # os/ lives inside the open_mower_ros checkout itself now (no separate
    # sibling submodule) -- REPO_ROOT *is* the open_mower_ros source tree,
    # and its docker/Dockerfile is what the ghcr image is built from too.
    OMR_DIR="$REPO_ROOT"
    OMR_CACHE="$HERE/.cache/openmower-rootfs"
    OMR_KEYFILE="$HERE/.cache/openmower-rootfs.key"

    case "$OMR_SOURCE" in
        ghcr)
            echo ">> Pulling open_mower_ros from GHCR: $OMR_IMAGE"
            docker pull --platform linux/arm64 "$OMR_IMAGE"
            OMR_KEY="ghcr-$(docker image inspect --format '{{.Id}}' "$OMR_IMAGE")"
            ;;
        local-image)
            # OMR_IMAGE is already sitting in the local docker daemon (e.g.
            # CI `docker load`-ing the exact arm64 image open_mower_ros' own
            # "Build" workflow just built from this same PR/commit's source
            # -- see build-image.yaml/push-pr-image.yaml) -- use it as-is,
            # no pull.
            echo ">> Using local open_mower_ros image: $OMR_IMAGE"
            OMR_KEY="local-image-$(docker image inspect --format '{{.Id}}' "$OMR_IMAGE")"
            ;;
        local)
            # HEAD, not a hash of docker/Dockerfile alone -- COPY ./ in the
            # assemble stage pulls in the whole build context (src/, utils/,
            # docker/assets/, ...), all of which invalidates the vendored
            # tree just as much as the Dockerfile itself. os/ is excluded
            # from that context (.dockerignore), so edits confined to os/
            # don't trigger a spurious ROS rebuild here.
            OMR_KEY="local-$(git -C "$OMR_DIR" rev-parse HEAD)"
            ;;
        *)
            echo "error: OMR_SOURCE must be 'ghcr', 'local-image' or 'local' (got '$OMR_SOURCE')" >&2
            exit 1
            ;;
    esac

    if [ ! -f "$OMR_KEYFILE" ] || [ "$(cat "$OMR_KEYFILE")" != "$OMR_KEY" ]; then
        echo ">> open_mower_ros source changed, refreshing vendored ROS rootfs"
        rm -rf "$OMR_CACHE"
        mkdir -p "$OMR_CACHE"
        case "$OMR_SOURCE" in
            ghcr | local-image)
                cid="$(docker create --platform linux/arm64 "$OMR_IMAGE")"
                docker export "$cid" | tar -xf - -C "$OMR_CACHE"
                docker rm "$cid" >/dev/null
                ;;
            local)
                docker buildx build --platform linux/arm64 \
                    -f "$OMR_DIR/docker/Dockerfile" \
                    --target assemble \
                    --output "type=local,dest=$OMR_CACHE" \
                    "$OMR_DIR"
                ;;
        esac
        # Consumed by scripts/rebuild-changed-local-packages.sh as a fast-path
        # instead of hashing every file in this (huge) tree on every invocation.
        echo -n "$OMR_KEY" > "$OMR_CACHE/.br-content-hash"
        echo -n "$OMR_KEY" > "$OMR_KEYFILE"
    fi

    # --- Vendor openmower-cli: always the latest GitHub release ------------
    # Deliberately not Buildroot's normal pinned-URL + static .hash download
    # (what this used to be): a moving "latest" target has no fixed bytes to
    # pin a hash against ahead of time, and BR2_DOWNLOAD_FORCE_CHECK_HASHES=y
    # (both defconfigs) would refuse an unverified download outright. Same
    # shape as the open_mower_ros vendoring above instead: resolve here,
    # stage into .cache/, gate on a content-hash key so a normal ./build.sh
    # doesn't re-fetch every time -- only when the latest release actually
    # changed. openmower-cli.mk is SITE_METHOD=local pointed at this cache,
    # matching openmower-ros.mk/improv-ble.mk, neither of which uses
    # Buildroot's hash mechanism either.
    #
    # Trade-off worth naming: this trades reproducibility (same build.sh
    # invocation can fetch different bytes on different days) and a human
    # review gate (bumping a pinned tag was an explicit, reviewable diff) for
    # always being current. Integrity-wise it's not actually a downgrade --
    # the old pinned .hash was itself just whatever `sha256sum` produced from
    # the same kind of plain HTTPS download, not an independently signed
    # attestation from upstream.
    OPENMOWER_CLI_REPO="ClemensElflein/openmower-cli"
    OPENMOWER_CLI_CACHE="$HERE/.cache/openmower-cli"
    OPENMOWER_CLI_KEYFILE="$HERE/.cache/openmower-cli.key"

    OPENMOWER_CLI_RELEASE_JSON="$(curl -fsSL "https://api.github.com/repos/$OPENMOWER_CLI_REPO/releases/latest")"
    OPENMOWER_CLI_TAG="$(printf '%s' "$OPENMOWER_CLI_RELEASE_JSON" | grep -m1 '"tag_name"' | sed -E 's/.*"tag_name": *"([^"]+)".*/\1/')"
    OPENMOWER_CLI_ASSET_URL="$(printf '%s' "$OPENMOWER_CLI_RELEASE_JSON" | grep -m1 '"browser_download_url"' | sed -E 's/.*"browser_download_url": *"([^"]+)".*/\1/')"
    if [ -z "$OPENMOWER_CLI_TAG" ] || [ -z "$OPENMOWER_CLI_ASSET_URL" ]; then
        echo "error: failed to resolve the latest openmower-cli release from GitHub" >&2
        exit 1
    fi

    if [ ! -f "$OPENMOWER_CLI_KEYFILE" ] || [ "$(cat "$OPENMOWER_CLI_KEYFILE")" != "$OPENMOWER_CLI_TAG" ]; then
        echo ">> openmower-cli: fetching latest release $OPENMOWER_CLI_TAG"
        rm -rf "$OPENMOWER_CLI_CACHE" "$OPENMOWER_CLI_CACHE.zip"
        mkdir -p "$OPENMOWER_CLI_CACHE"
        curl -fsSL "$OPENMOWER_CLI_ASSET_URL" -o "$OPENMOWER_CLI_CACHE.zip"
        unzip -q -o -d "$OPENMOWER_CLI_CACHE" "$OPENMOWER_CLI_CACHE.zip"
        rm -f "$OPENMOWER_CLI_CACHE.zip"
        # Consumed by scripts/rebuild-changed-local-packages.sh as a fast-path.
        echo -n "$OPENMOWER_CLI_TAG" > "$OPENMOWER_CLI_CACHE/.br-content-hash"
        echo -n "$OPENMOWER_CLI_TAG" > "$OPENMOWER_CLI_KEYFILE"
    fi
fi

docker build -q -t "$IMAGE_TAG" "$HERE/docker" >/dev/null

DOCKER_ARGS=(
    --rm
    --user "$(id -u):$(id -g)"
    -v "$REPO_ROOT:/work"
    -e BR2_DL_DIR=/work/os/.cache/dl
    -e OPENMOWER_VERSION="$OPENMOWER_VERSION"
    -e OPENMOWER_GIT_REV="$OPENMOWER_GIT_REV"
    -e OPENMOWER_GIT_HASH="$OPENMOWER_GIT_HASH"
    -e OPENMOWER_GIT_HASH_FULL="$OPENMOWER_GIT_HASH_FULL"
    -e OPENMOWER_GIT_BRANCH="$OPENMOWER_GIT_BRANCH"
    -w /work/os/buildroot
)
# Forwarded only if set on the host -- post-image.sh's own default (build the
# full sdcard.img) is unchanged for a bare ./build.sh / make image. CI sets
# OPENMOWER_BUILD_SDCARD=0 for ordinary PR/branch builds (bundle-only; see
# post-image.sh), 1 for tagged releases.
if [ -n "${OPENMOWER_BUILD_SDCARD+x}" ]; then
    DOCKER_ARGS+=(-e OPENMOWER_BUILD_SDCARD="$OPENMOWER_BUILD_SDCARD")
fi
[ -t 0 ] && DOCKER_ARGS+=(-it)

BR_MAKE=(make O="$OUTPUT_DIR" "BR2_EXTERNAL=/work/os/external")
KERNEL_BR_MAKE_CM4=(make O="$KERNEL_CM4_OUTPUT_DIR" "BR2_EXTERNAL=/work/os/external")
KERNEL_BR_MAKE_CM5=(make O="$KERNEL_CM5_OUTPUT_DIR" "BR2_EXTERNAL=/work/os/external")

case "$SUBCOMMAND" in
    shell)
        exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" bash
        ;;
    image)
        # Sequential, not parallel: Buildroot's shared BR2_DL_DIR/ccache
        # usage here is only relied upon for sequential invocations
        # elsewhere in this script, not simultaneous ones. The cm5 kernel
        # build is genuinely new added cost (the cm4 one was already
        # happening today, just as part of the single combined build) --
        # see os/README.md.
        exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" \
            bash -c "
                ${KERNEL_BR_MAKE_CM4[*]} openmower_kernel-cm4_defconfig && ${KERNEL_BR_MAKE_CM4[*]} &&
                ${KERNEL_BR_MAKE_CM5[*]} openmower_kernel-cm5_defconfig && ${KERNEL_BR_MAKE_CM5[*]} &&
                ${BR_MAKE[*]} $DEFCONFIG &&
                /work/os/scripts/rebuild-changed-local-packages.sh /work/os/buildroot $OUTPUT_DIR /work/os/external &&
                ${BR_MAKE[*]}
            "
        ;;
    image-migration)
        # No local-package rebuild step -- the migration defconfig enables none.
        exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" \
            bash -c "${BR_MAKE[*]} $DEFCONFIG && ${BR_MAKE[*]}"
        ;;
    menuconfig | menuconfig-migration | menuconfig-kernel-cm4 | menuconfig-kernel-cm5)
        # A fresh output directory has no .config, so Buildroot would otherwise
        # open menuconfig with its generic (x86) defaults.
        if [ ! -f "$HERE${OUTPUT_DIR#/work/os}/.config" ]; then
            docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" \
                bash -c "${BR_MAKE[*]} $DEFCONFIG && ${BR_MAKE[*]} menuconfig"
        else
            docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" "${BR_MAKE[@]}" menuconfig
        fi
        # Auto-persist: without this, the very next `./build.sh` starts with
        # `make $DEFCONFIG`, which wholesale-regenerates output/.config FROM
        # the checked-in file below -- silently wiping whatever menuconfig
        # just set there instead. Harmless no-op diff if nothing changed
        # (exited menuconfig without saving, or saved back the same values).
        # DEFCONFIG already resolves to the right variant for whichever of
        # these four SUBCOMMAND values was used (set in the OUTPUT_DIR/
        # DEFCONFIG selection above), so this needs no separate case per
        # variant.
        exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" "${BR_MAKE[@]}" \
            savedefconfig BR2_DEFCONFIG=/work/os/external/configs/$DEFCONFIG
        ;;
    savedefconfig-migration)
        exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" "${BR_MAKE[@]}" \
            savedefconfig BR2_DEFCONFIG=/work/os/external/configs/openmower_cm4_migration_defconfig
        ;;
    savedefconfig | savedefconfig-kernel-cm4 | savedefconfig-kernel-cm5)
        # Writes the CURRENT output/.config (whatever menuconfig left behind)
        # back out to the checked-in $DEFCONFIG file -- needed because
        # `./build.sh image` always starts with `make $DEFCONFIG`, which
        # wholesale-regenerates output/.config FROM that checked-in file. A
        # menuconfig change that's only ever been saved to output/.config
        # (not back to here) is silently gone the next time you build.
        # DEFCONFIG=openmower_dev_defconfig ./build.sh savedefconfig to
        # target the dev variant instead of the default; the -kernel-cm4/
        # -kernel-cm5 suffixes target those satellite builds instead (their
        # own OUTPUT_DIR/DEFCONFIG resolved above, same as menuconfig-*).
        exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" "${BR_MAKE[@]}" \
            savedefconfig BR2_DEFCONFIG=/work/os/external/configs/$DEFCONFIG
        ;;
    *)
        exec docker run "${DOCKER_ARGS[@]}" "$IMAGE_TAG" "${BR_MAKE[@]}" "$@"
        ;;
esac
