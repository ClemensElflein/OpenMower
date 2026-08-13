# OpenMower OS

Embedded Linux for the OpenMower robot, running this repo's own
[open_mower_ros](https://github.com/ClemensElflein/open_mower_ros)
(ROS Noetic) natively instead of in Docker — this directory (`os/`) is the
OS build system, living inside the `open_mower_ros` checkout itself (see
[Layout](#layout)). Buildroot-based, A/B partitioned with automatic
rollback, self-updating over the network, wifi-provisionable over BLE.

- **Targets:** Raspberry Pi CM4 (product) and CM5 (product) — **one unified image** boots either, mirroring how stock Raspberry Pi OS ships a single SD image across Pi generations. See [Unified CM4+CM5 image](#unified-cm4cm5-image) below.
- **Init:** systemd; root is read-only squashfs, persistent state on `/data`
- **A/B boot:** Raspberry Pi firmware `tryboot` (no U-Boot)
- **OTA:** RAUC signed bundles, pulled from a static HTTPS server
- **Provisioning:** Improv Wi-Fi over BLE
- **ROS stack:** vendored, run via `systemd-nspawn` (no image-store copy, no docker/podman for this part) — see [Running open_mower_ros](#running-open_mower_ros) below. Not started automatically; start explicitly (`systemctl start openmower.service`, eventually `openmower-cli`) once the mower is actually configured.
- **Auxiliary stack:** Mosquitto, OpenMowerApp, Dockge, ttyd — Docker Compose, scoped to just these small third-party images — see [Manage the auxiliary stack](#manage-the-auxiliary-stack) below. Mosquitto+OpenMowerApp are not started automatically either (Dockge and ttyd are).

## Unified CM4+CM5 image

`make image` builds **three** Buildroot trees, sequentially: a CM4/bcm2711
kernel-only satellite (`openmower_kernel-cm4_defconfig` →
`output-kernel-cm4/`), a CM5/bcm2712 kernel-only satellite
(`openmower_kernel-cm5_defconfig` → `output-kernel-cm5/`), then the real
rootfs/userland build (`openmower_defconfig` → `output/`, `BR2_LINUX_KERNEL=n`
— no kernel of its own). The main build's `post-build.sh`/`post-image.sh`
splice both satellites' `Image`/DTBs/`lib/modules/` output in directly, and
ship `kernel8.img` + `kernel_2712.img` + both boards' DTBs side by side in
the one boot partition. `config.txt.default` picks the right kernel file and
DTB per board **explicitly**, via `[cm4]`/`[pi4]`/`[cm5]` model-filter
sections (the same mechanism `usercfg.txt.default` already uses for
antenna/fan) — **not** by leaving `kernel=`/`device_tree=` unset for Pi
firmware auto-detect. Auto-detect-by-omission is real, documented firmware
behavior and was the original design here, but it depends on the specific
pinned firmware blob version (`buildroot/package/rpi-firmware/rpi-firmware.mk`)
actually supporting it — confirmed the hard way: an early build of this
unified image left `kernel=`/`device_tree=`/`start_file=`/`fixup_file=`
unset and didn't boot on real CM4 hardware at all. Explicit per-board
directives sidestep that dependency entirely instead of relying on it.

Why two extra builds instead of a `openmower_cm5_defconfig` alongside the
existing one: Buildroot only ever builds one kernel per output tree, and
one image booting both boards needs two. The userland itself stays one
build, tuned to `cortex-a53` (ARMv8.0-A baseline, same ISA level as CM4's
cortex-a72) rather than CM5's cortex-a76 (ARMv8.2-A) specifically, so it
runs correctly on either CPU — cortex-a76-tuned code can use ISA extensions
(e.g. LSE atomics) that `SIGILL` on cortex-a72.

Which physical board is actually running is resolved **at boot**, not at
build time: `openmower-detect-hardware` (systemd oneshot, reads
`/proc/device-tree/model`) writes `/run/openmower/hardware` and symlinks
`/etc/openocd/xcore.cfg` to `xcore-cm4.cfg` or `xcore-cm5.cfg` — the SWD
bitbang driver for flashing/debugging the xCore MCU differs per board
(`bcm2835gpio`, direct register access, CM4 only; `linuxgpiod`, kernel
gpiochip device, needed on CM5 since it dropped that direct register
access) and can't be picked at build time the way everything else here is.

CM5 uses the `bcm2712-rpi-cm5(l)-cm4io` device trees (CM5 module on the
official Raspberry Pi CM4 IO Board pinout), not RPi's own CM5 IO Board
ones — OpenMower's carrier board is pin/GPIO-compatible with the CM4 IO
Board (same fan GPIO18, same `ant1`/`ant2` antenna params, see
`usercfg.txt.default`'s `[cm4]`/`[cm5]` sections), and CM5 was designed for
backward compatibility with that same carrier.

**Trade-off, accepted explicitly:** RAUC's `compatible=` is now the generic
`openmower` (no per-board suffix), since one bundle now legitimately
installs on either board — it no longer acts as a hardware-mismatch flash
interlock the way `openmower-cm4`/`openmower-cm5` per-board strings would
have.

**Unverified, real-hardware-only risk items** (can't be resolved by code
review, flagging for whoever tests this on physical CM5 hardware first):

- **tryboot/EEPROM parity** — `autoboot.txt`'s `tryboot_a_b=1` and RAUC's
  whole rollback safety net assume CM5's EEPROM bootloader supports
  `tryboot` identically to CM4's. Treat "OTA update + simulated boot
  failure + confirmed rollback" on a real CM5 board as a hard gate, not
  just "it boots once."
- **`dtoverlay=` actually applying** — verify the fan overlay works on a
  real CM5 board.
- **CM5 console tty name** — `config.txt.default`'s `[cm5]` `cmdline=`
  override guesses `console=ttyAMA10,115200` (RP1 southbridge routes
  GPIO14/15 differently than CM4's direct SoC UART, so it's very unlikely
  to still be `ttyS0`). Confirm the real device name via HDMI (`tty1`,
  unaffected either way) on first real CM5 boot and fix
  `post-image.sh`/`config.txt.default` if wrong.
- **`/dev/mmcblk0` numbering** — `rauc/system.conf` and `cmdline-{a,b}.txt`
  hardcode `mmcblk0pN`; unconfirmed whether CM5's carrier/boot-media
  enumerates the same way on real hardware.

The migration installer (see [Migrating from stock Raspberry Pi
OS](#migrating-from-stock-raspberry-pi-os)) stays **CM4-only** for now —
not unified as part of this change.

## Building

Requires Docker. Used two ways: as the Buildroot toolchain container (as
before), and to fetch the `open_mower_ros` ROS install tree for arm64 on
the host before the Buildroot step (see [Running open_mower_ros](#running-open_mower_ros)).

```sh
make image          # -> output/images/sdcard.img + openmower-<version>.raucb
make menuconfig      # tweak Buildroot config
make savedefconfig   # persist config changes back to external/configs/
make shell           # shell inside the build container
```

`OPENMOWER_BUILD_SDCARD=0 ./build.sh image` skips assembling `sdcard.img`
(~8GB of pure disk-image writing the RAUC bundle doesn't need) and only
produces the bundle -- what CI (`.github/workflows/build-os.yaml`) uses for
ordinary PR/branch builds. It also vendors ROS from `OMR_SOURCE=local-image`
in CI (an image already `docker load`-ed from open_mower_ros' own "Build"
workflow, built from that exact commit -- not an unrelated ghcr tag) rather
than `local`/`ghcr`; see build.sh's own comments on all three. `v*` tag
pushes (same tags build-image.yaml itself reacts to) get the full
sdcard.img too, attached to a GitHub Release.

Dev signing keys are generated into `keys/` (gitignored) on first build.
All team members' dev builds are only installable on devices flashed with
the same `keys/dev-cert.pem`. Production: sign with an offline CA and put
its certificate into the image instead (`external/board/openmower/post-build.sh`).

`make image` automatically detects changes to enabled external packages that
use `SITE_METHOD = local` and runs Buildroot's corresponding `*-rebuild`
target before assembling the image. The source fingerprints are stored under
ignored `output/`; unchanged packages keep their normal incremental cache.

## Running open_mower_ros

Buildroot builds everything else here from source, but reimplementing ROS
Noetic and its full dependency graph (boost, opencv, pcl, tf2, the catkin
toolchain, ...) as hand-written Buildroot packages was out of scope. Instead,
`build.sh` fetches the repo's own `docker/Dockerfile`'s `assemble` stage as a
prebuilt arm64 Ubuntu-Focal filesystem tree (ROS Noetic + this workspace,
already built) **on the host**, and the `external/package/openmower-ros`
Buildroot package copies that tree wholesale into the target rootfs at
`/opt/openmower-root/`.

At runtime, `openmower.service` launches that tree via `systemd-nspawn`
(`BR2_PACKAGE_SYSTEMD_NSPAWN` — a lightweight container runtime built into
systemd, not a new dependency; no docker/podman on the device). It handles
`/dev`, `/tmp`, and DNS setup itself, `--user=openmower` resolves against
the container's own `/etc/passwd`, and `--bind=/data/openmower:/home/openmower`
gives it the persistent state dir. Docker on the host is also used at build
time to produce the tree in the first place (see below). Docker on the
*device* is scoped to the auxiliary stack only (Mosquitto, OpenMowerApp,
Dockge, ttyd) — see [Manage the auxiliary stack](#manage-the-auxiliary-stack)
below — deliberately not to open_mower_ros itself: that's a single
multi-GB payload already carried atomically by RAUC, and docker-compose
managing it too would mean pulling+unpacking a second copy into
`/var/lib/docker` on top of that, plus a second, independent update
mechanism alongside RAUC A/B. nspawn already runs the squashfs directly,
no image store, no copy.

Two ways to get that tree, picked by `OMR_SOURCE` (env var to `build.sh`/`make`):

- **`ghcr` (default)** — pull the prebuilt multi-arch image `open_mower_ros`'s
  own CI already publishes (`OMR_IMAGE`, defaults to
  `ghcr.io/clemenselflein/open_mower_ros:edge`) and extract it. Fast, no
  `linux/arm64` emulation needed (no code runs, just image layers pulled and
  unpacked).
- **`local`** — cross-build `docker/Dockerfile` locally via
  `docker buildx --platform linux/arm64` (needs `buildx` +
  `linux/arm64` emulation registered, i.e. binfmt_misc/qemu-user-static).
  Slow (full ROS Noetic + this workspace's dependency closure, emulated) —
  use this to test local `open_mower_ros` changes before they're pushed/built
  in CI.

```sh
make image                                     # default: pull from GHCR (:edge)
OMR_SOURCE=local make image                    # cross-build open_mower_ros locally instead
OMR_IMAGE=ghcr.io/clemenselflein/open_mower_ros:v2.1.0 make image   # pin a specific tag
```

Either way this is gated behind a content-hash key (`.cache/openmower-rootfs.key`)
so a normal `make image` doesn't pay for a re-pull/rebuild every time — only
when the resolved image digest (`ghcr`) or the repo's own HEAD commit
(`local`) actually changed.

### Configuration

`/data/openmower` is bind-mounted onto the ROS user's whole `$HOME`
(`/home/openmower`) inside the vendored tree — not just a curated subdir —
so it behaves like the original Docker container's writable home did
(pip/catkin caches etc. just work), and survives RAUC A/B updates.

- `/etc/openmower/openmower.conf` — safe/empty defaults, ships with the
  image, read-only. See its comments for the full picture.
- `/data/openmower/openmower.conf` — your actual per-device values
  (`HARDWARE_PLATFORM`, `MOWER`, `ESC_TYPE`, NTRIP/MQTT, ...), persists
  across updates, loaded second (overrides the file above).
- `/data/openmower/params/mower_params.yaml` — per-robot calibration.
  Required; `openmower.service` refuses to start without it (checked by
  `openmower-check-config`, logged to the journal, not a crash-loop). A
  generic-but-safe default (`enable_mower: false`)
  is copied here on first boot if missing, so that whenever you *do* start
  the service (not automatic — see below) it doesn't immediately fail on a
  missing file — edit it for your robot before relying on it.
- `DEBUG=1` (in either `.conf` file above) — full rosout logging instead of
  the default WARN-only console filter.
- `/data/openmower/ros/` and `/data/openmower/recordings/` — ROS's own log
  dir (`ROS_HOME`) and rosbag output (`RECORDINGS_PATH`), same "OS-level
  compose provides services" convention the lean Dockerfile was written for
  (see the repo's `docker-simulation/docker-compose.yaml`). All three
  set by `openmower-start`, overridable via the `.conf` files above.

### Debugging

`rostopic`/`rosnode`/etc. only exist inside `/opt/openmower-root`. Use:

```sh
openmower-shell
```

which boots that tree via `systemd-nspawn` (same as the real service — see
above) and drops you into a shell with the ROS environment already sourced.
Setup/teardown (`/dev`, `/tmp`, DNS, the `/data/openmower` bind) is handled
by nspawn itself, no manual mount dance. `rostopic list`/`rosnode list`/etc.
talk to the running `roscore` over `localhost:11311` same as from the real
service, since nspawn shares the host's network namespace by default (no
`--network-veth`/`--private-network` used).

`systemctl status openmower.service` / `journalctl -u openmower.service -f`
for service-level health — note `ROSCONSOLE_CONFIG_FILE` defaults to
`log4j.threshold=WARN`, so INFO-level node startup chatter is normal to not
see there.

### Internal LAN (CM4/CM5 ↔ xCore)

On v2 hardware (`HARDWARE_PLATFORM=2`, the documented default), the LL
board/xCore talks to `open_mower_ros` over IP, not serial — the compute
module's onboard ethernet is wired to the carrier board's internal switch
(bridging the xCore link and the external RJ45 jack on the same physical `eth0`).
`eth0` gets a static `172.16.78.1/24` address (`rootfs-overlay/etc/systemd/network/20-ethernet.network`,
coexists with whatever DHCP hands out if also plugged into a home network)
and `dnsmasq.service` serves DHCP to the xCore on that subnet
(`172.16.78.150-200`, see `/etc/dnsmasq.conf`) — DNS itself stays with
systemd-resolved (`port=0` in the dnsmasq config).

`10-classic-names.link` forces classic kernel interface naming
(`NamePolicy=kernel`), overriding systemd's default `onboard` naming policy
which otherwise renames the onboard ethernet away from `eth0`
(observed: `end0`) — every `eth0`-hardcoded file above would silently stop
matching anything without it. `wlan0` is unaffected either way (the
onboard-index heuristic only targets platform/PCI-bus devices with a
slot/index, not the SDIO wifi chip).

### Debug hardware access

`openocd` (`/etc/openocd/xcore.cfg`) flashes/debugs the xCore MCU via SWD
using the compute module's own GPIO as a bitbang adapter (`swclk`=GPIO27,
`swdio`=GPIO22) — `openocd -f /etc/openocd/xcore.cfg`. `/etc/openocd/xcore.cfg`
is a boot-time symlink (`openmower-detect-hardware`) to whichever of
`xcore-cm4.cfg` (`bcm2835gpio` driver, direct BCM2711 register access) or
`xcore-cm5.cfg` (`linuxgpiod` driver, kernel gpiochip device — CM5/BCM2712
dropped that direct register access) matches the board actually running —
see [Unified CM4+CM5 image](#unified-cm4cm5-image) above. Not verified
against real hardware yet — double check the pin mapping before relying on
it.

### open_mower_ros source

This directory (`os/`) lives *inside* the `open_mower_ros` checkout — it's
not a separate repo/submodule pointed at open_mower_ros, it's a subdirectory
of it. `OMR_SOURCE=local` (see above) therefore docker-builds the containing
checkout directly (`build.sh`'s `REPO_ROOT`, i.e. `os/..`), so local
open_mower_ros edits are picked up with no extra step — no submodule commit
to bump, no sibling checkout to keep in sync. `build.sh` already runs
`git submodule update --init --recursive`, which covers open_mower_ros' own
nested submodules (`xbot_driver_gps`, `xbot_framework`, `services`,
`ros_ntrip_client`) as well as `os/buildroot`.

## Manage the auxiliary stack

Docker + Compose run a small stack of third-party images alongside
open_mower_ros: Mosquitto (MQTT broker), OpenMowerApp (web dashboard),
Dockge (container-manager GUI), ttyd (web terminal). ROS itself is not
part of this stack (see above).

Only Dockge and ttyd start automatically. Mosquitto+OpenMowerApp do not —
same reasoning as `openmower.service` not auto-starting (see above):
nothing should come alive just because the device booted. Bring it up
explicitly once configured:

```sh
docker compose -f /opt/stacks/openmower/compose.yaml up -d
```

(`openmower-cli`'s `pull/start/stop/status` will wrap this once its
Compose-stack integration is wired up to this repo's layout — not yet.)
Docker's own `restart: unless-stopped` then keeps it up across reboots
once started, no systemd unit involved.

- **Dockge** (container GUI): `http://<hostname>:5001` — manages
  `/opt/stacks/*` (a symlink to `/data/stacks`, so edits persist across
  updates), including starting/stopping the openmower stack above via its
  UI instead of the CLI. Ships with the predecessor's own DB, byte-identical:
  auth disabled, no login needed (see "Known trade-offs").
- **OpenMowerApp**: `http://<hostname>:8080`, once started (see above).
- **WebTerminal (ttyd)**: `http://<hostname>:7681` — an unauthenticated (by
  default) root shell on the *host*, same as the predecessor; see
  `/data/stacks/webterminal/.env` (`TTYD_USER`/`TTYD_PASS`) to enable
  `--credential` auth in `compose.yaml` if this device is reachable beyond
  a trusted network.
- **Mosquitto**: `1883` (MQTT), `9001` (websockets), `allow_anonymous`
  ported as-is from the predecessor. To point open_mower_ros at it, set
  `OM_MQTT_HOSTNAME=localhost` in `/data/openmower/openmower.conf` (see its
  comments) — not on by default, matching the predecessor.
- **`openmower` CLI** (`ClemensElflein/openmower-cli`) is installed
  (`/usr/bin/openmower`, unmodified upstream zipapp) but its Compose-stack
  integration isn't wired up to this repo's layout yet — that's next.
  `openmower-shell` / `systemctl status openmower.service` / editing
  `/data/openmower/openmower.conf` directly still cover ROS management in
  the meantime.

## Flashing

The same `sdcard.img` flashes either board — see [Unified CM4+CM5
image](#unified-cm4cm5-image).

- **CM4/CM5 Lite (SD):** `dd if=output/images/sdcard.img of=/dev/sdX bs=4M conv=fsync`
- **CM4/CM5 eMMC:** put the IO board jumper into rpiboot mode, run `rpiboot`
  ([usbboot](https://github.com/raspberrypi/usbboot)), then `dd` to the
  exposed block device.

Serial console on `cmdline-{a,b}.txt`'s `console=ttyGS0,115200`: the USB0
composite gadget's own console function (see "USB device mode" — plug into
a PC, `openmower-usb-gadget-init.service` brings up a ConfigFS gadget with
ACM console + Improv-serial + ethernet), on every board, no separate
hardware needed.

GPIO14/15 (the hardware UART header pins) differ per board, permanently —
not a dev-vs-prod distinction. `[pi4]` wires them to real mower hardware
(a plain UART, no `console=`, no getty — see "Local boot-config overrides"
below). `[cm4]`/`[cm5]` dedicate them as their one and only UART instead,
always up as a boot/debug console: `config.txt.default`'s per-board
`cmdline=` directive points those boards at `cmdline-cm4.txt`/
`cmdline-cm5.txt` (generated by post-image.sh from the same per-slot
`cmdline-{a,b}.txt`, `console=ttyS0,115200` appended — `ttyS0`, the
mini-UART's real kernel device name, not the `serial0` alias stock
Raspberry Pi OS uses, same unverified-firmware-behavior caution as
"Unified CM4+CM5 image" above). No separate getty unit needed either way
(`systemd-getty-generator` auto-spawns `serial-getty@` for any `console=`
tty). CM5's actual console tty name is an unverified guess (`ttyAMA10`) —
see that file's own comment and the risk list below. Independent of the
USB gadget path entirely, which is the point: it's what you reach for when
ttyGS0/dwc2 itself is what's misbehaving.

Login over SSH/`openmower-shell`:
`root` / `openmower` (**development image only** — production must switch
to key-only auth).
`passwd` persists across reboots and RAUC A/B updates: `openmower-etc-overlay.service`
overlays the whole of `/etc` with a writable layer on `/data` at boot (`mkdir
-p`/`mount -t overlay`, `/data/.openmower-os/etc-overlay/{upper,work}`), so `passwd`
rewriting `/etc/shadow` (via its usual create-`shadow+`-then-rename dance)
actually persists instead of hitting `EROFS`. Not a build-time symlink or a
single-file bind-mount, deliberately — both look simpler but don't
actually work (see `post-build.sh`'s comment for why, including the exact
failure mode on real hardware). One side effect worth knowing: `/etc` as a
whole is now genuinely writable at runtime, so e.g. `systemctl enable`
persists too, not just `passwd`.

Root's `$HOME` persists too, differently: `/etc/passwd`'s root entry points
`HOME` straight at `/data/.openmower-os/root` (`post-build.sh`) rather than
`/root` — a `/root` → `/data/.openmower-os/root` symlink was the first
attempt, but collides with a generic buildroot device-table entry that
unconditionally expects a real directory there (`system/device_table.txt`,
unlike `/etc/dropbear` above, which has no such entry). Standard login
behavior (bash, sshd, dropbear) already sets `$HOME`/cwd from `pw_dir`, so
nothing else needed to change. `/etc/tmpfiles.d/home-root.conf` creates
`/data/.openmower-os/root` and seeds `.bashrc`/`.profile` from `/etc/skel`
on first boot only (on-device edits survive later boots/updates); it's
tucked under the hidden `.openmower-os` dir (see "`/data` layout" below)
rather than a bare `/data/root`, so it doesn't also show up as its own
entry in `ls /data` next to root's own `~`. Without this, anything writing under `$HOME`
— shell history, `ssh` `known_hosts`, and `openmower-cli`'s own shiv
extraction cache — silently
vanished every reboot, since `/root` used to be a plain directory on the
read-only squashfs. That's also why `openmower-cli`'s zipapp needs no
`SHIV_ROOT` override (or wrapper script at all) anymore: shiv's own default
extraction path, `$HOME/.shiv`, now just works.

## Migrating from stock Raspberry Pi OS

Devices already in the field running stock Raspberry Pi OS can move onto
this A/B/RAUC-managed OS entirely over the network — no SD card removal.
CM4-only for now (see [Unified CM4+CM5 image](#unified-cm4cm5-image) above).

```sh
make image            # -> kernel8.img, kernel_2712.img, both boards' *.dtb (under images/rpi-firmware/), config.vfat, boot-a.vfat, boot-b.vfat, rootfs.squashfs, sdcard.img
make image-migration  # -> output-migration/images/openmower-migrate-<version>.sh
```

`image-migration` produces a single self-extracting installer script -- but
deliberately a *small* one: just the CM4 kernel, both CM4 dtbs, and the
migration initramfs embedded and checksummed (a few tens of MB, rarely
changes). The actual OS -- `openmower-<version>.img.gz`, easily 1GB+ -- is
`make image`'s own output (`external/board/openmower/post-image.sh` gzips `sdcard.img`
and writes a matching `.sha256` sidecar next to it), meant to be uploaded as
a GitHub Releases asset. The installer downloads it straight from that
static URL over HTTPS at run time -- no manifest, no update server of our
own, just the URL and its sha256. That split is what lets the installer
script itself stay valid across every future OS release without needing to
be rebuilt or redistributed each time -- only rerun `make image-migration`
when the kernel/dtbs/migration initramfs change.

```sh
gh release upload v1.2.3 output/images/openmower-*.img.gz output/images/openmower-*.img.gz.sha256
```

Copy the installer to the device however's convenient (`scp`, USB stick,
...) and run it as root, pointing `--url` at that uploaded asset:

```sh
scp output-migration/images/openmower-migrate-*.sh pi@device:/tmp/
ssh pi@device sudo /tmp/openmower-migrate-*.sh --url https://github.com/<org>/<repo>/releases/download/v1.2.3/openmower-v1.2.3.img.gz --dry-run   # verify first: checks + downloads, no disk writes
ssh pi@device sudo /tmp/openmower-migrate-*.sh --url https://github.com/<org>/<repo>/releases/download/v1.2.3/openmower-v1.2.3.img.gz
```

The version is baked into the asset filename (same as the `.raucb`
bundle), so `--url` always names a specific release explicitly -- pick
whichever one you actually want on the device.

It's a real file on disk, not something to pipe into `sh` — it re-reads its
own embedded payload off disk partway through.

Before touching anything, it refuses to run unless it finds real evidence
this is actually an OpenMower device — OpenMowerOS itself, or (the stock
Raspberry Pi OS path above) OpenMower's own stack/home dir/CLI on the
system. Matching the Pi 4/CM4 board model alone isn't enough — that's true
of any Raspberry Pi, homeserver or NAS included, not just mowers. Past
that check, it prints a warning:
**back up your map/settings first**, and this migration is experimental
and may fail partway, in which case recovery needs physical access to the
mower (SD card out, reflash) — no remote recovery path. Requires typed
`yes` (or `--yes`) before proceeding. It then makes a best-effort attempt
to carry your data over on its own: downloads + sha256-verifies the OS
image from `--url` before touching anything else, stops the running
`openmower`/docker stack and prunes docker images (frees up disk space,
skipped under `--dry-run`), and stages `/home/openmower` plus your stack's
`.env` next to the downloaded OS image so the migration initramfs restores
them onto the new `/data/openmower` after writing it — non-fatal at every
step, a failure here never blocks the OS migration itself, it's still your
job to have backed up separately.

Mechanism: NOT kexec -- `kexec_load` is either disabled outright in stock
Raspberry Pi OS kernels, or hangs on real hardware even when enabled ("CPUs
are stuck in the kernel"): Raspberry Pi's boot firmware has no PSCI, so the
kernel has no way to park/re-wake secondary cores across the jump
(long-standing, unresolved upstream -- confirmed the hard way against real
CM4 hardware before switching approaches). Instead the script drops the
migration kernel/dtbs/initramfs/cmdline into a `migration/` subdirectory of the
*existing* stock boot partition (purely additive) plus a `tryboot.txt` at
its root (`os_prefix=migration/`), then triggers Raspberry Pi firmware's own
one-shot `tryboot` reboot -- a real cold reset, no PSCI involved. Firmware
loads `tryboot.txt` instead of `config.txt` for exactly one boot attempt;
the embedded busybox initramfs (`openmower_cm4_migration_defconfig`) then
stages the downloaded `sdcard.img.gz` into RAM and `dd`s it wholesale onto
the disk -- partition table included, so the layout below comes straight
from the image, nothing re-declared at migration time -- and reboots again —
firmware then boots the new slot A like any other device. See
`external/board/openmower-cm4-migration/rootfs-overlay/init` for exactly what
happens to the disk, and `scripts/migrate-to-openmower.sh`'s own header for
the safety checks (disk space, embedded-payload checksum, OS-image
checksum) it runs first.

Same safety net `rauc-mark-good` relies on elsewhere in this project: the
tryboot flag is one-shot and self-clears the moment it's consumed, so if
the migration boot never gets far enough to trigger its own
(ordinary) reboot — crash, hang, power loss — any subsequent reset boots
the stock OS again, automatically. And per Raspberry Pi's firmware
behavior, a misconfigured `tryboot.txt`/`migration/` (wrong filename, missing
file) just gets silently ignored, not hung. **The actual repartitioning is
still destructive and cannot be undone** — everything up to the explicit
"type yes to continue" prompt only stages files (reversibly: delete
`migration/` and `tryboot.txt` off the boot partition to fully undo), nothing
irreversible happens until the reboot after that prompt.

## Disk layout

| Part | Size | Content |
|---|---|---|
| p1 | 16M | `autoboot.txt` — tryboot A/B control (which slot is primary) |
| p2/p3 | 192M each | boot A/B: Pi firmware, both kernels, both boards' DTBs, per-slot `cmdline.txt` |
| p5/p6 | 3072M each | rootfs A/B (read-only squashfs, includes the vendored ROS tree). Measured squashfs on a real build: 936MiB — ~3.3x headroom for growth. |
| p7 | 64M **build-time floor** | `/data` — wifi credentials, OpenMower config/params, Docker stack files, plus OS-internal state (RAUC, dropbear, Docker's data-root, ...) — see "`/data` layout" below |

Built `sdcard.img` is ~6.5GiB (down from an earlier, unmeasured 12GB
budget — p5/p6 used to reserve 3584M each purely as a guess, p7 4096M).

**`/data` auto-expands on first boot** (`openmower-expand-data`, a systemd
oneshot: `sfdisk ", +"` to grow p7's partition-table entry to consume
whatever's left on the real disk, `partx -u` to make the running kernel see
it without unmounting, `resize2fs` to grow the filesystem) — the 64M
build-time floor is not the real budget on any actual device, just enough
for the pre-expand window (a few KB of keys/config). p7 **must stay the
last partition** for this to work — it only ever extends into
already-unpartitioned space past the last partition-table entry, never
moves/shrinks anything before it.

**Known limitation, accepted deliberately, not an oversight:** p5/p6's
3072M budget is sized for headroom, not for what any given device's `/data`
can necessarily match. On an 8GB eMMC (~7.4GiB real usable), `/data` after
full expand settles at only ~1GiB — comfortably enough for today's ~955MiB
bundles, but if actual rootfs usage ever grows to fill the full 3072M
budget, an 8GB device won't have room to stage a same-size update and stops
being updatable. No `/data` floor size changes this — it's a function of
total media size vs. the rootfs budget, not of anything picked at build
time. 8GB eMMC is treated as a bounded/legacy tier that may eventually
reach end-of-updates as the OS grows; 16GB+ media keep full update headroom
throughout (16GB → ~8.5GiB `/data`, 32GB → ~23GiB).

### `/data` layout

`ls /data` on a running device shows only what you might actually want to
look at:

- `openmower/` — your config, params, and ROS's own recordings/logs (see
  "Configuration" above)
- `boot/usercfg.txt` — local Pi firmware overrides (see "Local boot-config
  overrides" above)
- `stacks/`, `dockge/` — the auxiliary Docker stack's compose files (see
  "Manage OpenMower stack" above)
- `wifi/` — the Wi-Fi credentials `wpa_supplicant` reads (see "Wi-Fi
  provisioning" above)

Everything else the OS itself needs to persist but a user never needs to
browse or edit by hand -- RAUC's A/B state, dropbear's host keys, root's
own `$HOME`, Docker's data-root (image/container storage, as opposed to
the stacks/dockge compose files above), the Bluetooth pairing DB, the
`/etc` overlay's upper/work dirs, pending-update staging -- lives under
one hidden `.openmower-os/` directory instead of cluttering the top level.

`lost+found/` is also still there, unavoidably: that's ext2/3/4's own
`e2fsck` convention (a well-known filename `e2fsck` expects and recreates
at the filesystem's root if missing, for orphaned-inode recovery after an
unclean shutdown), not something this OS controls or can relocate/hide.

## Update flow

`openmower-check-update.timer` polls api.openmower.de once a day for a newer
release and records the result to `/data/openmower/cli/os_update_status.json`
(the CLI warns about it on next use) -- but updates are still installed on
demand, never automatically: run `openmower update-os` (openmower-cli;
`--from-pr`/`--from-branch`/`--tag` select the build, see its own `--help`)
on the device, or manually:

1. Fetch a `.raucb` bundle (carries the full vendored ROS tree along with
   the base OS, so multi-GB) and `rauc install` it into the inactive slot
   pair -- RAUC verifies the bundle signature as part of install.
2. Reboot with the one-shot `tryboot` flag ⇒ firmware boots the new slot
   **once**: `printf '0 tryboot' > /run/systemd/reboot-param && systemctl reboot`.
3. Reaching `multi-user.target` runs `rauc-mark-good.service` ⇒ the
   tryboot backend commits the new slot as primary in `autoboot.txt`.
4. Any crash/hang before that (hardware watchdog, 15 s) resets the SoC ⇒
   tryboot flag is gone ⇒ firmware boots the old slot. Rollback is
   firmware-guaranteed, no software has to run for it.

## Wifi provisioning

While `/data/wifi/wpa_supplicant-wlan0.conf` does not exist, `improv-ble`
advertises the Improv Wi-Fi service over BLE; provision from the Home
Assistant app or any Improv client. Credentials
land on `/data`, so they survive updates. To re-provision:
`rm /data/wifi/wpa_supplicant-wlan0.conf && reboot`.

Note: Improv sends credentials over unencrypted BLE GATT — fine for
onboarding at home, revisit (BLE pairing or app-layer crypto) before
shipping.

BLE is the *only* provisioning path — deliberately, see "Known trade-offs".
There is no AP/captive-portal fallback here. If no BLE-capable client is available,
provision over Ethernet (`ssh root@<hostname or IP>`, edit
`/data/wifi/wpa_supplicant-wlan0.conf` by hand) -- GPIO14/15 is a plain
UART wired to real mower hardware, not a debug console (see "Local
boot-config overrides" below), so there's no serial fallback either.

## Local boot-config overrides

`config.txt` (Pi firmware config, see `external/board/openmower/rootfs-overlay/etc/openmower/config.txt.default`)
is baked into every image and OTA bundle, overwritten wholesale on each
update -- there is no runtime mount of it at all otherwise (root is
squashfs; the boot partitions aren't mounted anywhere in the running OS).
It only holds the pre-Linux essentials itself, though: the actual OpenMower
hardware config (antenna selection, UART setup, fan control) lives entirely
in `usercfg.txt.default` instead, single-sourced from there -- `post-build.sh` appends that file's
content onto config.txt at build time (plus a trailing `include usercfg.txt`
it adds itself), so those defaults are baked in and active from the very
first boot of a fresh flash. config.txt itself is never copied onto
`/data` -- the effective, fully-assembled config.txt (base + baked
defaults + include) is readable straight from the rootfs at
`/etc/openmower/config.txt.default`, identical to what's actually on the
boot partition, so there's nothing to duplicate or accidentally edit
without effect on `/data`.

`/data/boot/usercfg.txt` is the actual escape hatch: it lives on `/data`
instead of the boot partition, so unlike config.txt it survives updates.
Seeded on first boot from the same `/etc/openmower/usercfg.txt.default` that
got baked into config.txt -- not an empty template, it ships with the real
live defaults (see its own comments, e.g. for switching the CM4/CM5 antenna
from internal to external). Because config.txt's baked copy comes first and
this file is `include`d after it, an edit here overrides the baked default
for the same setting -- normal config.txt semantics, later directives win.

Edit it directly on a running device (`/data/boot/usercfg.txt`), then
reboot -- no race with "edit, then immediately reboot": editing it triggers
`openmower-sync-bootcfg.path` (an inotify watch on that exact file), which
applies the edit to the active boot partition right away, in the
background, well before you'd get to typing `reboot` yourself. Worst case
(edit lands mid-sync) the boot after that picks it up instead, since
config.txt/usercfg.txt are read by the Pi firmware before Linux even
starts. `openmower-sync-bootcfg.service` also runs unconditionally on every
boot regardless (self-healing fallback, e.g. after a fresh flash before any
edit/OTA has happened at all), and OTA updates re-apply it to the freshly
installed *other* slot's boot partition too, before that slot is ever
booted (see `rauc-hook.sh`), so it isn't lost on the next update either.

## Layout

This directory (`os/`) lives inside the `open_mower_ros` repo, alongside
`src/` etc. — see [open_mower_ros source](#open_mower_ros-source) above.

```
docker/           build container (Buildroot toolchain only)
buildroot/        Buildroot submodule (2026.02 LTS)
external/         BR2_EXTERNAL tree: defconfigs, board files, own packages
  configs/openmower_defconfig            main build (rootfs/userland, no kernel)
  configs/openmower_dev_defconfig        same, + CLion remote-debug tooling
  configs/openmower_kernel-cm4_defconfig kernel-only satellite, CM4/bcm2711
  configs/openmower_kernel-cm5_defconfig kernel-only satellite, CM5/bcm2712
  configs/openmower_cm4_migration_defconfig  CM4-only migration installer, see "Migrating..."
  board/openmower/            partition layout, tryboot + RAUC integration, overlay
  board/openmower-kernel/     kernel config fragments shared by both kernel satellites
  board/openmower-cm4-migration/  migration installer initramfs (CM4-only)
  package/improv-ble/        BLE provisioning daemon
  package/openmower-ros/     vendors the docker-buildx-built ROS tree + runtime unit
  package/openmower-cli/     fetches the openmower-cli zipapp release (see "Manage the auxiliary stack")
keys/             dev signing keys (gitignored, auto-generated)
```

## Known trade-offs (current state)

- 8GB eMMC devices have a bounded update lifetime — see [Disk
  layout](#disk-layout)'s last paragraph. Not fixable by any `/data`
  build-time floor size; a function of total media vs. the rootfs budget.
- `machine-id` is persisted on `/data`. Journal is in RAM.
- mark-good only requires reaching `multi-user.target`; it does not yet
  require network/streaming health, nor does it check that
  `openmower.service` actually came up -- it's disabled by default (see
  "Manage the auxiliary stack" / "Running open_mower_ros"), and even when
  enabled its `ExecCondition=` can legitimately skip it on an unconfigured
  device.
- Every OTA update ships the full ROS/Ubuntu payload alongside the base OS —
  there's no separate, independently-versioned update channel for just the
  ROS side. Simpler and matches RAUC's atomic A/B model, at the cost of
  bigger bundles and doubled on-SD footprint across both A/B slots.
- Python-based improv daemon costs ~30 MB rootfs; C rewrite is a later
  size optimization.
- Bluetooth is deliberately kept enabled (`config.txt` does NOT set
  `dtoverlay=disable-bt`) because `improv-ble` needs it for BLE wifi
  provisioning. `uart2-5` already provide four independent hardware UARTs
  for LL board/xESC/GPS comms, so this shouldn't cost any usable serial
  capacity in practice.
- No Raspberry Pi Imager first-boot customization (no `userconf`/`firstrun`/
  cloud-init anywhere in this repo). Hostname and password both default the same on every device at flash
  time (see below) and Wi-Fi has no preseed path — only BLE provisioning
  post-boot. Flashing is `dd` of a prebuilt image; deliberately not
  pursuing Imager compatibility for now, so there's no way to set any of
  these to a per-device value *before* first boot, only after.
- Hostname defaults to `openmower` on every device, same as the password
  (`/etc/passwd` default, "Flashing" above) — same-network collisions
  (mDNS, `ssh root@openmower`, the URLs under "Manage the auxiliary stack")
  if you run more than one. Both persist on `/data` and are changeable
  post-boot (`/data/hostname` + `openmower-set-hostname` to apply
  immediately, or `passwd`) — just not before first boot, and not from a
  fleet-management tool yet, one device at a time by hand.
- No AP/captive-portal Wi-Fi fallback — see "Wifi provisioning" above.
- Mosquitto (`allow_anonymous`), ttyd's host root shell (`--credential` auth
  off by default), and Dockge (pre-baked DB, `disableAuth=true`, one fixed
  `jwtSecret`) are all reachable, unauthenticated, from wherever `eth0`/Wi-Fi
  is plugged in. Same `jwtSecret` on every device means anyone who can reach one
  device's Dockge and gets hold of it can forge a valid session against
  any other. Keep these off untrusted networks. See "Manage the auxiliary
  stack".
