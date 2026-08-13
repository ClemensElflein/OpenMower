# Host-side convenience wrapper. Real work happens in the container (build.sh).

.PHONY: image image-migration shell menuconfig \
	menuconfig-migration menuconfig-kernel-cm4 menuconfig-kernel-cm5 \
	linux-menuconfig savedefconfig savedefconfig-migration \
	savedefconfig-kernel-cm4 savedefconfig-kernel-cm5 clean distclean

# Unified CM4+CM5 image -- also builds the two kernel-only satellites
# (openmower_kernel-cm4_defconfig/openmower_kernel-cm5_defconfig) along the
# way, see build.sh and os/README.md.
image:
	./build.sh image

# Migration installer, see external/configs/openmower_cm4_migration_defconfig
# and scripts/migrate-to-openmower.sh. Needs a prior `make image` (pulls the
# kernel/dtbs from output/images/rpi-firmware/); own output-migration/ dir
# otherwise. CM4-only for now. Result: output-migration/images/openmower-migrate-<version>.sh
image-migration:
	./build.sh image-migration

shell:
	./build.sh shell

menuconfig:
	./build.sh menuconfig

menuconfig-migration:
	./build.sh menuconfig-migration

# Kernel-only satellite builds -- see openmower_kernel-cm4_defconfig/
# openmower_kernel-cm5_defconfig and build.sh's header comment.
menuconfig-kernel-cm4:
	./build.sh menuconfig-kernel-cm4

menuconfig-kernel-cm5:
	./build.sh menuconfig-kernel-cm5

linux-menuconfig:
	./build.sh linux-menuconfig

# Write the current .config back to external/configs/openmower_defconfig
savedefconfig:
	./build.sh savedefconfig BR2_DEFCONFIG=/work/os/external/configs/openmower_defconfig

# Write the current .config back to external/configs/openmower_cm4_migration_defconfig
savedefconfig-migration:
	./build.sh savedefconfig-migration

# Write the current .config back to the respective kernel-only satellite defconfig
savedefconfig-kernel-cm4:
	./build.sh savedefconfig-kernel-cm4

savedefconfig-kernel-cm5:
	./build.sh savedefconfig-kernel-cm5

clean:
	./build.sh clean

distclean:
	rm -rf output output-migration output-kernel-cm4 output-kernel-cm5
