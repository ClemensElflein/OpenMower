# Host-side convenience wrapper. Real work happens in the container (build.sh).

.PHONY: image image-dev image-migration shell menuconfig menuconfig-dev \
	menuconfig-migration linux-menuconfig \
	savedefconfig savedefconfig-dev savedefconfig-migration clean distclean

image:
	./build.sh image

# Dev variant: gdbserver + host gdb + openssh, see external/configs/openmower_cm4_dev_defconfig.
# Shares the same output/ dir as prod; switch back with `make image`.
image-dev:
	DEFCONFIG=openmower_cm4_dev_defconfig ./build.sh image

# Migration installer, see external/configs/openmower_cm4_migration_defconfig
# and scripts/migrate-to-openmower.sh. Needs a prior `make image` (pulls the
# kernel/dtbs from output/images/); own output-migration/ dir otherwise.
# Result: output-migration/images/openmower-migrate-<version>.sh
image-migration:
	./build.sh image-migration

shell:
	./build.sh shell

menuconfig:
	./build.sh menuconfig

menuconfig-dev:
	DEFCONFIG=openmower_cm4_dev_defconfig ./build.sh menuconfig

menuconfig-migration:
	./build.sh menuconfig-migration

linux-menuconfig:
	./build.sh linux-menuconfig

# Write the current .config back to external/configs/openmower_cm4_defconfig
savedefconfig:
	./build.sh savedefconfig BR2_DEFCONFIG=/work/os/external/configs/openmower_cm4_defconfig

# Write the current .config back to external/configs/openmower_cm4_dev_defconfig
savedefconfig-dev:
	./build.sh savedefconfig BR2_DEFCONFIG=/work/os/external/configs/openmower_cm4_dev_defconfig

# Write the current .config back to external/configs/openmower_cm4_migration_defconfig
savedefconfig-migration:
	./build.sh savedefconfig-migration

clean:
	./build.sh clean

distclean:
	rm -rf output output-migration
