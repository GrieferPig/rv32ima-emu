BUILDROOT_DIR := buildroot
OVERLAY_DIR := buildroot_overlay
IMAGES_DIR := ../build
DTS_FILE := ../rv32ima.dts
BOOT_IMAGE := $(IMAGES_DIR)/linux-rv32ima-boot.bin
FDT_OFFSET := 8388608  # 8 MiB

.PHONY: all defconfig build images clean

all: images

defconfig: $(BUILDROOT_DIR)
	make -C $(BUILDROOT_DIR) BR2_EXTERNAL=../$(OVERLAY_DIR) rv32ima_defconfig

$(BUILDROOT_DIR):
	@echo "Error: buildroot directory not found. Run:" >&2
	@echo "  wget https://buildroot.org/downloads/buildroot-2025.02.tar.gz" >&2
	@echo "  tar -xzf buildroot-2025.02.tar.gz && mv buildroot-2025.02 buildroot" >&2
	@exit 1

build: defconfig
	make -C $(BUILDROOT_DIR)

$(IMAGES_DIR)/dtb: $(DTS_FILE)
	mkdir -p $(IMAGES_DIR)
	$(BUILDROOT_DIR)/output/host/bin/dtc -I dts -O dtb -o $@ $<

images: build $(IMAGES_DIR)/dtb
	mkdir -p $(IMAGES_DIR)
	# Create combined boot image: kernel at offset 0, DTB at 8MiB
	cp $(BUILDROOT_DIR)/output/images/Image $(BOOT_IMAGE)
	truncate -s $(FDT_OFFSET) $(BOOT_IMAGE)
	cat $(IMAGES_DIR)/dtb >> $(BOOT_IMAGE)
	@echo ""
	@echo "Boot image created: $(BOOT_IMAGE)"
	@echo "Run with: ./emulator $(BOOT_IMAGE)"

clean:
	-make -C $(BUILDROOT_DIR) clean
	rm -rf $(IMAGES_DIR)/linux-rv32ima-boot.bin $(IMAGES_DIR)/dtb
