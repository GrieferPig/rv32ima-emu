################################################################################
#
# fbdoom
#
################################################################################

FBDOOM_VERSION = 6c599f50e9e8e9436a5c064f42836eb48ff6bde0
FBDOOM_SITE = $(BR2_EXTERNAL_rv32ima_overlay_PATH)/package/fbdoom/src
FBDOOM_SITE_METHOD = local
FBDOOM_LICENSE = Doom Source License, GPL-2.0+
FBDOOM_LICENSE_FILES = README.TXT
FBDOOM_SUBDIR = fbdoom

# Build the framebuffer-only port to avoid SDL and keep dependencies minimal.
define FBDOOM_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D)/$(FBDOOM_SUBDIR) \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		CFLAGS="$(TARGET_CFLAGS)" \
		LDFLAGS="$(TARGET_LDFLAGS)" \
		NOSDL=1
endef

define FBDOOM_INSTALL_TARGET_CMDS
	$(INSTALL) -m 0755 -D $(@D)/$(FBDOOM_SUBDIR)/fbdoom $(TARGET_DIR)/fbdoom
endef

$(eval $(generic-package))
