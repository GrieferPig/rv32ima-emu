################################################################################
#
# freedoom1-wad
#
################################################################################

FREEDOOM1_WAD_VERSION = 0.13.0
FREEDOOM1_WAD_SOURCE = freedoom-$(FREEDOOM1_WAD_VERSION).zip
FREEDOOM1_WAD_SITE = https://github.com/freedoom/freedoom/releases/download/v$(FREEDOOM1_WAD_VERSION)

# Extract only the IWAD we want into the root of the package build directory.
define FREEDOOM1_WAD_EXTRACT_CMDS
	$(UNZIP) -p $(FREEDOOM1_WAD_DL_DIR)/$($(PKG)_SOURCE) \
		freedoom-$(FREEDOOM1_WAD_VERSION)/freedoom1.wad > $(@D)/freedoom1.wad
endef

define FREEDOOM1_WAD_INSTALL_TARGET_CMDS
	$(INSTALL) -m 0644 -D $(@D)/freedoom1.wad $(TARGET_DIR)/freedoom1.wad
endef

$(eval $(generic-package))
