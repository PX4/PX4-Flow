#
# Built products
#
DESIRED_FIRMWARES 	 = $(foreach config,$(CONFIGS),$(IMAGE_DIR)$(config).px4)
STAGED_FIRMWARES	 = $(foreach config,$(KNOWN_CONFIGS),$(IMAGE_DIR)$(config).px4)
FIRMWARES		 = $(foreach config,$(KNOWN_CONFIGS),$(BUILD_DIR)$(config).build/firmware.px4)

all:	$(DESIRED_FIRMWARES)

#
# Copy FIRMWARES into the image directory.
#
# XXX copying the .bin files is a hack to work around the PX4IO uploader
#     not supporting .px4 files, and it should be deprecated onced that
#     is taken care of.
#
$(STAGED_FIRMWARES): $(IMAGE_DIR)%.px4: $(BUILD_DIR)%.build/firmware.px4
	@$(ECHO) %% Copying $@
	$(Q) $(COPY) $< $@
	$(Q) $(COPY) $(patsubst %.px4,%.bin,$<) $(patsubst %.px4,%.bin,$@)

#
# Generate FIRMWARES.
#
.PHONY: $(FIRMWARES)
$(BUILD_DIR)%.build/firmware.px4: config   = $(patsubst $(BUILD_DIR)%.build/firmware.px4,%,$@)
$(BUILD_DIR)%.build/firmware.px4: BAREMETAL_CONFIG = $(if $(findstring bootloader,$@),bootloader,baremetal)
$(BUILD_DIR)%.build/firmware.px4: work_dir = $(BUILD_DIR)$(config).build/
$(FIRMWARES): $(BUILD_DIR)%.build/firmware.px4:	checkgitversion checksubmodules
	@$(ECHO) %%%%
	@$(ECHO) %%%% Building $(config) on $(PX4_TARGET_OS) in $(work_dir)
	@$(ECHO) %%%% 
	$(Q) $(MKDIR) -p $(work_dir)
	$(Q) $(MAKE) -r -C $(work_dir) \
		-f $(PX4_MK_DIR)firmware.mk \
		CONFIG=$(config) \
		WORK_DIR=$(work_dir) \
		BAREMETAL_CONFIG=$(BAREMETAL_CONFIG)  \
		$(FIRMWARE_GOAL)


#
# Build the Baremetal export archives.
#
# Note that there are no explicit dependencies extended from these
# archives. If Baremetal is updated, the user is expected to rebuild the
# archives/build area manually. Likewise, when the 'archives' target is
# invoked, all archives are always rebuilt.
#
# XXX Should support fetching/unpacking from a separate directory to permit
#     downloads of the prebuilt archives as well...
#
BAREMETTAL_BOOTLOADER_ARCHIVES  = $(foreach board,$(BOARDS_WITH_BOOTLOADERS),$(ARCHIVE_DIR)$(board).bootloader.export)
BAREMETTAL_ARCHIVES		 = $(BAREMETTAL_BOOTLOADER_ARCHIVES) $(foreach board,$(BOARDS),$(ARCHIVE_DIR)$(board).$(PX4_TARGET_OS).export)
.PHONY:			archives
archives:		checksubmodules $(BAREMETTAL_ARCHIVES)

# We cannot build these parallel; note that we also force -j1 for the
# sub-make invocations.
ifneq ($(filter archives,$(MAKECMDGOALS)),)
.NOTPARALLEL:
endif

J?=1

$(ARCHIVE_DIR)%.export:	board = $(basename $(notdir $(basename $@)))
$(ARCHIVE_DIR)%.export:	configuration = $(PX4_TARGET_OS)
$(BAREMETTAL_ARCHIVES): $(ARCHIVE_DIR)%.export: $(BAREMETTAL_SRC)
	@$(ECHO) %% Configuring $(board) On $(configuration)
	$(Q) $(MKDIR) -p $(BUILD_DIR)
	$(Q) (cd $(BUILD_DIR) && $(RMDIR) baremetal-configs)
	$(Q) $(MKDIR) -p $(BUILD_DIR)baremetal-configs/baremetal-export/build
	$(Q) $(COPY) $(PX4_BASE)baremetal-configs/$(board)/scripts/* $(BUILD_DIR)baremetal-configs/baremetal-export/build
	@$(ECHO) %% Exporting Baremetal for $(board) $(configuration) 
	$(Q) $(MKDIR) -p $(dir $@)
	$(Q) (cd $(BUILD_DIR)baremetal-configs && $(ZIP_CMD) -q -r $(BUILD_DIR)baremetal-configs/baremetal-export.zip  baremetal-export) 
	$(Q) $(COPY) $(BUILD_DIR)baremetal-configs/baremetal-export.zip $@
	$(Q) (cd $(BUILD_DIR) && $(RMDIR) baremetal-configs)

$(BAREMETTAL_SRC): checkgitversion checksubmodules

$(UAVCAN_DIR):
	$(Q) (./Tools/check_submodules.sh)

