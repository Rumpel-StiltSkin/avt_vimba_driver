include $(CURDIR)/Common.mk
include $(CURDIR)/VimbaC.mk

#Compile options needed for VimbaCPP
VIMBACPP_CFLAGS = -I$(VIMBASDK_DIR) $(VIMBAC_CFLAGS)

#Linker options needed for VimbaCPP
VIMBACPP_LIBS   = -L$(BIN_DIR) -lVimbaCPP $(VIMBAC_LIBS) -Wl,-rpath-link,$(BIN_DIR)

#By default we copy libVimbaCPP.so next to the binary
$(BIN_DIR)/libVimbaCPP.so: $(BIN_DIR) VimbaC
	$(CP) ../../lib/$(ARCH)_$(WORDSIZE)bit/libVimbaCPP.so $(BIN_DIR)/

#Operations we have to do in order to prepare VimbaCPP
VimbaCPP: $(BIN_DIR)/libVimbaCPP.so
