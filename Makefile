#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
BUILD_IOCS=YES
DIRS := $(DIRS) configure
DIRS := $(DIRS) mythenApp
etc_DEPENDS_DIRS += mythenApp
ifeq ($(BUILD_IOCS), YES)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
iocs_DEPEND_DIRS += mythenApp
endif
include $(TOP)/configure/RULES_TOP

uninstall: uninstall_iocs
uninstall_iocs:
	$(MAKE) -C iocs uninstall
.PHONY: uninstall uninstall_iocs

realuninstall: realuninstall_iocs
realuninstall_iocs:
	$(MAKE) -C iocs realuninstall
.PHONY: realuninstall realuninstall_iocs

