# -*- makefile -*-
#----------------------------------------------------------------------------
#       GNU ACE Workspace
#
# $Id: GNUACEWorkspaceCreator.pm 94635 2011-10-06 12:59:23Z johnnyw $
#
# This file was generated by MPC.  Any changes made directly to
# this file will be lost the next time it is generated.
#
# MPC Command:
# /home/lsa/ace/ACE_wrappers/bin/mwc.pl -type gnuace -features vrep=0,tests=0 workspace.mwc
#
#----------------------------------------------------------------------------

MAKEFILE = GNUmakefile

ifeq ($(findstring k,$(MAKEFLAGS)),k)
  KEEP_GOING = -
endif

include $(ACE_ROOT)/include/makeinclude/macros.GNU

all: custom_controller

depend: custom_controller-depend

REMAINING_TARGETS := $(filter-out all,$(TARGETS_NESTED:.nested=)) $(CUSTOM_TARGETS)

$(REMAINING_TARGETS):
	$(KEEP_GOING)@$(MAKE) -f GNUmakefile.custom_controller $(@)

.PHONY: custom_controller
custom_controller:
	$(KEEP_GOING)@$(MAKE) -f GNUmakefile.custom_controller all

.PHONY: custom_controller-depend
custom_controller-depend:
	$(KEEP_GOING)@$(MAKE) -f GNUmakefile.custom_controller depend

project_name_list:
	@echo custom_controller
