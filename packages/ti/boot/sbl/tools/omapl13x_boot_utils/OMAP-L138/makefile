DIRS:=$(wildcard *)

all:
%::
# Make sure GNU stuff gets built first
		make -C GNU $@
#	    make -C CCS $@
#		@for i in $(DIRS); do \
#				if [ -d $${i} ]; then if [ -e $${i}/makefile ]; then make -C $${i} $@; fi; fi; \
#		done
