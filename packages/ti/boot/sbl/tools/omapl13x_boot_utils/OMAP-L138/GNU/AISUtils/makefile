DIRS:=$(wildcard *)

all:
%::
		@for i in $(DIRS); do \
				if [ -d $${i} ]; then if [ -e $${i}/makefile ]; then make -C $${i} $@; fi; fi; \
		done
		