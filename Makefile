lib.name = simple-del

class.sources = src/simple_delwrite~.c src/simple_delread~.c src/delay~.c src/delay1~.c src/delay1_cubic~.c src/delay2~.c src/multitap~.c

PDLIBBUILDER_DIR=pd-lib-builder/
include ${PDLIBBUILDER_DIR}/Makefile.pdlibbuilder
