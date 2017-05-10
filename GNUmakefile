# Library Directories
LIBDIRS += -L /usr/lib -L /usr/local/lib/

# Static Libraries
STATICLIBS += -lEposCmd

# Include Directories
#INCLUDEDIRS = -I./

all:: motor_control

motor_control: src/motor_control.cc src/cmaxonmotor.cc include/cmaxonmotor.h 
	g++ $(INCLUDEDIRS) src/cmaxonmotor.cc -ldl -o $@ $< -lm $(LIBDIRS) $(STATICLIBS) -std=c++11

clean::
	rm -f build/*.o motor_control 

.PHONY: clean
