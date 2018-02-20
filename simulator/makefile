ALL: harness-static test-tsin

LIB_FILES = motor.cc motor.h

test-%: test-%.cc $(LIB_FILES)
	g++ -O2 -g motor.cc $< -o $@

harness-%: harness-%.cc $(LIB_FILES)
	g++ -O3 -g motor.cc $< -o $@
