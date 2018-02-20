ALL: harness-static test-tsin

LIB_FILES = motor.cc motor.h makefile

test-%: test-%.cc $(LIB_FILES)
	g++ -g motor.cc $< -o $@

harness-%: harness-%.cc $(LIB_FILES)
	g++ -g motor.cc $< -o $@
