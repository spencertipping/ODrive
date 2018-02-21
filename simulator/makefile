ALL: harness-static test-tsin

LIB_FILES = board.h motor.h makefile
CC_OPTS = -DDEBUG -O2 -g -std=c++11 -Wall -Wno-parentheses

test-%: test-%.cc $(LIB_FILES)
	g++ $(CC_OPTS) $< -o $@

harness-%: harness-%.cc $(LIB_FILES)
	g++ $(CC_OPTS) $< -o $@
