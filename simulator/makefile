OUTPUTS = harness-static harness-generator harness-zero \
          test-tsin test-backemf

all: $(OUTPUTS)

LIB_FILES = board.h motor.h makefile
CC_OPTS = -DDEBUG -O2 -g -std=c++11 -Wall -Wno-parentheses

.PHONY: clean
clean:
	rm $(OUTPUTS)

test-%: test-%.cc $(LIB_FILES)
	g++ $(CC_OPTS) $< -o $@

harness-%: harness-%.cc $(LIB_FILES)
	g++ $(CC_OPTS) $< -o $@
