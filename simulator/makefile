ALL: harness-static

harness-%: harness-%.cc motor.cc motor.h
	g++ -O3 -g motor.cc $< -o $@
