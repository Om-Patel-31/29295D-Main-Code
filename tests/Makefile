# Desktop-only C++11 test build; it links the VEX-free control modules.
CXX ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -pedantic -I../include
# Default target builds and runs the complete control test executable.
all: test
test: test_control
# The target intentionally excludes robot.cpp and all VEX hardware APIs.
test_control: test_control.cpp ../src/ladrc2.cpp ../src/scurve.cpp ../src/bezier.cpp ../src/odometry.cpp ../src/drive.cpp
	$(CXX) $(CXXFLAGS) -Werror $^ -o $@
	./test_control
# Remove only the generated desktop test executable.
clean:
	rm -f test_control
