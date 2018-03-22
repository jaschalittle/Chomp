CPPFLAGS=-I../chomp -I.
CXXFLAGS=-std=c++11

TEST_COMMON_SRCS=micros.cpp cosmos_listener.cpp
TEST_PIDSTEER_SRCS=$(TEST_COMMON_SRCS) test_pidsteer.cpp ../chomp/chump_targeting.cpp
TEST_PIDSTEER_OBJS=$(TEST_PIDSTEER_SRCS:.cpp=.o)

test_pidsteer: $(TEST_PIDSTEER_OBJS)
	g++ -o $@ $^

chump_targeting.o: chump_targeting.h
