CXX = g++
CXXFLAGS = -I. -I./mocks -I.. -g -std=c++14
LDFLAGS = -lpthread

TESTS = test_md5 test_utilities

all: $(TESTS)

test_md5: test_md5.o ../src/misc/MD5.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test_utilities: test_utilities.o mocks/Arduino.o mocks/EEPROM.o ../src/misc/MD5.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

../src/misc/MD5.o: ../src/misc/MD5.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f *.o mocks/*.o ../src/misc/*.o $(TESTS)

run: $(TESTS)
	./test_md5
	./test_utilities
