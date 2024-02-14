CXX=g++
CXXFLAGS=-Wall -Wextra
LDFLAGS=`pkg-config --libs --cflags raylib bullet`

all: randice randice_debug

randice: randice.cpp
	$(CXX) $(CXXFLAGS) -o randice randice.cpp $(LDFLAGS)

randice_debug: randice.cpp
	$(CXX) $(CXXFLAGS) -o randice_debug randice.cpp $(LDFLAGS) -ggdb

clean:
	rm -rf randice
	rm -rf randice_debug
