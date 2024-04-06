CXX=g++
CXXFLAGS=-Wall -Wextra `pkg-config -cflags raylib bullet`
LDFLAGS=`pkg-config --libs raylib bullet`

all: randice

randice: main.o randice.o
	$(CXX) -o randice main.o randice.o $(LDFLAGS)

main.o: main.cpp randice.o
	$(CXX) $(CXXFLAGS) -c -o main.o main.cpp

randice.o: randice.cpp
	$(CXX) $(CXXFLAGS) -c -o randice.o randice.cpp

clean:
	rm -rf randice
	rm -rf main.o
	rm -rf randice.o
