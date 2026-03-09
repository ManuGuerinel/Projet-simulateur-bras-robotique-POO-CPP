# Makefile pour simulateur de bras robotique
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude $(shell pkg-config --cflags pinocchio eigen3)
LDFLAGS = $(shell pkg-config --libs pinocchio eigen3)

SRC = $(wildcard src/*.cpp)
OBJ = $(SRC:.cpp=.o)
EXEC = main

.PHONY: all clean run

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(EXEC)
	./$(EXEC)

clean:
	rm -f src/*.o $(EXEC)