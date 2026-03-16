# --- Variables de compilation ---
CXX = g++

CXXFLAGS = -std=c++17 -Wall -Wextra -O2 $(shell pkg-config --cflags pinocchio eigen3)
LDFLAGS = $(shell pkg-config --libs pinocchio eigen3)
GTEST_LIBS = -lgtest -lgtest_main -pthread


OBJS = ex1_A.o ex2_A.o


all: test_joints test_bras simulateur

# --- Règles de compilation des objets ---
ex1_A.o: ex1_A.cpp ex1_A.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

ex2_A.o: ex2_A.cpp ex2_A.hpp ex1_A.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# --- Règles de compilation des exécutables ---
#Test Ex 1
test_joints: test_ex1_A.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(GTEST_LIBS)


#Test Ex 2
test_bras: test_ex2_A.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(GTEST_LIBS)


# 3. Programme principal (Séance 4) 
simulateur: simulateur.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)


# --- Nettoyage ---
clean:
	rm -f *.o test_joints test_bras simulateur