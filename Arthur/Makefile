# Compilateur
CXX = g++

# Options compilation
CXXFLAGS = -std=c++17 -Wall -Iinclude $(shell pkg-config --cflags pinocchio eigen3)

# Librairies
LIBS = $(shell pkg-config --libs pinocchio eigen3) -lgtest -lgtest_main

# -----------------------------
# FICHIERS COMMUNS AU PROJET
# -----------------------------

COMMON_SRC = \
	src/CJoint.cpp \
	src/CJointRevolute.cpp \
	src/CJointPrismatic.cpp \
	src/CBras.cpp

COMMON_OBJ = $(COMMON_SRC:.cpp=.o)

# -----------------------------
# EXECUTABLE EXERCICE 1
# -----------------------------

EX1_SRC = src/test_joint.cpp
EX1_OBJ = $(EX1_SRC:.cpp=.o)

EX1 = exercice1

# -----------------------------
# EXECUTABLE EXERCICE 2
# -----------------------------

EX2_SRC = src/test_bras.cpp
EX2_OBJ = $(EX2_SRC:.cpp=.o)

EX2 = exercice2

# -----------------------------
# BUILD
# -----------------------------

all: $(EX1) $(EX2)

$(EX1): $(COMMON_OBJ) $(EX1_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

$(EX2): $(COMMON_OBJ) $(EX2_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compilation générique
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Nettoyage
clean:
	rm -f src/*.o $(EX1) $(EX2)