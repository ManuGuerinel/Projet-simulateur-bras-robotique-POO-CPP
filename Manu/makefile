# Compilateur
CXX = g++

# Options compilation
CXXFLAGS = -std=c++17 -Wall -Iinclude $(shell pkg-config --cflags pinocchio eigen3)

# Librairies
LIBS = $(shell pkg-config --libs pinocchio eigen3)

# Dossiers
OBJ_DIR = bin
EXE_DIR = build
SRC_DIR = src

# -----------------------------
# FICHIERS COMMUNS AU PROJET
# -----------------------------

COMMON_SRC = \
	$(SRC_DIR)/CJoint.cpp \
	$(SRC_DIR)/CjointRevolut.cpp \
	$(SRC_DIR)/CjointPrismatic.cpp \
	$(SRC_DIR)/Cbras.cpp

COMMON_OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(COMMON_SRC))

# -----------------------------
# EXECUTABLE EXERCICE 1
# -----------------------------

EX1_SRC = $(SRC_DIR)/test_joints.cpp
EX1_OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(EX1_SRC))

EX1 = $(EXE_DIR)/exercice1

# -----------------------------
# EXECUTABLE EXERCICE 2
# -----------------------------

EX2_SRC = $(SRC_DIR)/test_bras.cpp
EX2_OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(EX2_SRC))

EX2 = $(EXE_DIR)/exercice2

# -----------------------------
# EXECUTABLE EXERCICE 3
# -----------------------------

EX3_SRC = $(SRC_DIR)/test_vector.cpp
EX3_OBJ = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(EX3_SRC))

EX3 = $(EXE_DIR)/exercice3

# -----------------------------
# BUILD
# -----------------------------

all: $(EX1) $(EX2) $(EX3)

$(EX1): $(COMMON_OBJ) $(EX1_OBJ)
	@mkdir -p $(EXE_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

$(EX2): $(COMMON_OBJ) $(EX2_OBJ)
	@mkdir -p $(EXE_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

$(EX3): $(COMMON_OBJ) $(EX3_OBJ)
	@mkdir -p $(EXE_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# Compilation vers bin/
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Nettoyage
clean:
	rm -rf $(OBJ_DIR) $(EXE_DIR)