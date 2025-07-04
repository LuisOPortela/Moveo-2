CXX = g++
CXXFLAGS = -std=c++11 -I./serial/include -I./serial/src -fdiagnostics-color=always -g -Wall
LDFLAGS = -lpthread

SRC = ./src/serial_port_test.cpp ./serial/src/serial.cpp
OBJ = $(SRC:.cpp=.o)

TARGET = serial_port_test

all: $(TARGET)

$(TARGET): $(OBJ)
	@echo "Linking $(TARGET)..."
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)
	@echo "Build complete."

%.o: %.cpp
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@echo "Cleaning up..."
	rm -f $(OBJ) $(TARGET)
	@echo "Clean complete."
