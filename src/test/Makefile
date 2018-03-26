# Makefile for gtest examples

GOOGLE_TEST_LIB = gtest
GOOGLE_TEST_INCLUDE = /usr/local/include

G++ = g++
G++_FLAGS = -c -Wall -I $(GOOGLE_TEST_INCLUDE)
LD_FLAGS = -L /usr/local/lib -l $(GOOGLE_TEST_LIB) -l pthread

OBJECTS = main.o test.o file_coordinates.o
TARGET = test

all: $(TARGET)

file_coordinates.o: ../file_coordinates.cpp ../file_coordinates.h
	g++ -c ../file_coordinates.cpp -o file_coordinates.o

$(TARGET): $(OBJECTS)
	g++ -o  $(TARGET) $(OBJECTS) $(LD_FLAGS)

%.o : %.cpp
	$(G++) $(G++_FLAGS) $<

clean:
	rm -f $(TARGET) $(OBJECTS)
                    
.PHONY: all clean