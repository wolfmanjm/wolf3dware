# Declaration of variables
#CC = /datadisk/aux/Stuff/reprap/Firmwares/gcc-arm-none-eabi-4_9-2014q4/bin/arm-none-eabi-g++
CC = g++-4.8
CC_FLAGS = -Wall -Wextra -g -std=gnu++11 -MP -MMD

# File names
EXEC = run
SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
# Set the dependency files that will be used to add header dependencies
DEPS = $(OBJECTS:.o=.d)

# Main target
$(EXEC): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXEC)

# To obtain object files
%.o: %.cpp
	$(CC) -c $(CC_FLAGS) $< -o $@

# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS) *.o *.d

execute:
	$(EXEC)

# Add dependency files, if they exist
-include $(DEPS)
