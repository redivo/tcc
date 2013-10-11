# Flags for PC compilation
CCPC	= gcc
SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)
INCLUDES = -I. -I./..

pc: $(OBJ)

%.o: %.c
	$(CCPC) -DPC_COMPILATION -c $< $(INCLUDES)
	

clean:
	-rm -rf *.o *.elf *~ *.bin *.map *.hex

