CC = g++
LD = g++

CFLAGS = -Wall -Winline
LDFLAGS = 

DEPS = client_memdisk.h ../rn1-brain/mapping.h
OBJ = rn1client.o client_memdisk.o

all: rn1client

%.o: %.cc $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

rn1client: $(OBJ)
	$(LD) $(LDFLAGS) -o rn1client $^ -lm -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system
