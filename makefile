CC = g++
LD = g++

CXXFLAGS = -fpermissive
LDFLAGS =

DEPS = client_memdisk.h ../rn1-brain/mapping.h sfml_gui.h
OBJ = rn1client.o client_memdisk.o sfml_gui.o

all: rn1client

%.o: %.cc $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

rn1client: $(OBJ)
	$(LD) $(LDFLAGS) -o rn1client $^ -lm -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system

e:
	gedit --new-window rn1client.cc sfml_gui.cc sfml_gui.h &
