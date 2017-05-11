all: rn1client

rn1client: rn1client.cc
	gcc -Wall rn1client.cc -o rn1client -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system
