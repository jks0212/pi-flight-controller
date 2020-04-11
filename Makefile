#all: test

drone: main.o
	g++ -Wall -lrf24-bcm -pthread -o main main.cc -lpigpio -lrt 
	# -lstdc++
	# -li2c

drone.o: main.cc
	g++ -c main.cc -lstdc++
