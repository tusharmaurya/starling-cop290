# $ make
# $ ./flock

CC = g++
CFLAGS = -c
DEBUG = -g
SFML = -lsfml-graphics -lsfml-window -lsfml-system

all: boids

boids: Boid.o Flock.o ObsVector.o main.o Simulate.o
	$(CC) $(DEBUG) Boid.o Flock.o ObsVector.o Simulate.o main.o -o flock $(SFML)

Boid.o : Boid.cpp
	$(CC) $(CFLAGS) $(DEBUG) Boid.cpp

ObsVector.o : ObsVector.cpp
	$(CC) $(CFLAGS) $(DEBUG) ObsVector.cpp

Flock.o : Flock.cpp
	$(CC) $(CFLAGS) $(DEBUG) Flock.cpp

Simulate.o : Simulate.cpp
	$(CC) $(CFLAGS) $(DEBUG) Simulate.cpp

clean:
	\rm *.o flock

tar:
	tar cfv boids.tar ObsVector.h ObsVector.cpp Boid.h Boid.cpp \
		Flock.h Flock.cpp Simulate.cpp Simulate.h
