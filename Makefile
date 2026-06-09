.PHONY: all clean run

all:
	$(MAKE) -C "Boid Simulation"

clean:
	$(MAKE) -C "Boid Simulation" clean

run:
	$(MAKE) -C "Boid Simulation" run
