Measurement infrastructure


Simple python scripts to automate the (performance, memory, energy, etc) measurement for specific benchmarks in different configurations.

Call `python measure_PRG.py SERIESID` for measuring a program PRG with a specific test series.

The program gets configurations that have not been measured yet from a database and writes the measurement results back to the database.

It can be run concurrently on different machines. Each machine will grab different measurement tasks.

mdb.py contains the database connection and queries
mcontrol.py contains the main loop that looks for new measurement tasks and executes them
measure\_xxx contains the actual measurement scripts that performs the measurement and collects the measurement results from various sources


This repository contains a simple script to measure average CPU utilization (using package sysstat) and read results from powermeter (see WemoControl project).

