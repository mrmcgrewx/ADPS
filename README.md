# ADPS
 automated package delivery service (APDS) system 

There are two systems in this project that will interact with each other. One is the automated package 
delivery service (APDS) system to control an AGV and perform package
pickup, scheduling, and delivery. The other is the e-commerce service (ECS) which will provide the APDS the
current AGV location, the current time, information about packages to be delivered, and other relevant
information. The ECS is provided as part of the simulation environment. The APDS
implemented will have one vehicle. This vehicle will need to navigate around the street (i.e., follow the path, in either the clockwise or counterclockwise direction for each package, changing direction
between package deliveries if deemed preferable).
Along this path, there are stops, or houses, where the vehicle will need to stop at to deliver a package. The vehicle
will need to deliver packages one after another as requests arrive. To deliver a package, the vehicle will need to
go to a designated pickup station (to be specified below) to get the package and go to the package’s corresponding
destination house for delivery. Only one package can be picked up and delivered at a time before the next package
is picked up. To deliver these packages, the system will implement two different non-preemptive scheduling algorithms
i.e., delivery of one package is not interrupted by the presence or arrival of other packages in the package queue.
The first is a fixed priority scheduling algorithm called first-come first-served (FCFS) (aka first-in first-out or
FIFO). This is a simple scheduling algorithm in which packages are picked up and delivered in the same order as
they arrive in the package queue. The second is a dynamic priority scheduling algorithm called earliest deadline
first (EDF) or Horn’s algorithm
