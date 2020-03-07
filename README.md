# Dijkstra - Project 2, ENPM661.
 
The project was created towards completion of Project 2 as part of ENPM661- Planning for Autonomous Robots at University of Maryland, College park

The obstacles were defined as per shown in the problem statement. 

The Project contains 2 files
* Dijkstra_point.py
* Dijkstra_rigid.py

## Dependencies to run the code. 


* Python Version 3
* Ubuntu 16.04 (might work on future version, not tested)
* OpenCV Version > 3.4.4

### Run the code. 

```
* Clone the repository
```
For point robot, run
```
* python3 Dijkstra_point.py # for point robot
```
A user input of x and y is expected for input and goal point in Cartesian coordinates. 


For rigid robot, run
```
* python3 Dijkstra_rigid.py # for rigid robot
```
In addition to the start point and goal point, user is expected to input robot radius and clearance. 


### Notes
Average Run time for Point robot - 47.14 sec

Average Run time for Rigid robot - 55.5 sec
