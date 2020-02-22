Nicholas Jensen
ENPM661 Project 1

Running proj1_Nicholas_Jensen.py

Library Requirements:

1:numpy
2:time

The program will prompt you to enter the state of the 
unsolved gameboard formatted as (1,2,3),(4,5,6),(7,8,0)
Remember to use 0 for the blank space!

This formatting is a representation of the gameboard 
state as shown below:

|1 2 3|
|4 5 6|
|7 8 0|

The program will then display the following:

1:Number of inversions
2:States that will be explored and # of moves need for each iteration
3:The time it takes to find the solution in seconds
4:The solution itself, in moves that should be applied to the 0 tile

The program will generate 4 text files:

1:nodePath.txt  -- The gameboard states between the start and the goal
2:Nodes.txt -- All gameboard states explored, including the move 
  sequence that created those states
3:Nodesinfo.txt -- A 2 column list with nodes on left and parent nodes
  on the right
4:ordered_Nodesinfo.txt -- A ordered list of all move sequences attempted


 
