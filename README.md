This is an algorithm that has been done.
We have a map, we need to go to the end point, ('S'->'G')
but the way through a certain number of points, ('@')
how to get out of the shortest path, the number of steps away.

OS: 
Ubuntu 14.04 x64

Compiler: 
gcc version 4.8.4 (Ubuntu 4.8.4-2ubuntu1~14.04)

Language:
C++

KeyWord： 
1)AStar
2)Floyd
3)Makefile

Function: 
Will example1.txt file in of information read out, '#' as obstacles, '.'for the feasible region, 
through the operation to find a starting point for the 'S', the end point was the 'G', 
and after all the '@' shortest path number of steps.

Ideas: 
First, the shortest path and step of any two points are calculated by A* algorithm, 
then the problem of Hamiltonian path is solved. 
Finally, the shortest path number is obtained by Floyd algorithm.
