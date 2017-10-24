//
///* A utility function to print solution matrix sol[rows][columns] */
//void printSolution(int sol[rows][columns])
//{
//	for (int i = 0; i < rows; i++)
//	{
//		for (int j = 0; j < columns; j++)
//			printf(" %d ", sol[i][j]);
//		printf("\n");
//	}
//	for(int i =0;i<10;i++){
//	printf("%c",dir[i]);
//}}
//
///* A utility function to check if x,y is valid index for N*N maze */
//bool isSafe(int maze[rows][columns], int x, int y, int visited[rows][columns])
//{
//	// if (x,y outside maze) return false
//	if(x >= 0 && x < columns && y >= 0 && y < rows && maze[y][x] == 1 && visited[y][x]==0)
//		return true;
//
//	return false;
//}
//
//bool solveMazeUtil(int maze[rows][columns], int x, int y, int sol[rows][columns])
//{
//    counter++;
//		
//	// if (x,y is goal) return true
//	if(x == destx && y == desty)
//	{
//		sol[y][x] = 1;
//		return true;
//	}
//
//	// Check if maze[y][x] is valid
//	if(isSafe(maze, x, y,visited) == true)
//	{
//		// mark x,y as part of solution path
//		sol[y][x] = 1;
//		visited[y][x]=1;
//		
//		/* Move forward in x direction */
//		dir[counter]='f';
//		if (solveMazeUtil(maze, x+1, y, sol) == true)
//		{
//		    if(maze[x+1][y]==1)
//			dir[counter]='f';
//		    else
//            counter--;
//            return true;
//		}
//		else
//		dir[counter]='z';
//         
//         dir[counter]='b';
//         if (solveMazeUtil(maze, x-1, y, sol) == true)
// 		{
// 		    if(maze[x-1][y]==1)
//			dir[counter]='b';
//		    else
//            counter--;
//            return true;
//		}
//		else
//		dir[counter]='z';
//        
//		/* If moving in x direction doesn't give solution then
//		Move down in y direction */
//		dir[counter]='r';
//		if (solveMazeUtil(maze, x, y+1, sol) == true) //Put concept of visited
//		{
//            if(maze[x][y+1]==1)
//			dir[counter]='r';
//		    else
//            counter--;
//            return true;
//		}
//		else
//		dir[counter]='z';
//        
//        dir[counter]='l';
//		    
//         if (solveMazeUtil(maze, x, y-1, sol) == true)
// 		{
//             if(maze[x][y-1]==1)
//			dir[counter]='l';
//		    else
//            counter--;
//            return true;
//		}
//		else
//		dir[counter]='z';
//        
//		/* If none of the above movements work then BACKTRACK: 
//			unmark x,y as part of solution path */
//		sol[y][x] = 0;
//		counter--;
//		return false;
//	} 
//        else
//        {
//            counter--;
//        }
//	return false;
//}
//
///* This function solves the Maze problem using Backtracking. It mainly
//uses solveMazeUtil() to solve the problem. It returns false if no 
//path is possible, otherwise return true and prints the path in the
//form of 1s. Please note that there may be more than one solutions, 
//this function prints one of the feasible solutions.*/
//bool solveMaze(int maze[rows][columns], point start, point dest)
//{
//    dest->x=destinationx;
//    dest->y=destinationy;
//    start->x=startx;
//    start-y=starty;
//	int sol[rows][columns]={0};
//	if(solveMazeUtil(maze, startx, starty, sol) == false)
//	{
//		printf("Solution doesn't exist");
//		return false;
//	}
//    dir[counter]='z';
//	// printSolution(sol);
//	return true;
//}
//
///* A recursive utility function to solve Maze problem */
//
//// driver program to test above function
//// int main()
//// {
//	
//// 	solveMaze(maze,0,0,4,4);
//// 	return 0;
//// }
