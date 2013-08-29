/*
errors:
	for goal position x=0 and y=100....
*/

#include "path.hpp"
PathStructure ps; //Coming from main cognition.
Path p; //The main class containing all functions and variables.
PathPacket pathpackvar; //The class for writing the files.

void call_main() //Function made to replicate path calling from the cognition module.
{
	srand(time(NULL));
	ps.n_obstacles = 2;
	
	ps.absObstacles[0].x=80;
	ps.absObstacles[0].y=80;
	ps.absObstacles[1].x=30;
	ps.absObstacles[1].y=50;
	ps.absObstacles[2].x=100;
	ps.absObstacles[2].y=140;
	for(int i=0;i<ps.n_obstacles;i++)
	{
		// ps.absObstacles[i].x=rand() % 200;
		// ps.absObstacles[i].y=rand() % 200;
		std::cout<<"obstacle coords "<<i<<": "<<ps.absObstacles[i].x<<" "<<ps.absObstacles[i].y<<std::endl;
	}
	
	// ps.ball.x=rand() % 200 - 100;
	// ps.ball.y=rand() % 200 - 100;
	ps.ball.x = 150;
	ps.ball.y = 150;
	std::cout<<"Ball Coordinates: "<<ps.ball.x<<" "<<ps.ball.y<<std::endl;

	// ps.goal.x = rand() % 200 - 100;
	// ps.goal.y = rand() % 200 - 100;
	ps.goal.x=100;
	ps.goal.y=100;
	std::cout<<"Goal Coordinates: "<<ps.goal.x<<" "<<ps.goal.y<<std::endl;
	
	p.path_return(ps); // the main function that is called which returns the final path.
	p.updatePathPacket(pathpackvar,false);//true if file writing to be on...else off.
	cvWaitKey();
}

int main()
{
	call_main();
	return 0;
}