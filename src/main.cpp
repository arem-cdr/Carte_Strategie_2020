#include <mbed.h>
#include <iostream>
#include <vector>

//#include "Vector2f.h"
#include "Obstacle.h"
#include "RRT.h"
#include "Point.h"
#include "PathfindingDebug.h"
#include "Vector2f.h"

Serial pc(USBTX, USBRX);

int main()
{
	pc.baud(2000000);

	PathfindingDebug *pathfindingDebug = new PathfindingDebug(Vector2f(70.f, 40.f));
	float timeAtWhileBegin = 0.f;

	while (1)
	{
		timeAtWhileBegin = us_ticker_read() * 0.001f;

		pathfindingDebug->update(timeAtWhileBegin);

		while (us_ticker_read() * 0.001f - timeAtWhileBegin < 140)
		{
		}
		pathfindingDebug->Qtdisplay(&pc);
		//pathfindingDebug->debugDisplay();
	}
	delete pathfindingDebug;
}