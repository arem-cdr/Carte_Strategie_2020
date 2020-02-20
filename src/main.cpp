#include <mbed.h>
#include <iostream>

#include "PathfindingDebug.h"

Serial pc(USBTX, USBRX);

int main()
{
	pc.baud(2000000);
	pc.printf("%d\n%d\n", -2, -1); // signal d'init de l'interface de debug

	PathfindingDebug *pathfindingDebug = new PathfindingDebug(Vector2i(700, 400));

	while (1)
	{
		pathfindingDebug->mainLoop(&pc);
		//pathfindingDebug->debugDisplay();
	}

	delete pathfindingDebug;
}