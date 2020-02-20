#include "PathfindingDebug.h"

PathfindingDebug::PathfindingDebug(Point point) : MyRRT(point.getPos())
{
    pos = point;
    target = Point(2500, 1000, 0.f);
    this->globalTime = 0.f;

    //obstacles.push_back(Obstacle(Vector2i(700, 1000), robotRadius));
    obstacles.push_back(new Gobelet(Vector2i(670, 100), RED));
    obstacles.push_back(new Gobelet(Vector2i(2330, 100), GREEN));

    obstacles.push_back(new Gobelet(Vector2i(300, 400), RED));
    obstacles.push_back(new Gobelet(Vector2i(950, 400), GREEN));
    obstacles.push_back(new Gobelet(Vector2i(2050, 400), RED));
    obstacles.push_back(new Gobelet(Vector2i(2700, 400), GREEN));

    obstacles.push_back(new Gobelet(Vector2i(450, 510), GREEN));
    obstacles.push_back(new Gobelet(Vector2i(2550, 510), RED));

    obstacles.push_back(new Gobelet(Vector2i(1100, 800), RED));
    obstacles.push_back(new Gobelet(Vector2i(1900, 800), GREEN));

    obstacles.push_back(new Gobelet(Vector2i(450, 1080), RED));
    obstacles.push_back(new Gobelet(Vector2i(2550, 1080), GREEN));

    obstacles.push_back(new Gobelet(Vector2i(300, 1200), GREEN));
    obstacles.push_back(new Gobelet(Vector2i(1270, 1200), GREEN));
    obstacles.push_back(new Gobelet(Vector2i(1730, 1200), RED));
    obstacles.push_back(new Gobelet(Vector2i(2700, 1200), RED));

    obstacles.push_back(new Gobelet(Vector2i(1065, 1650), GREEN));
    obstacles.push_back(new Gobelet(Vector2i(1335, 1650), RED));
    obstacles.push_back(new Gobelet(Vector2i(1665, 1650), GREEN));
    obstacles.push_back(new Gobelet(Vector2i(1935, 1650), RED));

    obstacles.push_back(new Gobelet(Vector2i(1005, 1955), RED));
    obstacles.push_back(new Gobelet(Vector2i(1395, 1955), GREEN));
    obstacles.push_back(new Gobelet(Vector2i(1605, 1955), RED));
    obstacles.push_back(new Gobelet(Vector2i(1995, 1955), GREEN));
}

PathfindingDebug::~PathfindingDebug()
{
    for (auto &it : obstacles)
    {
        delete &it;
        it = 0;
    }
}

void PathfindingDebug::mainLoop(Serial *pc)
{
    this->globalTime = us_ticker_read() * 0.001f;

    this->update();

    this->Qtdisplay(pc);
}

void PathfindingDebug::update()
{
    // simulation du mouvement du robot
    pos = Vector2i(1500 + (int)(1100.f * sin(this->globalTime * 0.00021f)), 1000 + (int)(750 * cos(this->globalTime * 0.0003f)));

    target = Vector2i(1500 - (int)(1100 * sin(this->globalTime * 0.00023f)), 1000 + (int)(750 * cos(this->globalTime * 0.0005f)));

    // fonctions principales du RRT
    MyRRT.mainLoop(pos, target, obstacles, robotRadius, robotHalfWidth, 50.f);
}

void PathfindingDebug::Qtdisplay(Serial *pc)
{
    float beginTime = us_ticker_read() * 0.001f;
    // display general informations
    {
        // display robot position
        pc->printf("%d\n", -3);
        pc->printf("%d\n%d\n", pos.getPos().getX(), pos.getPos().getY());
        pc->printf("%d\n", -1);

        // display robot target
        pc->printf("%d\n", -4);
        pc->printf("%d\n%d\n", target.getPos().getX(), target.getPos().getY());
        pc->printf("%d\n", -1);

        // display time since boot
        pc->printf("%d\n", -5);
        pc->printf("%d\n", (int)time);
        pc->printf("%d\n", -1);
    }

    // display pathfinding
    {
        // display obstacles
        pc->printf("%d\n", -101);
        for (size_t i = 0; i < obstacles.size(); ++i)
        {
            obstacles[i]->Qtdisplay(pc);
        }
        pc->printf("%d\n", -1);

        // display RRT
        pc->printf("%d\n", -102);
        //MyRRT.QtdisplayRRT(pc);
        pc->printf("%d\n", -1);

        // display node count
        pc->printf("%d\n", -103);
        pc->printf("%d\n", MyRRT.countNodes());
        pc->printf("%d\n", -1);

        // display final path
        pc->printf("%d\n", -104);
        MyRRT.QtdisplayPath(pc);
        pc->printf("%d\n", -1);
    }

    // display asser pour tester l'interface de debug
    {
        pc->printf("%d\n", -201);
        pc->printf("%d\n", static_cast<int>(this->globalTime));
        pc->printf("%d\n", 50 + (int)(sin(this->globalTime * 0.01f) * 15.f));
        pc->printf("%d\n", 50 + (int)(sin(this->globalTime * 0.02f) * 10.f));
        pc->printf("%d\n", 50 + (int)(sin(this->globalTime * 0.002f + 1.f) * 25.f));
        pc->printf("%d\n", 50 + (int)(sin(this->globalTime * 0.012f + 5.f) * 5.f));
        pc->printf("%d\n", -1);
    }

    // display random info pour tester l'interface de debug
    {
        pc->printf("%d\n", -301);
        pc->printf("%d\n", static_cast<int>(this->globalTime));
        pc->printf("%d\n", MyRRT.countNodes());
        pc->printf("%d\n", (int)(us_ticker_read() * 0.001f - globalTime));
        pc->printf("%d\n", (int)(us_ticker_read() * 0.001f - beginTime));
        pc->printf("%d\n", -1);
    }
}

void PathfindingDebug::debugDisplay()
{
    std::cout << "Debug Display call. Root in : " << MyRRT.getRoot()->getPos().getX() << " " << MyRRT.getRoot()->getPos().getY() << std::endl;

    MyRRT.display();
}
