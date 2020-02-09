#include "PathfindingDebug.h"

PathfindingDebug::PathfindingDebug(Point point) : MyRRT(point.getPos())
{
    std::cout << "debut creation Objets" << std::endl;
    std::cout << "RRT créé. Root in : " << MyRRT.getRoot()->getPos().getX() << " " << MyRRT.getRoot()->getPos().getY() << std::endl;
    pos = point;
    target = Point(250.f, 100.f, 0.f);
    time = 0.f;

    obstacles.push_back(Obstacle(Vector2f(50.f, 100.f)));
    obstacles.push_back(Obstacle(Vector2f(270.f, 150.f)));
    obstacles.push_back(Obstacle(Vector2f(150.f, 100.f)));
    obstacles.push_back(Obstacle(Vector2f(170.f, 130.f)));
    obstacles.push_back(Obstacle(Vector2f(110.f, 80.f)));
    obstacles.push_back(Obstacle(Vector2f(210.f, 130.f)));
    obstacles.push_back(Obstacle(Vector2f(80.f, 70.f)));
    obstacles.push_back(Obstacle(Vector2f(100.f, 140.f)));
    obstacles.push_back(Obstacle(Vector2f(180.f, 50.f)));
}

void PathfindingDebug::update(float time)
{
    // simulation du mouvement du robot
    pos = Vector2f(150.f + 110.f * sin(time * 0.00021f), 100.f + 75.f * cos(time * 0.0003f));

    target = Vector2f(150.f - 110.f * sin(time * 0.00023f), 100.f + 75.f * cos(time * 0.0005f));

    this->time = time;

    // boucles indispensables aux calculs de RRT
    MyRRT.maintainTree(pos, target, obstacles, 15.f);
    MyRRT.update(pos, target, obstacles, 15.f);
}

void PathfindingDebug::Qtdisplay(Serial *pc)
{
    // display robot position
    {
        pc->printf("%f\n", (float)-2);

        pc->printf("%f\n%f\n", pos.getPos().getX(), pos.getPos().getY());

        pc->printf("%f\n", (float)-1);
    }

    // display robot target
    {
        pc->printf("%f\n", (float)-3);

        pc->printf("%f\n%f\n", target.getPos().getX(), target.getPos().getY());

        pc->printf("%f\n", (float)-1);
    }

    // display time since boot
    {
        pc->printf("%f\n", (float)-4);

        pc->printf("%f\n", time);

        pc->printf("%f\n", (float)-1);
    }

    // display obstacles
    {
        pc->printf("%f\n", (float)-101);

        for (size_t i = 0; i < obstacles.size(); ++i)
        {
            pc->printf("%f\n%f\n", obstacles[i].getPos().getX(), obstacles[i].getPos().getY());
        }

        pc->printf("%f\n", (float)-1);
    }

    // display RRT
    {
        pc->printf("%f\n", (float)-102);

        //MyRRT.QtdisplayRRT(pc);

        pc->printf("%f\n", (float)-1);
    }

    // display node count
    {
        pc->printf("%f\n", (float)-103);

        pc->printf("%f\n", (float)MyRRT.countNodes());

        pc->printf("%f\n", (float)-1);
    }

    // display final path
    {
        pc->printf("%f\n", (float)-104);

        MyRRT.QtdisplayPath(pc);

        pc->printf("%f\n", (float)-1);
    }
}

void PathfindingDebug::debugDisplay()
{
    std::cout << "Debug Display call. Root in : " << MyRRT.getRoot()->getPos().getX() << " " << MyRRT.getRoot()->getPos().getY() << std::endl;

    MyRRT.display();
}
