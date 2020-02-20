#include "RRT.h"

bool IntersectObs(const Vector2i &p1, const Vector2i &p2, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject)
{
	// intersection avec la liste d'obstacle
	for (size_t i = 0; i < obstacles.size(); i++)
	{
		Vector2i centre = Vector2i(obstacles[i]->getPos().getX(), obstacles[i]->getPos().getY());
		int rayon = obstacles[i]->getRadius();

		if (dist(p1 - centre) < rayon + radiusObject ||
			dist(p2 - centre) < rayon + radiusObject ||
			segmentIntersectSimplifiedCircle(p1, p2, centre, rayon + halfWidthObject))
		{
			return true;
		}
	}

	//intersection avec les murs
	if (dist(p1 - tasseauBleu) < 1 + radiusObject ||
		dist(p2 - tasseauBleu) < 1 + radiusObject ||
		segmentIntersectSimplifiedCircle(p1, p2, tasseauBleu, 1 + halfWidthObject) ||

		dist(p1 - tasseauJaune) < 1 + radiusObject ||
		dist(p2 - tasseauJaune) < 1 + radiusObject ||
		segmentIntersectSimplifiedCircle(p1, p2, tasseauJaune, 1 + halfWidthObject) ||

		dist(p1 - tasseauCentral1) < 1 + radiusObject ||
		dist(p2 - tasseauCentral1) < 1 + radiusObject ||
		segmentIntersectSimplifiedCircle(p1, p2, tasseauCentral1, 1 + halfWidthObject) ||

		dist(p1 - tasseauCentral2) < 1 + radiusObject ||
		dist(p2 - tasseauCentral2) < 1 + radiusObject ||
		segmentIntersectSimplifiedCircle(p1, p2, tasseauCentral2, 1 + halfWidthObject))
	{
		return true;
	}

	return false;
}

bool segmentIntersectCircle(const Vector2i &p1, const Vector2i &p2, const Vector2i &c, const int &r)
{
	/** Fonction à optimiser car appelée d'innombrables fois. Dans la plupart des cas, 
	 * la fonction renvoie false, il faudrait donc optimiser au maximum les performances 
	 * lorsqu'il n'y a pas collision
     * */

	// légère optimisation permettant de gagner en performance : on verifie si le segment a des chances d'entrer en collision avec le cercle
	if (min(p1.getX(), p2.getX()) > c.getX() + r ||
		max(p1.getX(), p2.getX()) < c.getX() - r ||
		min(p1.getY(), p2.getY()) > c.getY() + r ||
		max(p1.getY(), p2.getY()) < c.getY() - r)
	{
		return false;
	}
	float dx = p2.getX() - p1.getX();
	float dy = p2.getY() - p1.getY();

	float Ox = p1.getX() - c.getX();
	float Oy = p1.getY() - c.getY();

	float A = dx * dx + dy * dy;
	float B = 2 * (dx * Ox + dy * Oy);
	float C = Ox * Ox + Oy * Oy - r * r;

	float delta = B * B - 4 * A * C;

	if (delta < 0)
		return false;

	float x1 = (-B + sqrt(delta)) / 2.f / A;
	float x2 = (-B - sqrt(delta)) / 2.f / A;

	if ((x1 >= 0.f && x1 <= 1.f) || (x2 >= 0.f && x2 <= 1.f))
	{
		return true;
	}

	return false;
}

bool segmentIntersectSimplifiedCircle(const Vector2i &p1, const Vector2i &p2, const Vector2i &c, const int &r)
{
	int x1 = p1.getX();
	int x2 = p2.getX();

	int y1 = p1.getY();
	int y2 = p2.getY();

	int xc = c.getX();
	int yc = c.getY();

	if (x1 == x2) // cas où le segment [p1, p2] est vertical
	{
		if (abs(x1 - xc) <= r && (y1 - yc) * (y2 - yc) <= 0)
		{
			return true;
		}
		return false;
	}

	float a = (y2 - y1) * 1.f / (x2 - x1); // coeff directeur
	float b = y1 - a * x1;				   // ordonnée à l'origine

	if ((x1 - xc) * (x2 - xc) <= 0) // si x1 et x2 sont de part et d'autre horizontalement de xc
	{
		if (abs(a * xc + b - yc) <= r) // check si collision avec le segment vertical de la croix
		{
			return true;
		}
	}

	if ((y1 - yc) * (y2 - yc) <= 0) // si y1 et y2 sont de part et d'autre verticalement de yc
	{
		if (abs((yc - b) - a * xc) <= abs(a * r)) // check si collision avec le segment horizontal de la croix
		{
			return true;
		}
	}
	return false;
}

float dist(const std::vector<Vector2i> &vector)
{
	float d = 0;
	for (size_t i = 0; i < vector.size(); i++)
	{
		d += dist(vector[i]);
	}

	return d;
}

// Node ----------------------------------------
Node::Node(const Vector2i pos, Node *parentNode, const float distToOrigin) : pos(pos), parentNode(parentNode), distToOrigin(distToOrigin)
{
	childrenNodes = std::vector<Node *>();
}

Node::~Node()
{
	while (childrenNodes.size() > 0)
	{
		delete childrenNodes[static_cast<int>(childrenNodes.size()) - 1];
		childrenNodes[static_cast<int>(childrenNodes.size()) - 1] = NULL;
		childrenNodes.pop_back();
	}
}

int Node::getNbChildren() const
{
	if (childrenNodes == std::vector<Node *>())
		return 0;
	return static_cast<int>(childrenNodes.size());
}

void Node::addChild(const Vector2i &posChild)
{
	this->childrenNodes.push_back(new Node(posChild, this, dist(this->pos - posChild) + this->distToOrigin + rotationCost));
}

void Node::linkToTarget(Node &target)
{
	this->childrenNodes.push_back(&target);
	target.setParent(this);
}

void Node::detachTarget(Node &target)
{
	for (size_t i = 0; i < childrenNodes.size(); i++)
	{
		if (childrenNodes[i] == &target)
		{
			childrenNodes.erase(childrenNodes.begin() + i);
		}
	}
	target.setParent(NULL);
}

Node *Node::detachChild(int indChild)
{
	Node *returnNode = this->childrenNodes[indChild];
	childrenNodes.erase(childrenNodes.begin() + indChild);
	return returnNode;
}

void Node::findClosestParcoursRec(const Vector2i &p, float &shortestDist, Node *&closest, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject)
{
	if (this->distToOrigin + dist(p - this->pos) + rotationCost < shortestDist && !IntersectObs(p, this->pos, obstacles, radiusObject, halfWidthObject))
	{
		closest = this;
		shortestDist = this->distToOrigin + dist(p - this->pos) + rotationCost;
	}
	for (int i = 0; i < static_cast<int>(childrenNodes.size()); i++)
	{
		childrenNodes[i]->findClosestParcoursRec(p, shortestDist, closest, obstacles, radiusObject, halfWidthObject);
	}
}

void Node::displayNode() const
{
	std::cout << "node in " << pos.getX() << " " << pos.getY() << ". Nb children : " << static_cast<int>(childrenNodes.size()) << std::endl;
}

void Node::QtdisplayNode(Serial *pc) const
{
	pc->printf("%d\n%d\n", this->pos.getX(), this->pos.getY());
}

// RRT ---------------------------------------------
// Private

Node *RRT::findClosestNode(const Vector2i &p, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject) const
{
	Node *closest = NULL;
	float d = 10000.f;
	this->root->findClosestParcoursRec(p, d, closest, obstacles, radiusObject, halfWidthObject);

	return closest;
}

void RRT::addNode(const Vector2i &pos, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject)
{
	Node *indClosest = findClosestNode(pos, obstacles, radiusObject, halfWidthObject);
	if (indClosest) // si un point a ete trouve
	{
		indClosest->addChild(pos);
	}
}

void RRT::generateTree(const int nbIterations, const Point &target, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject)
{
	for (int i = 0; i < nbIterations; i++)
	{
		Vector2i newNode(radiusObject + 1.f * rand() * (tableLength - 2 * radiusObject) / RAND_MAX,
						 radiusObject + 1.f * rand() * (tableWidth - 2 * radiusObject) / RAND_MAX);
		addNode(newNode, obstacles, radiusObject, halfWidthObject);
	}
}

void RRT::deleteRandomBranch()
{
	int indRand = static_cast<int>(rand() * static_cast<float>(root->getChildren().size()) / RAND_MAX);
	delete root->detachChild(indRand);
}

void RRT::updatePath(const Vector2i &targetPos, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject)
{
	// setting target up
	target.setPos(targetPos);
	if (target.getParent())
	{
		target.getParent()->detachTarget(target);
	}

	// finding the best node to link target to
	Node *closest = findClosestNode(targetPos, obstacles, radiusObject, halfWidthObject);

	if (closest != NULL) // si un point a ete trouve
	{
		closest->linkToTarget(target);

		std::vector<Vector2i> tempPath;
		Node *temp = &target;
		while (temp != NULL)
		{
			tempPath.push_back(temp->getPos());
			temp = temp->getParent();
		}
		if (evaluatePath(tempPath) < this->pathLength - rotationCost)
		{
			// application du chemin trouvé dans path
			this->path.resize(tempPath.size(), Vector2i());
			for (int i = 0; i < static_cast<int>(tempPath.size()); ++i)
			{
				this->path[i] = tempPath[tempPath.size() - 1 - i];
			}
			this->pathLength = evaluatePath(path);
		}
	}
	if (target.getParent())
	{
		target.getParent()->detachTarget(target);
	}
}

bool RRT::isPathLegit(const Vector2i &objectPos, const Vector2i &targetPos, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject)
{
	if (path.size() == 0 || path[0] != objectPos || path[path.size() - 1] != targetPos)
		return false;

	for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
	{
		if (IntersectObs(path[i], path[i + 1], obstacles, radiusObject, halfWidthObject))
		{
			return false;
		}
	}
	return true;
}

// Public
RRT::RRT(const Vector2i &rootPos) : target(Vector2i(-1, -1))
{
	this->root = new Node(rootPos);

	path = std::vector<Vector2i>();
	pathLength = basePathLength;
}

RRT::~RRT()
{
	if (target.getParent())
	{
		target.getParent()->detachTarget(target);
	}
	delete root;
}

void RRT::resetTree(const Point &rootPos)
{
	if (root)
		delete root;
	root = new Node(rootPos.getPos());
}

void RRT::resetPath()
{
	path.resize(0, Vector2i());
	pathLength = basePathLength;
}

int RRT::countNodes() const
{
	int returnNb = 0;

	std::vector<Node *> toVisit = std::vector<Node *>();
	toVisit.push_back(this->root);
	Node *current;
	while (toVisit.size() > 0)
	{
		returnNb++;

		current = toVisit[static_cast<int>(toVisit.size()) - 1];
		toVisit.pop_back();

		for (size_t i = 0; i < current->getChildren().size(); i++)
		{
			toVisit.push_back(current->getChildren()[i]);
		}
	}
	return returnNb;
}

void RRT::maintainTree(const Point &objectPos, const Point &target, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject)
{
	root->setPos(objectPos.getPos());
	std::vector<Node *> toCheck, orphanBranches;
	toCheck.push_back(root);
	Node *current;

	while (toCheck.size() > 0)
	{
		current = toCheck[static_cast<int>(toCheck.size()) - 1];
		toCheck.pop_back();
		if (current->getParent())
		{
			current->setDistToOrigin(dist(current->getParent()->getPos() - current->getPos()) + current->getParent()->getDistToOrigin() + rotationCost);
		}
		for (size_t i = 0; i < current->getChildren().size(); i++)
		{
			if (IntersectObs(current->getPos(), current->getChildren()[i]->getPos(), obstacles, radiusObject, halfWidthObject))
			{
				Node *toDelete = current->detachChild(i);
				delete toDelete;
			}
			else
			{
				toCheck.push_back(current->getChildren()[i]);
			}
		}
	}
}

void RRT::update(const Point &objectPos,
				 const Point &target,
				 const std::vector<Obstacle *> &obstacles,
				 const int &radiusObject,
				 const int &halfWidthObject,
				 const float &timeAvailable)
{
	float timeAtBegin = us_ticker_read() * 0.001f;
	float currentTime = us_ticker_read() * 0.001f;

	while (currentTime - timeAtBegin < timeAvailable * 0.9f) // *0.9f pour ne pas dépasser timeAvailable
	{
		generateTree(3, target, obstacles, radiusObject, halfWidthObject);
		if (countNodes() > 600) // permet de renouveler l'arbre
		{
			deleteRandomBranch();
		}
		currentTime = us_ticker_read() * 0.001f;
	}

	if (!isPathLegit(objectPos.getPos(), target.getPos(), obstacles, radiusObject, halfWidthObject))
		resetPath();

	updatePath(target.getPos(), obstacles, radiusObject, halfWidthObject);
}

void RRT::mainLoop(const Point &objectPos,
				   const Point &target,
				   const std::vector<Obstacle *> &obstacles,
				   const int &radiusObject,
				   const int &halfWidthObject,
				   const float &timeAvailable)
{
	float beginLoopTime = us_ticker_read() * 0.001f;

	this->maintainTree(objectPos, target, obstacles, radiusObject, halfWidthObject);

	float timeLeft = timeAvailable - (us_ticker_read() * 0.001f - beginLoopTime);

	this->update(objectPos, target, obstacles, radiusObject, halfWidthObject, timeLeft);
}

void RRT::display() const
{
	std::cout << "Beginning of RRT display" << std::endl;

	std::cout << "current target : ";
	target.displayNode();

	std::cout << "Root : ";
	this->root->displayNode();

	std::vector<Node *> toVisit = std::vector<Node *>();
	toVisit.push_back(this->root);
	Node *current = toVisit[0];
	while (toVisit.size() > 0)
	{
		current = toVisit[static_cast<int>(toVisit.size()) - 1];

		current->displayNode();

		toVisit.pop_back();

		for (int i = 0; i < current->getNbChildren(); i++)
		{
			toVisit.push_back(current->getChildren()[i]);
		}
	}

	std::cout << "End of RRT display" << std::endl;
	std::cout << "Final path display" << std::endl;

	for (int i = 0; i < static_cast<int>(path.size()) - 1; i++)
	{
		std::cout << path[i].getX() << ", " << path[i].getY() << std::endl;
		std::cout << path[i + 1].getX() << ", " << path[i + 1].getY() << std::endl;
	}

	std::cout << "End of Path display" << std::endl
			  << std::endl;
}

void RRT::QtdisplayRRT(Serial *pc) const
{
	std::vector<Node *> toVisit = std::vector<Node *>();
	toVisit.push_back(this->root);
	Node *current;
	while (toVisit.size() > 0)
	{
		current = toVisit[static_cast<int>(toVisit.size()) - 1];
		toVisit.pop_back();

		if (current->getParent())
		{
			current->getParent()->QtdisplayNode(pc);
			current->QtdisplayNode(pc);
		}

		for (std::size_t i = 0; i < current->getChildren().size(); ++i)
		{
			toVisit.push_back(current->getChildren()[i]);
		}
	}
}

void RRT::QtdisplayPath(Serial *pc) const
{
	for (int i = 0; i < static_cast<int>(path.size()) - 1; i++)
	{
		pc->printf("%d\n%d\n", path[i].getX(), path[i].getY());
		pc->printf("%d\n%d\n", path[i + 1].getX(), path[i + 1].getY());
	}
}