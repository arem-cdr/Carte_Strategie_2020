#include "RRT.h"

bool IntersectObs(const Vector2f &p1, const Vector2f &p2, const std::vector<Obstacle> &obstacles, const float &radiusObject)
{
	for (size_t i = 0; i < obstacles.size(); i++)
	{
		if (segmentIntersectCircle(p1,
								   p2,
								   Vector2f(obstacles[i].getPos().getX(), obstacles[i].getPos().getY()),
								   obstacles[i].getRadius() + radiusObject))
		{
			return true;
		}
	}
	return false;
}

// Node ----------------------------------------
Node::Node(const Vector2f pos, Node *parentNode, const float distToOrigin) : pos(pos), parentNode(parentNode), distToOrigin(distToOrigin)
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
	std::cout << "node in " << pos.getX() << " " << pos.getY() << " Nb children : " << static_cast<int>(childrenNodes.size());
	if (childrenNodes == std::vector<Node *>())
		return 0;
	return static_cast<int>(childrenNodes.size());
}

void Node::addChild(const Vector2f posChild)
{
	//if (this->childrenNodes.size() < 51)
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

void Node::findClosestParcoursRec(Vector2f p, float &d, Node *&closest, const std::vector<Obstacle> &obstacles, float radiusObject)
{
	if (this->distToOrigin + dist(p - this->pos) + rotationCost < d && !IntersectObs(p, this->pos, obstacles, radiusObject))
	{
		closest = this;
		d = this->distToOrigin + dist(p - this->pos) + rotationCost;
	}
	for (int i = 0; i < static_cast<int>(childrenNodes.size()); i++)
	{
		childrenNodes[i]->findClosestParcoursRec(p, d, closest, obstacles, radiusObject);
	}
}

void Node::displayNode() const
{
	std::cout << "node in " << pos << std::endl;
}

void Node::QtdisplayNode(Serial *pc) const
{
	pc->printf("%f\n%f\n", this->pos.getX(), this->pos.getY());
}

// RRT ---------------------------------------------
// Private

Node *RRT::findClosestNode(Vector2f p, const std::vector<Obstacle> &obstacles, float radiusObject) const
{
	Node *closest = NULL;
	float d = 10000.f;
	this->root->findClosestParcoursRec(p, d, closest, obstacles, radiusObject);

	return closest;
}

void RRT::addNode(Vector2f pos, const std::vector<Obstacle> &obstacles, float radiusObject)
{
	Node *indClosest = findClosestNode(pos, obstacles, radiusObject);
	if (indClosest) // si un point a ete trouve
	{
		indClosest->addChild(pos);
	}
}

void RRT::generateTree(int nbIterations, Point target, const std::vector<Obstacle> &obstacles, float radiusObject)
{
	for (int i = 0; i < nbIterations; i++)
	{
		Vector2f newNode(radiusObject + 1.f * rand() * (tableLength - 2 * radiusObject) / RAND_MAX,
						 radiusObject + 1.f * rand() * (tableWidth - 2 * radiusObject) / RAND_MAX);
		addNode(newNode, obstacles, radiusObject);
	}
}

void RRT::deleteRandomBranch()
{
	int indRand = static_cast<int>(rand() * static_cast<float>(root->getChildren().size()) / RAND_MAX);
	delete root->detachChild(indRand);
}

void RRT::updatePath(const Vector2f &targetPos, const std::vector<Obstacle> &obstacles, const float &radiusObject)
{
	// setting target up
	target.setPos(targetPos);
	if (target.getParent())
	{
		target.getParent()->detachTarget(target);
	}

	// finding the best node to link target to
	Node *closest = findClosestNode(targetPos, obstacles, radiusObject);

	if (closest != NULL) // si un point a ete trouve
	{
		closest->linkToTarget(target);

		std::vector<Vector2f> tempPath;
		Node *temp = &target;
		while (temp != NULL)
		{
			tempPath.push_back(temp->getPos());
			temp = temp->getParent();
		}
		if (evaluatePath(tempPath) < pathLength)
		{
			// application du chemin trouvé dans path
			path.resize(tempPath.size(), Vector2f());
			for (int i = 0; i < static_cast<int>(tempPath.size()); ++i)
			{
				path[i] = tempPath[tempPath.size() - 1 - i];
			}
			pathLength = evaluatePath(path);
		}
	}
	if (target.getParent())
	{
		target.getParent()->detachTarget(target);
	}
}

bool RRT::isPathLegit(const Vector2f &objectPos, const Vector2f &targetPos, const std::vector<Obstacle> &obstacles, const float &radiusObject)
{
	if (path.size() == 0 || path[0] != objectPos || path[path.size() - 1] != targetPos)
		return false;

	for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i)
	{
		if (IntersectObs(path[i], path[i + 1], obstacles, radiusObject))
		{
			return false;
		}
	}
	return true;
}

// Public
RRT::RRT(const Vector2f &rootPos) : target(Vector2f(-1, -1))
{
	std::cout << "Root creation in : " << rootPos.getX() << " " << rootPos.getY() << std::endl;
	this->root = new Node(rootPos);
	//root->setPos(Vector2f(cmToPx(p1.getX()), cmToPx(p1.getY())));

	path = std::vector<Vector2f>();
	pathLength = 5000.f;
}

RRT::~RRT()
{
	if (target.getParent())
	{
		target.getParent()->detachTarget(target);
	}
	delete root;
}

void RRT::resetTree(Point rootPos)
{
	if (root)
		delete root;
	root = new Node(rootPos.getPos());
}

void RRT::resetPath()
{
	path.resize(0, Vector2f());
	pathLength = 5000.f;
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

void RRT::maintainTree(Point objectPos, Point target, const std::vector<Obstacle> &obstacles, float radiusObject)
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
			if (IntersectObs(current->getPos(), current->getChildren()[i]->getPos(), obstacles, radiusObject))
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

void RRT::update(Point objectPos, Point target, const std::vector<Obstacle> &obstacles, float radiusObject)
{
	if (!isPathLegit(objectPos.getPos(), target.getPos(), obstacles, radiusObject))
		resetPath();

	updatePath(target.getPos(), obstacles, radiusObject);
	if (countNodes() > 800)
	{
		deleteRandomBranch();
		deleteRandomBranch();
	}
	else
	{
		generateTree(50, target, obstacles, radiusObject);
	}
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
	//pc.printf("%f\n", (float)-102);

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
		pc->printf("%f\n%f\n", path[i].getX(), path[i].getY());
		pc->printf("%f\n%f\n", path[i + 1].getX(), path[i + 1].getY());
	}
}