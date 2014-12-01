#include "node.h"

Node::Node() {
	//TODO
}

Node::Node(int h, int g, int f) {
	this->h = h;
        this->g = g;
	this->f = f;
}

int Node::getH() {
	return this->h;
}

int Node::getG() {
	return this->g;
}

int Node::getF() {
	return this->f;
}
