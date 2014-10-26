#include "nivel.h"

Nivel::Nivel(int f, int nodes, float time) {
	this->f = f;
	this->nodes = nodes;
	this->time = time;
}

int Nivel::getF() {
	return this->f;
}

void Nivel::setF(int f) {
	this->f = f;
}

int Nivel::getNodes() {
	return this->nodes;
}

void Nivel::setNodes(int nodes) {
	this->nodes = nodes;
}

float Nivel::getTime() {
	return this->time;
}

void Nivel::setTime(float time) {
	this->time = time;
}

