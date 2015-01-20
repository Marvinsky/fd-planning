#include "weight.h"

Weight::Weight() {
	this->weight = 0.0;	
}

Weight::Weight(double w) {
	this->weight = w;
}

Weight::~Weight() {
	//TODO
}

double Weight::getWeight() {
	return this->weight;
}

void Weight::setWeight(double w) {
	this->weight = w;
}

