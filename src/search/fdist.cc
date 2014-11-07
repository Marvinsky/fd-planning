#include "fdist.h"

FDist::FDist() {
	//TODO
}

FDist::FDist(vector<int> level_max, int max_g) {
	this->levelmax = level_max;
	this->maxg = max_g;
}

std::vector<int> FDist::getLevelMax() {
	return this->levelmax;
}

void FDist::setLevelMax(vector<int> level_max) {
	this->levelmax = level_max;
}

int FDist::getMaxg() {
	return this->maxg;
}

void FDist::setMaxg(int max_g) {
	this->maxg = max_g;
}

