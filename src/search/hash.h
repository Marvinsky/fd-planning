#ifndef HASH_H
#define HASH_H

#include <iostream>
#include <cstdlib>
#include <string>

using namespace std;

class hash {
private:
	static const int tableSize = 100000; //10000000;

	struct item {
             int fvalue;
	     int h;
	     int g;
	     item* next;
	};

        item* HashTable[tableSize];

public:
	hash();
	int Hash(int key);

};
