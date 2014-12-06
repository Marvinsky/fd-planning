#include <iostream>
#include <cstdlib>
#include <string>
#include "hash.h"

using namespace std;

hash::hash() {
	for (int i = 0; i < tableSize; i++) {
	    HashTable[i] = new item;
            HashTable[i]->h = -1;
	    HashTable[i]->g = -1;
            HashTable[i]->next = NULL;
        }
}

int hash::Hash(int key) {
	int hash = 0;
        hash = fvalue;
        int index;
        
        index = hash % tableSize;
        return index;
}


