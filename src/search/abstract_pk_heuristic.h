#ifndef abstract_pk_heuristic_h
#define abstract_pk_heuristic_h

#include <stdio.h>
#include <stdlib.h>

#include "search_space.h"

class AbstractPKHeuristic {
public:
	virtual int getHeuristic(SearchNode &node) = 0;

};

#endif
