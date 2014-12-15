#ifndef TYPE_SYSTEM_h__
#define TYPE_SYSTEM_h__

#include <vector>
#include <string>

#include "type.h"
#include "type_children.h"
#include "state.h"
#include "heuristic.h"
#include "operator.h"
#include "search_engine.h"

using namespace::std;


class TypeSystem {
private:

	void sample(State state, int parent_heuristic, TypeChildren& children, int type, int current_level);
	short* getEmptyFeatures(int lookahead);
	Heuristic* heuristic;
	int best_h;

public:
	TypeSystem(Heuristic* heuristic);
	~TypeSystem();

	Type getType(State state, int h, int type);
};

#endif

