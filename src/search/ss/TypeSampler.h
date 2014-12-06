#ifndef TypeSampler_h__
#define TypeSampler_h__

#include <vector>
#include <string>

#include "Type.h"
#include "TypeChildren.h"
#include "../state.h"
#include "../heuristic.h"
#include "../operator.h"
#include "../search_engine.h"

using namespace::std;


class TypeSampler {
private:

	void sample(State state, int parent_heuristic, TypeChildren& children, int type, int current_level);
	short* getEmptyFeatures(int lookahead);
	Heuristic* heuristic;
	int best_h;

public:
	TypeSampler(Heuristic* heuristic);
	~TypeSampler();

	Type getType(State state, int h, int type);
};

#endif

