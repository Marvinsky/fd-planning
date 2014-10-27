#ifndef type_system_h
#define type_system_h

#include "type.h"
#include "search_space.h"

#include <map>



class TypeSystem {
private:
	//AbstractPKHeuristic *hf;
public:
	TypeSystem();
	Type getType(SearchNode &node, long level);

};

TypeSystem::TypeSystem() {

}

Type TypeSystem::getType(SearchNode &node, long level) {
	long h = node.get_h();
	Type obj(h, level);

	return obj;
}

#endif
