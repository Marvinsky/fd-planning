#ifndef type_system_h
#define type_system_h

#include "type.h"
#include "search_space.h"
//#include "abstract_pk_heuristic.h"

#include <map>



class TypeSystem {
private:
	//AbstractPKHeuristic *hf;
public:
	TypeSystem();
	//TypeSystem(AbstractPKHeuristic *hf);
	//~TypeSystem();
	Type getType(SearchNode &node, long level);

};

TypeSystem::TypeSystem() {

}

//TypeSystem::TypeSystem(AbstractPKHeuristic *hf) {
//	this->hf = hf;
//}

//TypeSystem::~TypeSystem() {
//	delete hf;
	//}


Type TypeSystem::getType(SearchNode &node, long level) {
	long h = node.get_h();
	Type obj(h, level);

	return obj;
}

#endif
