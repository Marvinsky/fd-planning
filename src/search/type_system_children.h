#ifndef type_system_children_h
#define type_system_children_h

#include <map>

#include "child.h"

class TypeSystemChildren {
private:
	map<Child, int> col;

public:
	TypeSystemChildren();
	void addChild(Child h);
	void addChildValue(Child h, int number);
	map<Child, int> getCol() const;

	friend bool operator < (const TypeSystemChildren&, const TypeSystemChildren&);
	friend class Object2;
};

TypeSystemChildren::TypeSystemChildren() {

}

map<Child, int> TypeSystemChildren::getCol() const {
	return this->col;
}

void TypeSystemChildren::addChild(Child c) {
	map<Child, int>::iterator it = col.find(c);

	if (it == col.end()) {
		col.insert(pair<Child, int>(c, 1));
	} else {
		++col[c];
	}
}

bool operator< (const TypeSystemChildren& o1, const TypeSystemChildren& o2) {
	if (o1.col.size() != o2.col.size()) {
		return o1.col.size() < o2.col.size();
	}
	
	map<Child, int>::const_iterator it1 = o1.col.begin();
	map<Child, int>::const_iterator it2 = o2.col.begin();

	for (; it1 != o1.col.end(); ++it1, ++it2) {
		if (it1->first != it2->first) {
			return it1->first < it2->first;
		}
		if (it1->second != it2->second) {
			return it1->second < it2->second;
		}
	}
	return false;
}

#endif
