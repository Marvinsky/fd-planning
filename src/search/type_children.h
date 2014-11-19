#ifndef TYPE_CHILDREN_
#define TYPE_CHILDREN_

#include <map>

#include "Child.h"

using namespace std;

class TypeChildren {
private:
	map<Child, int> col;
public:
	TypeChildren();
	void addChild(Child h);
	void addChildValue(Child h, int number);
        map<Child, int> getCol() const;
 
	friend bool operator< (const TypeChildren&, const TypeChildren&);
	friend class Object2;
};

TypeChildren::TypeChildren() {

}

map<Child, int> TypeChildren::getCol() const {
	return this->col;
}

void TypeChildren::addChild(Child c) {
	map<Child, int>::iterator it = col.find(c);
        if (it ==  col.end()) {
	   col.insert(pair<Child, int>(c, 1));
	} else {
	   ++col[c];
	}
}

void TypeChildren::addChildValue(Child h, int number) {
	col.insert(pair<Child, int>(h, number));
}

bool operator< (const TypeChildren& o1, const TypeChildren& o2) {
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
