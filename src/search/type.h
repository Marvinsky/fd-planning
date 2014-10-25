#ifndef type_h
#define type_h


#include <map>

//#include "type_children.h"

class Type {
private:
	//TypeChildren  children;
	int p;
	long h;
	long level;
	double prob;
	int random;

public:
	Type();
	//Type(int parent, long heuristic);
	Type(long heuristic, long level);
	//void addAditionalInfo(int);
	//TypeChildren& getChildren() {return children;}
	//TypeChildren getConstChildren() const  {return  children;}
	//void setChildren(TypeChildren c) {this->children = c;}
	
	friend bool operator< (const Type&, const Type&);
	//friend bool operator== (const Type&, const Type&);
	long getH() const {return h;}
	void setH(long i) {h = i;}
	int getP() const {return p;}
	void setP(int i) {p = i;}
	long getLevel() const {return level;}
	void setLevel(long i) {level = i;}
	double getProb() const {return prob;}
	void setProb(double i) {prob = i;}
	int getRandom() {return random;}
	void setRandom(int r) {this->random = r;}

        //static int lookahead;
};

//int Type::lookahead = 0;

/*Type::Type(int parent, long heuristic) {
	this->p = parent;
	this->h = heuristic;
	this->level = -1;
	this->random = -1;
}
*/
Type::Type(long heuristic, long level) {
	this->h = heuristic;
	this->level = level;
}

Type::Type() {
	this->p = -1;
	this->h = -1;
	this->level = -1;
	this->random = -1;
}

bool operator< (const Type& o1, const Type& o2) {
	if (o1.level != o2.level) {
		return o1.level < o2.level;
	}

	if (o1.p != o2.p) {
		return o1.p < o2.p;
	}

	if (o1.h != o2.h) {
		return o1.h < o2.h;
	}

	if (o1.random != o2.random) {
		return o1.random < o2.random;
	}
	return false;
	//return o1.children < o2.children;
}

/*bool operator== (const Type& o1, const Type& o2) {
	if ((o1.level == o2.level) && (o1.h == o2.h)) {
		return true;
	}
	return false;
}

struct CompareObjectDeeperLevelsFirst {
	bool operator() (const Type& o1, const Type& o2) const {
		if (o1.getLevel() != o2.getLevel()) {
			return o1.getLevel() > o2.getLevel();
		}

		if (o1.getP() != o2.getP()) {
			return o1.getP() < o2.getP();
		}
	
		if (o1.getH() != o2.getH()) {
			return o1.getH() < o2.getH();
		}

		if (o1.getRandom() != o2.getRandom()) {
			return o1.getRandom() < o2.getRandom();
		}
	
		return false;
		//return o1.getConstChildren() < o2.getConstChildren();
	}
};


struct CompareObjectLevelLessRandomLess {
	bool operator() (const Type& o1, const Type& o2) {
		if (o1.getP() != o2.getP()) {
			return o1.getP() < o2.getP();
		}		

		if (o1.getH() != o2.getP()) {
			return o1.getH() < o2.getH();
		}
		
		return false;
		//return o1.getConstChildren() < o2.getConstChildren();
	}
};

struct CompareObjectCluster {
	bool operator() (const Type& o1, const Type& o2) const {
		if (o1.getP() != o2.getP()) {
			return o1.getP() < o2.getP();
		}
	return false;
	}
};

*/

#endif
