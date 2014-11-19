#ifndef type_h
#define type_h

#include <map>
#include "type_children.h"



class Type {
private:
        TypeChildren children;
	int p;
	long h;
	long level;
	double prob;
	int random;

public:
	Type();
	//Type(int parent, long heuristic);
	Type(long heuristic, long level);
	
	TypeChildren& getChildren()  {return children;}
        TypeChildren getConstChildren() const {return children;}

	void setChildren(TypeChildren c) {this->children = c;}

	friend bool operator< (const Type&, const Type&);

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

	if (o1.h != o2.h) {
		return o1.h < o2.h;
	}

	return o1.children < o2.children;
}

#endif
