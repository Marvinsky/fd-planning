#ifndef CHILD_
#define CHILD_

class Child {
private:
	int level;
	int parentHeuristic;
	int h;

public:
	Child(int level, int parentHeuristic, int h) {
		this->level = level;
		this->parentHeuristic = parentHeuristic;
		this->h = h;
	}

	int getLevel() const {return level;}
	int getH() const {return h;}

	//friend class

	friend bool operator!= (const Child& c1, const Child& c2) {
		if (c1.level != c2.level) {
		   return true;
		}

		if (c1.h != c2.h) {
		   return true;
		}
		return false;
	}
	
	friend bool operator< (const Child& c1, const Child& c2) {
		if (c1.level != c2.level) {
		   return c1.level < c2.level;
		}

		if (c1.h != c2.h) {
		   return c1.h < c2.h;
		}
		return false;
	}
};

#endif
