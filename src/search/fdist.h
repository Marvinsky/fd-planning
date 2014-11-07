#ifndef fdist_h
#define fdist_h


#include <vector>

class FDist {
private:
	vector<int> levelmax;
        int max_g;
public:
	FDist();
	FDist(vector<int> level_max, int max_g);
	vector<int> getLevelmax();
	void setLevelMax(vector<int> levelmax);
	int getMaxg();
	void setMaxg(int max_g);
};

#endif
