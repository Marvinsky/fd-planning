#ifndef fdist_h
#define fdist_h


#include <vector>

class FDist {
private:
	std::vector<int> levelmax;
        int maxg;
public:
	FDist();
        FDist(std::vector<int> level_max, int max_g);
	std::vector<int> getLevelMax();
	void setLevelMax(std::vector<int> level_max);
	int getMaxg();
	void setMaxg(int max_g);
};

#endif
