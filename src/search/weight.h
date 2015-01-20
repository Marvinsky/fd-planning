#ifndef WEIGHT_H
#define WEIGHT_H

class Weight {
private:
	mutable double weight;
public:
        Weight();
        Weight(double w);
        ~Weight();
	double getWeight();
	void setWeight(double w);
};
#endif
