#ifndef STRATIFIED_SAMPLING_H_
#define STRATIFIED_SAMPLING_H_


#include "state.h"
#include "heuristic.h"
#include "operator.h"
#include "search_engine.h"
#include "type.h"
#include "type_system.h"
#include "node2.h"

#include "map"

#include "randomc/randomc.h"
#include "randomc/mersenne.cpp"


static const double DEFAULT_SS_RG = 0.01;
static const double DEFAULT_SS_RL = 0.01;
static const int DEFAULT_SS_LOOKAHEAD = 1;
static const int DEFAULT_SS_BEAMSIZE = 10000;
static const int DEFAULT_SS_MAXLEVELS = 100;
static const int DEFAULT_SS_TIMELIMIT = 1800;

using namespace std;
typedef vector<const Operator*> Path;

class SSNode{
private:
	State state;
	double weight;
public:
        SSNode(): state(NULL), weight(0.0){}
        SSNode(State s, double w) : state(s), weight(w){}
        State getState() {return this->state;}
        void setState(State s) {this->state = s;}
        double getWeight()  {return this->weight;}
        void setWeight(double w) {this->weight = w;} 
};

/*
class SSNode {
public:
	State state;
	double weight;
        SSNode(State s, double w) : state(s), weight(w) {}
};
*/


class SSSearch : public SearchEngine {
public: 
private:
	double rg;
	double rl;
	int lookahead; //defines the type system being used
	long beamsize; //maximum number of nodes expanded by level
	int maxlevels; //maximum number of levels expanded with no progress before switching to a more refined type system
	int timelimit; //time limit to solve the problem in seconds

	std::map<int, SSNode> open;
	map<Type, SSNode> queue;
        map<Type, double> S; 
        vector<SSNode> vweight;
        std::map<Node2, unsigned long long> collector;

        std::map<Node2, unsigned long long> average;
        double totalPrediction;         

	std::vector<Heuristic*> heuristics; 
	Heuristic* heuristic;
	State current_state;
	bool progress;
	int total_min;
	int depth;
	int initial_value;
        int threshold;
        int count_value;
        int count_level_value;

	Timer search_time;
	Timer level_time; //time required to expand an entire level

	TypeSystem * sampler;

	//void restart();
	//void jump();
	//bool global_restart();
	void report_progress();

        CRandomMersenne* RanGen;        

protected:

	virtual int step();
	virtual void initialize();

public:
	enum{A_LOT=10000000};
	SSSearch(const Options &opts);
	virtual ~SSSearch();
        void printQueue();
        void generateReport(int id);
        void generateAverageReport();
        double getProbingResult();
        void probe();
        void predict(int probes);
};

#endif /*MRW_H_*/
