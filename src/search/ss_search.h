#ifndef STRATIFIED_SAMPLING_H_
#define STRATIFIED_SAMPLING_H_


#include "state.h"
#include "heuristic.h"
#include "operator.h"
#include "search_engine.h"
#include "type.h"
#include "type_system.h"

static const double DEFAULT_SS_RG = 0.01;
static const double DEFAULT_SS_RL = 0.01;
static const int DEFAULT_SS_LOOKAHEAD = 1;
static const int DEFAULT_SS_BEAMSIZE = 10000;
static const int DEFAULT_SS_MAXLEVELS = 100;
static const int DEFAULT_SS_TIMELIMIT = 1800;

using namespace std;
typedef vector<const Operator*> Path;

class SSNode{
public:
	State state;
	int weight;
	SSNode(State s, int w) : state(s), weight(w){}
};

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
	std::map<Type, SSNode> queue;
	std::vector<Heuristic*> heuristics;
	Heuristic* heuristic;
	State current_state;
	bool progress;
	int total_min;
	int depth;
	int initial_value;
        int threshold;

	Timer search_time;
	Timer level_time; //time required to expand an entire level

	TypeSystem * sampler;

	void restart();
	void jump();
	bool global_restart();
	void report_progress();

protected:

	virtual int step();
	virtual void initialize();

public:
	enum{A_LOT=10000000};
	SSSearch(const Options &opts);
	virtual ~SSSearch();
};

#endif /*MRW_H_*/
