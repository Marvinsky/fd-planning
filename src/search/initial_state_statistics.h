#ifndef INITIAL_STATE_STATISTICS_H
#define INITIAL_STATE_STATISTICS_H

#include "search_engine.h"
#include "search_space.h"
#include "state.h"
#include "timer.h"
#include "evaluator.h"
#include "search_progress.h"

class Heuristic;
class ScalarEvaluator;
class Options;

class InitialStateStatistics : public SearchEngine {
    ScalarEvaluator *evaluator;
    vector<Heuristic *> heuristics;
protected:
    virtual void initialize();
    virtual int step() { return FAILED; }
public:
    InitialStateStatistics(const Options &opts);
    virtual void statistics() const;
};

#endif // INITIAL_STATE_STATISTICS_H
