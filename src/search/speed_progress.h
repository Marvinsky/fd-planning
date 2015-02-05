#ifndef SPEED_PROGRESS_H
#define SPEED_PROGRESS_H

#include <vector>
#include <map>

#include "open_lists/open_list.h"
#include "search_engine.h"
#include "search_space.h"
#include "state.h"
#include "timer.h"
#include "evaluator.h"
#include "search_progress.h"
#include "Tree.h"
#include "node2.h"

class Heuristic;
class Operator;
class ScalarEvaluator;
class Options;

class SpeedProgress : public SearchEngine {
    // Search Behavior parameters
    bool reopen_closed_nodes; // whether to reopen closed nodes upon finding lower g paths
    bool do_pathmax; // whether to use pathmax correction
    bool use_multi_path_dependence;
    bool mark_children_as_finished;

    double total_sampling_timer;
    OpenList<state_var_t *> *open_list;
    ScalarEvaluator *f_evaluator;
    bool first_sample;
    bool first_time;
    int nivel;
    int count_last_nodes_gerados;
    
    map<Node2, int> collector;
    int count_value;
    Timer time_level;
    vector<double> v_timer;
protected:
    int step();
    pair<SearchNode, bool> fetch_next_node();
    void update_jump_statistic(const SearchNode &node);
    void reward_progress();

    vector<Heuristic *> heuristics;
    vector<Heuristic *> orig_heuristics;
    vector<Heuristic *> preferred_operator_heuristics;
    vector<Heuristic *> estimate_heuristics;

    virtual void initialize();
       
public:
    SpeedProgress(const Options &opts); 
};

#endif