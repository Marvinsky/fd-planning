#ifndef EAGER_SEARCH_H
#define EAGER_SEARCH_H

#include <vector>

#include "open_lists/open_list.h"
#include "search_engine.h"
#include "search_space.h"
#include "state.h"
#include "timer.h"
#include "evaluator.h"
#include "search_progress.h"
#include "Tree.h"

class Heuristic;
class Operator;
class ScalarEvaluator;
class Options;

class EagerSearch : public SearchEngine {
    // Search Behavior parameters
    bool reopen_closed_nodes; // whether to reopen closed nodes upon finding lower g paths
    bool do_pathmax; // whether to use pathmax correction
    bool use_multi_path_dependence;
    bool mark_children_as_finished;

    Timer IDA_iter_sampling_timer;
    double total_sampling_timer;
    OpenList<state_var_t *> *open_list;
    ScalarEvaluator *f_evaluator;
    bool first_sample;
    bool first_time;
    int nivel;
    int count_last_nodes_gerados; 
protected:
    int step();
    pair<SearchNode, bool> fetch_next_node();
    bool check_goal(const SearchNode &node);
    void update_jump_statistic(const SearchNode &node);
    void print_heuristic_values(const vector<int> &values) const;
    void reward_progress();

    vector<Heuristic *> heuristics;
    vector<Heuristic *> orig_heuristics;
    vector<Heuristic *> preferred_operator_heuristics;
    vector<Heuristic *> estimate_heuristics;
    // TODO: in the long term this
    // should disappear into the open list

    vector<int> vniveles;
    
    virtual void initialize();
    void sample_frontier_now(int next_f_boundary);
    void output_problem_results();
public:
    EagerSearch(const Options &opts);
    void statistics() const;

    void dump_search_space();
    double get_total_sampling_time(){return total_sampling_timer;}
};

#endif
