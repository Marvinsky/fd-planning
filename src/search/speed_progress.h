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

#include <iostream>
#include <fstream>

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

    Timer IDA_iter_sampling_timer;
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

    //Velocity-Based Search Speed Estimator
    
    int initial_value;
    int total_min;
    map<Node2, int> collector2;
    Timer search_time;
    Timer level_time; //time required to expand an entire level
    double target_search_velocity;
    double V; // Search velocity - it is calcultated based the number of nodes generated
    double search_speed; // Search velocity - it is calculated based the number of nodes expanded
    double SEv;  //Future search effort or search effort estimation
    double VeSP; //Velocity Search Progress Estimator

    ofstream outputFile2;
    
    //Vacillation-Based Search Speed Estimator   


 
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

    virtual void initialize();
    void sample_frontier_now(int next_f_boundary);

public:
    SpeedProgress(const Options &opts);
    void statistics() const;

    void dump_search_space();
    double get_total_sampling_time(){return total_sampling_timer;}

    void generateReport();
    int returnMaxF(vector<int> levels);
    int returnMinF(vector<int> levels);
    void reportProgress();
    int generatedSoFar();
    int expandedSoFar();
};

#endif
