#ifndef DFS_SEARCH_H
#define DFS_SEARCH_H

#include <vector>

#include "open_lists/open_list.h"
#include "search_engine.h"
#include "search_space.h"
#include "state.h"
#include "timer.h"
#include "evaluator.h"
#include "search_progress.h"
#include "Tree.h"

#include "time.h"

#include <map>
#include <locale>
#include <stack>
#include "Stack.h"

#include <iostream>
#include <exception>

class Heuristic;
class Operator;
class ScalarEvaluator;
class Options;

template<class charT, charT sep>
class punct_facet: public std::numpunct<charT> {
protected:
	charT do_decimal_point() const {
              return sep;
        }
};

class DFSSearch : public SearchEngine {
	//search behavior parameters
	bool reopen_closed_nodes; // whether to reopen closed nodes upon finding lower g paths.
	bool do_pathmax; //whether to use pathmax correction
	bool use_multi_path_dependence;
	bool mark_children_as_finished;
	
	Timer IDA_iter_sampling_timer;
	double total_sampling_timer;
	OpenList<state_var_t *> *open_list;
	ScalarEvaluator *f_evaluator;
	bool first_sample;


        string heuristic_name;
        stack<SearchNode> P;
        //stack<SearchNode> S;
        Stack<SearchNode> S;

        vector<int> v_f;
        vector<int> v_g;
        vector<int> v_h;
        int counter;

        Stack<SearchNode> P2;
        Stack<int> Pint;
        

protected:
	int step();
	pair<SearchNode, bool> fetch_next_node();
	//bool check_goal(const SearchNode &node);
	//void update_jump_statistic(const SearchNode &node);
	//void print_heuristic_values(const vector<int> &values) const;	
	void reward_progress();

	vector<Heuristic *> heuristics;
	vector<Heuristic *> orig_heuristics;
	vector<Heuristic *> preferred_operator_heuristics;
	vector<Heuristic *> estimate_heuristics;
	//TODO: in the long term this
	//should disappear into the open list
	
	virtual void initialize();
	void sample_frontier_now(int next_f_boundary);
	void output_problem_results();
public:
	DFSSearch(const Options &opts);
	void statistics() const;
        	
	//void dump_search_space();
	double get_total_sampling_time(){return total_sampling_timer;}
        int getMax_gvalue(vector<int> v_g);
        map<int, int> getFDistribution(vector<int> v_f_value);
        vector<string> readFile();
        void generateReport(vector<int> v_h, vector<int> v_g);
        void printStack(stack<SearchNode> S);
        void printStack2(Stack<SearchNode> S);
        void printStack3(Stack<int> S);
        void DFS(SearchNode n);
}; 
#endif

