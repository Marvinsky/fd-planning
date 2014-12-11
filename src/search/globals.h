#ifndef GLOBALS_H
#define GLOBALS_H

#include "operator_cost.h"

#include <iosfwd>
#include <string>
#include <vector>
#include <set>
#include "boost/dynamic_bitset.hpp"
#include <map>
#include <iostream>
using namespace std;

class Axiom;
class AxiomEvaluator;
class CausalGraph;
class DomainTransitionGraph;
class LegacyCausalGraph;
class Operator;
class RandomNumberGenerator;
class State;
class SuccessorGenerator;
class Timer;
class RandomNumberGenerator;
boost::dynamic_bitset<> MyBitSet();//To store CCs and heuristic sets 

bool test_goal(const State &state);
void save_plan(const std::vector<const Operator *> &plan, int iter);
int calculate_plan_cost(const std::vector<const Operator *> &plan);

void read_everything(std::istream &in);
void dump_everything();

void verify_no_axioms_no_cond_effects();

void check_magic(std::istream &in, std::string magic);

bool are_mutex(const std::pair<int, int> &a, const std::pair<int, int> &b);


extern bool g_use_metric;
extern int g_min_action_cost;
extern int g_max_action_cost;

// TODO: The following five belong into a new Variable class.
extern std::vector<std::string> g_variable_name;
extern std::vector<int> g_variable_domain;
extern std::vector<std::vector<std::string> > g_fact_names;
extern std::vector<int> g_axiom_layers;
extern std::vector<int> g_default_axiom_values;

extern State *g_initial_state;
extern std::vector<std::pair<int, int> > g_goal;

extern std::vector<Operator> g_operators;
extern std::vector<Operator> g_axioms;
extern AxiomEvaluator *g_axiom_evaluator;
extern SuccessorGenerator *g_successor_generator;
extern std::vector<DomainTransitionGraph *> g_transition_graphs;
extern CausalGraph *g_causal_graph;
extern LegacyCausalGraph *g_legacy_causal_graph;
extern Timer g_timer;
extern Timer search_timer;
extern double F_boundary_time;//F-boundary time
extern  double total_IDA_iter_sampling_timer;
extern  double total_credit_assignment_timer;
extern  double total_solving_timer;
extern std::string g_plan_filename;
extern RandomNumberGenerator g_rng;
extern std::vector<bool> h_capped;
extern unsigned problem_index;
extern double comb(unsigned n, unsigned m);
#define ITER_STEP 1
enum RIDA_PHASES {SAMPLING_PHASE, PREDICTING_PHASE, SOLVING_PHASE, FIXED_HEUR_COMB,BEST_BY_FITNESS, RANDOM_HEUR_SELEC};
extern double time_limit;
extern double memory_limit;
extern int node_time_adjusted_reval;
extern bool check_consistency;
extern bool global_duplicate_check;
extern bool cycle_duplicate_check;
extern enum RIDA_PHASES Current_RIDA_Phase;
extern bool random_sampling;
extern int leaves_to_sample;
extern int g_random_seed;
extern int hoff_root_F;
extern string chosen_heurs;
extern double leaf_selection_ratio;
extern double leaf_reval_percent;
extern bool revaluation_random_sampled_leaves;
extern unsigned Degree;
extern bool check_consistency;
extern map<int,int> real_heur_pos;
extern long maximum_combination_limit;
extern set<int> strong_heur;
extern set<int> initial_strong_heur;
extern bool random_heur_selec;
extern double node_gen_and_exp_cost;
extern bool one_time_sampling;
extern bool full_sampling;
extern int argc_copy; 
extern char **argv_copy;
extern long CAPPING_LIMIT;
extern string log_file;
extern string problem_name;
extern int random_comb_index;
extern int last_full_f_boundary;
extern long last_gen_nodes;
extern double last_f_boundary_time;
extern bool problem_was_solved;
extern bool use_saved_pdbs;
extern double overall_original_pdbs_time;
extern double sampling_thresshold;
extern double sampling_time_limit;//with respect to previous F-boundary
extern double gen_to_eval_ratio;
extern double gen_to_exp_ratio;
extern map<int,int> nodes_expanded_by_level;
extern map<int,int> nodes_generated_by_level;
extern map<int, vector<long> > mapv_f;
extern string domain_name;
extern string problem_name2;
extern string heuristic_name2;
extern bool evaluate_randomization;
extern bool adding_found_lower_depth;
extern long RANDOM_CAP;//100000 heur combined to 15 degrees
extern boost::dynamic_bitset<> culprit_id;
extern vector<unsigned> culprit_chains;
extern long states_generated_lower_depth;
extern int limited_expansion;
extern vector<double> selec_probs;
extern bool draw_graph;
extern int pdb_lp_max_memory;
extern int pdb_lp_starting_memory;
extern int pdb_lp_time_limit;
extern bool no_more_ga_pdbs;
extern bool incremental_memory_limit;
extern int canonical_max_memory;//50MB
struct shortvect_comp2 
{
  bool operator() (const std::vector<short> & s1, const std::vector<short> & s2) const
  {
    if(s1.size()!=s2.size()){
      return true;
    }
    for(int x=s1.size()-1; x >= 0; x--)
    {
      if(s1[x] != s2[x])
	return s1[x] < s2[x];
    }
    return false;
  }
}; // end of ci_less
extern int F_bound_to_print;
extern int common_sampling_F_boundary;
#endif
