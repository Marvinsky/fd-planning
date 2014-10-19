
#ifndef _TREE_
#define _TREE_


#include <stdio.h>    
#include <queue>
#include <vector>
#include <map>
#include <string>
#include <bitset>
//#include "ff.h"
#include"state.h"
#include"operator_cost.h"
#include"globals.h"
#include "search_node_info.h"
#include "search_space.h"
#include "state_proxy.h"
#include <ext/hash_map>
#include "timer.h"


//#include "heuristic.h"
//#include "option_parser.h"
#include "successor_generator.h"
#include "operator.h"
//#include "g_evaluator.h"
//#include "sum_evaluator.h"
#include "plugin.h"


#include <cstdlib>
using namespace std;

#define STARTING_CAPACITY 10000
#define HEURISTICS 1


void create_emergency_strong_heur_call(set<int> strong_heurs,vector<Heuristic *> heuristics);
double get_ED_from_EBF(double EBF,long N);
struct compressed_search_node{
  state_var_t S;
  unsigned char depth;
};
/*struct vcompressed_node{
  char S_char[MAX_TILE];
  unsigned char depth;
};*/

struct bitset_comp2 
{
  bool operator() (const boost::dynamic_bitset<> & s1, const boost::dynamic_bitset<> & s2) const
  {
    if(s1.size()!=s2.size()){
      return true;
    }
    for(int x=s1.size(); x >= 0; x--)
    {
      if(s1[x] != s2[x])
	return s1[x] < s2[x];
    }
    return false;
  }
}; // end of ci_less
void binary(int number);

/*class Heuristic;
class Operator;
class ScalarEvaluator;
class Options;*/

/******************************************************************************/
/*********************** CLASS HUSTSEARCHNODE *************************************/
/******************************************************************************/

class HUSTSearchNode
{
  //state_var_t S; 
  //state_var_t *S;
  const State *S;
  //SearchNodeInfo &info;
  int cost_type;
  const Operator *generating_op;
  //int father;
  int depth;
  int H;
  //vector<int> children_to_generate;
  vector<const Operator *> children_to_generate;
public:
  HUSTSearchNode(const State *_S,const Operator *_generating_op, int _depth, int _H);
  void deallocate();
  int get_depth();
  int get_last_depth();
  const Operator* get_op();
//  int get_father();
  int get_H();
  bool to_be_expanded();
  //int get_next_op();
  const Operator* get_next_op();
  const Operator* get_next_random_op();
  const Operator* get_next_random_op_no_erase(int &chosen_op);
  const Operator* get_op_no_erase(int chosen_op);
  int get_children_number();
  void get_children(vector<const Operator*> children);
  void get_state(const State* &_S);
  void print_list_children();
  void print_state();
};
class DrawNode
{
  int father;
  string op;
  int depth;
  int H;
  int hg_node;
  bool gfth_node;
  string state_id;
  int heuristic_states;
public:
  DrawNode(int father, string op, int depth, int H, int hg_node, string state_id,int heuristic_states );
  DrawNode(int father, string op, int depth, int H, int hg_node, bool gfth_node );
  DrawNode(int father, string op, int depth, int H, int hg_node, bool gfth_node, string state_id,int heuristic_states );
  DrawNode(int father, string op, int depth, int H, int hg_node, bool gfth_node , string state_id);
  int get_father();
  string get_op();
  int get_depth();
  int get_F();
  int get_hg_node();
  bool get_gfth_status();
  string get_state_id();
  int get_heuristic_states();
};

/******************************************************************************/
/*********************** CLASS HST ********************************************/
/******************************************************************************/
class HST
{
  class HashTable;
  HashTable *nodes;
  unsigned size;
  unsigned capacity;
  unsigned iter_index;
//  int lspace_end; 
  vector<HUSTSearchNode> lsearch_space;
  queue<compressed_search_node> next;
  vector<DrawNode> TreeDraw;
  vector<vector<int> > nodes_culled;
  vector<vector<int> > nodes_gfth;
  vector<vector<int> > nodes_fully_expanded;
//  vector<long> heuristic_states_per_depth;
//  vector<Homogeneus> hg_tree;
  int current_hg_node;
  int current_draw_node;
  int current_F_bound;
  int node_id;
  int node_id_father;
  int alternative_paths;//used for counting optimal paths to same state on duplicated state search
  string current_state_id;
  map<string,unsigned char> visited_database;
  vector<unsigned char> pattern_database;
  vector<vector<unsigned char> > multiple_pattern_databases;
  vector<map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> > h_counter;
  vector<map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> > h_counter_substract;
  map<int,pair<bitset<HEURISTICS>,long> > best_heuristic;
  unsigned F_bound;
  vector<int> earliest_depth_h_culled;
  unsigned hset_size;
  unsigned orig_hset_size;
  vector<vector<unsigned> > h_comb_to_degree;
 
  map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> Culling_size;
  map<boost::dynamic_bitset<>,double,bitset_comp2> HBF;
  map<boost::dynamic_bitset<>,unsigned,bitset_comp2> culling_iteration;
  map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> prev_val;
  map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> h_comb_map;
  vector< boost::dynamic_bitset<> > best_h_comb_list;
  vector< map<unsigned long,boost::dynamic_bitset<> > > h_comb_map2;
  vector<long double> time_cost;
  boost::dynamic_bitset<> best_h_comb;
  unsigned long HUST_total_size;
  unsigned long HSTs_total_size;
  unsigned long HSTs_total_time;
  unsigned long HUST_total_time;
  unsigned long Credit_total_time;
  unsigned long Total_Sampling_time;
  vector<const Operator*> solution_path;
  long dead_end_nodes;
  Timer current_sampling_timer;
  Timer credit_sampling_timer;
  vector<Heuristic *> strong_and_comp_heur;
  int last_sampled_HoF_Root;
  int F_boundary_size;
  double Pred_Asymptotic_HBF;
  double Pred_Extra_Nodes;
private:
  void annotate_sampling_data(vector<Heuristic *> orig_heuristics,vector<Heuristic *> heuristics);
  void annotate_sampling_preds(vector<Heuristic *> orig_heuristics);
public:
  HST( unsigned start_capacity=STARTING_CAPACITY,unsigned start_h_setsize=1 );
  ~HST();
  int get_depth(int index);
  int get_last_depth();
  const Operator* get_op(int index);
  const Operator* get_last_op();
  int get_last_H();
//  int get_father(int index);
  int get_H(int index);
  int get_children_number(int index );
  bool to_be_expanded();
  int get_nodes_culled(int depth, int H);
  int get_nodes_gfth(int depth, int H);
  int get_nodes_fully_expanded(int depth, int H);
  void add_nodes_culled(int depth, int H);
  void add_nodes_culled(int depth, int H, const Operator* op, const State* S);
  /*void add_nodes_gfth(int depth, int H);
  void add_nodes_gfth(int depth, int H, int heuristic_states);*/
  void add_nodes_gfth();
  int get_lspace_end( );
  int get_nodes_fully_expanded_size();
  //void add_to_HST( int op, int father, int H );
//  long get_heuristic_states_per_depth(int depth);
  bool checking_consistency();

  /*void add_to_HST(vector<int> *children_to_generate, int op, int father, int H );
  void add_to_HST( vector<int> *children_to_generate,int op, int father, int H, int depth  );
  void add_to_HST(vector<int> *children_to_generate, int op, int father, int H, int depth,int heuristic_states);
  void add_to_HST(vector<int> *children_to_generate, int op, int father, int H, const state_var_t* initial_state );
  void add_to_HST(vector<int> *children_to_generate, int op, int father, int H, const state_var_t* initial_state, int depth );*/
  void add_to_HST(const State* parent_state,const Operator* op, int H, int depth);
  void add_to_HST_breadth_first(const state_var_t* current_state,const vector<int>* tiles_grouped);

  void reset_HST();
  void reset_duplicate_states();
  void reset_h_counters();
  void reset_earliest_depth();
  bool empty();
  //int get_next_op();
  const Operator* get_next_op();
  const Operator* get_next_random_op();
  const Operator* get_next_random_op_no_erase(int &chosen_op);
  const Operator* get_op_no_erase(int chosen_op);
  void backtrack();
  void copy_current_path(vector<HUSTSearchNode> *solution_path);
  int get_max_depth();
  int get_max_size();
  long get_nodes_fully_expanded();
  int get_hg_tree_size();
  int get_hg_children(int index);
  int get_hg_root_depth(int index);
  int get_hg_max_depth(int index);
  int get_hg_F(int index);
  int get_hg_size(int index);
  bool get_hg_closed(int index);
  bool get_hg_gfth_status(int index);
  bool get_avg_hg_tree_size_by_F_lim(int F_limit,double *avg_size, int *number_clusters, double *avg_children_number2, double *avg_children_number, double *total_size_growth);
  bool get_hg_EBF_ED(int index,double *EBF,double *ED);
  int get_TreeDraw_size();
  int get_TreeDraw_depth(int index);
  int get_TreeDraw_F(int index);
  int get_TreeDraw_father(int index);
  int get_TreeDraw_hg_node(int index);
  int get_TreeDraw_heuristic_nodes(int index);
  bool get_TreeDraw_gfth_status(int index);
  const char* get_TreeDraw_state_id(int index);
  string get_TreeDraw_op(int index);
  double get_ED();
  int get_alternative_paths();
  void add_alternative_path();
  void set_current_state_id(string *state_id);
  void get_current_state_id(string *state_id);
  void get_EBF_ED_from_2iter(int step_size,double *EBF,double *ED,long N1,long N2);
  //void generate_next_state(state_var_t *next_state);
  //int get_hg_leaf_nodes_size(int index);
  //int get_hg_leaf_nodes_children(int index);
  void get_state(int node,const State* &S);  
  void extract_current_path();
  int count_significant_tiles(int start, int end);
  void print_list_children();
  int get_next_node_to_expand_breadth_first();
  void create_pattern_database();
  //bool check_duplicate_database(const state_var_t *successor_state,const vector<int>* tiles_grouped,unsigned char depth);
  bool check_duplicate(const State* current_state,const Operator* generating_op);
  bool check_duplicate(const State* current_state,const Operator* generating_op,vector<Heuristic *> &heuristics);
  bool check_duplicate_database(const State* current_state);
  //void get_compact_database_index(const state_var_t* current_state,const vector<int> *tiles_grouped,int &index,int call_number);
  void get_compact_database_index();
  void print_database_to_file(const vector<int>* tiles_grouped,int number);
  void read_databases_from_files();
  int get_database_value(int index,int call_number);
  void get_state_id_from_index(int index,const vector<int> *tiles_grouped,string *state_id);
  void add_to_counter(boost::dynamic_bitset<> *h_bitset, unsigned long nodes);
  void add_to_substract_counter(boost::dynamic_bitset<> *h_bitset, unsigned long nodes);
  void substract_from_counter(boost::dynamic_bitset<> *h_bitset, unsigned nodes);
  void print_h_counters();
  void fprint_h_counters();
  long unsigned total_generated_by_dropped_HEURISTICS(bitset<HEURISTICS> bs);
  void set_current_F_bound(int F);
  int get_current_F_bound();
  unsigned get_iter_index();
  void inc_iter_index();
  void print_current_state();
  void set_F_bound(unsigned F);
  unsigned get_F_bound();
  void set_earliest_depth_h_culled(int heuristic,int depth);
  int get_earliest_depth_h_culled(int database);
  int get_current_children_number();
  void get_current_state(const State* &S);  
  const Operator* get_current_state();
  void print_final_size_counters(bool actual_print,int degree);
  void calculate_heuristics_to_degree(unsigned degree);
  void calculate_heuristics_to_degree(unsigned degree,const vector<Heuristic *> orig_heuristics);
  void reset_iter_index();
  void update_hset_size(unsigned set_size);
  void update_original_hset_size(unsigned set_size);
  unsigned get_hset_size();
  void select_best_heuristics(unsigned degree);
  void select_best_heuristics(unsigned degree,const vector<Heuristic *> orig_heuristics);
  void switch_to_max_strong_and_comp(vector<Heuristic *> &heuristics);
  void calculate_time_costs();
  boost::dynamic_bitset<> get_best_h_comb();
  void get_best_h_comb(boost::dynamic_bitset<> &selec_heur);
  void copy_solution_path(const Operator* last_op);
  void print_solution_path();
  void clear_solution_path( );
  void add_dead_end_nodes();
  void create_h_counters(map < vector<short>,long,shortvect_comp2 > *sampled_F_values, int F_bound);
  void print_Dot_Tree();
  void print_counters();
  void get_current_path(vector<const Operator*> &current_path,const Operator* last_op);
  void select_best_estimated_heuristic_subset(SearchSpace *search_space,const vector<pair<State,int> >* chosen_Hoff_Roots,const vector<Heuristic *> orig_heuristics,vector<Heuristic *> &heuristics,int current_f, int F_boundary_size);
  int evaluate_HUST_node(const State* S,int depth,int children,vector<Heuristic *> heuristics);
  void compare_MAX_heuristics(const State* S,vector<int> &winning_h,vector<Heuristic *> heuristics,double min_tpn);
  double calculate_time_costs_specific(boost::dynamic_bitset<> h_comb,const vector<Heuristic *> orig_heuristics);
  void calculate_h_index_only(const State* S, int depth, vector<Heuristic *> heuristics, boost::dynamic_bitset<> &h_index);
  long calculate_specific_heuristic(boost::dynamic_bitset<> h_comb);
  void create_selected_heur_call(vector<Heuristic *> heuristics);
};
#endif
