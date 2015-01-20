#include "globals.h"

#include "axioms.h"
#include "causal_graph.h"
#include "domain_transition_graph.h"
#include "heuristic.h"
#include "legacy_causal_graph.h"
#include "operator.h"
#include "rng.h"
#include "state.h"
#include "successor_generator.h"
#include "timer.h"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <sstream>
using namespace std;

#include <ext/hash_map>
using namespace __gnu_cxx;


static const int PRE_FILE_VERSION = 3;


// TODO: This needs a proper type and should be moved to a separate
//       mutexes.cc file or similar, accessed via something called
//       g_mutexes. (Right now, the interface is via global function
//       are_mutex, which is at least better than exposing the data
//       structure globally.)

static vector<vector<set<pair<int, int> > > > g_inconsistent_facts;

bool test_goal(const State &state) { 
    for (int i = 0; i < g_goal.size(); i++) {
        if (state[g_goal[i].first] != g_goal[i].second) {
            return false;
        }
    }
    return true;
}

int calculate_plan_cost(const vector<const Operator *> &plan) {
    // TODO: Refactor: this is only used by save_plan (see below)
    //       and the SearchEngine classes and hence should maybe
    //       be moved into the SearchEngine (along with save_plan).
    int plan_cost = 0;
    for (int i = 0; i < plan.size(); i++) {
        plan_cost += plan[i]->get_cost();
    }
    return plan_cost;
}

void save_plan(const vector<const Operator *> &plan, int iter) {
    // TODO: Refactor: this is only used by the SearchEngine classes
    //       and hence should maybe be moved into the SearchEngine.
    ofstream outfile;
    if (iter == 0) {
        outfile.open(g_plan_filename.c_str(), ios::out);
    } else {
        ostringstream out;
        out << g_plan_filename << "." << iter;
        outfile.open(out.str().c_str(), ios::out);
    }
    for (int i = 0; i < plan.size(); i++) {
        cout << plan[i]->get_name() << " (" << plan[i]->get_cost() << ")" << endl;
        outfile << "(" << plan[i]->get_name() << ")" << endl;
    }
    outfile.close();
    int plan_cost = calculate_plan_cost(plan);
    ofstream statusfile;
    statusfile.open("plan_numbers_and_cost", ios::out | ios::app);
    statusfile << iter << " " << plan_cost << endl;
    statusfile.close();
    cout << "Plan length: " << plan.size() << " step(s)." << endl;
    cout << "Plan cost: " << plan_cost << endl;
}

bool peek_magic(istream &in, string magic) {
    string word;
    in >> word;
    bool result = (word == magic);
    for (int i = word.size() - 1; i >= 0; i--)
        in.putback(word[i]);
    return result;
}

void check_magic(istream &in, string magic) {
    string word;
    in >> word;
    if (word != magic) {
        cout << "Failed to match magic word '" << magic << "'." << endl;
        cout << "Got '" << word << "'." << endl;
        if (magic == "begin_version") {
            cerr << "Possible cause: you are running the planner "
                 << "on a preprocessor file from " << endl
                 << "an older version." << endl;
        }
        exit(1);
    }
}

void read_and_verify_version(istream &in) {
    int version;
    check_magic(in, "begin_version");
    in >> version;
    check_magic(in, "end_version");
    if (version != PRE_FILE_VERSION) {
        cerr << "Expected preprocessor file version " << PRE_FILE_VERSION
             << ", got " << version << "." << endl;
        cerr << "Exiting." << endl;
        exit(1);
    }
}

void read_metric(istream &in) {
    check_magic(in, "begin_metric");
    in >> g_use_metric;
    check_magic(in, "end_metric");
}

void read_variables(istream &in) {
    int count;
    in >> count;
    for (int i = 0; i < count; i++) {
        check_magic(in, "begin_variable");
        string name;
        in >> name;
        g_variable_name.push_back(name);
        int layer;
        in >> layer;
        g_axiom_layers.push_back(layer);
        int range;
        in >> range;
        g_variable_domain.push_back(range);
        if (range > numeric_limits<state_var_t>::max()) {
            cerr << "This should not have happened!" << endl;
            cerr << "Are you using the downward script, or are you using "
                 << "downward-1 directly?" << endl;
            exit(1);
        }

        in >> ws;
        vector<string> fact_names(range);
        for (size_t i = 0; i < fact_names.size(); i++)
            getline(in, fact_names[i]);
        g_fact_names.push_back(fact_names);
        check_magic(in, "end_variable");
    }
}

void read_mutexes(istream &in) {
    g_inconsistent_facts.resize(g_variable_domain.size());
    for (size_t i = 0; i < g_variable_domain.size(); ++i)
        g_inconsistent_facts[i].resize(g_variable_domain[i]);

    int num_mutex_groups;
    in >> num_mutex_groups;

    /* NOTE: Mutex groups can overlap, in which case the same mutex
       should not be represented multiple times. The current
       representation takes care of that automatically by using sets.
       If we ever change this representation, this is something to be
       aware of. */

    for (size_t i = 0; i < num_mutex_groups; ++i) {
        check_magic(in, "begin_mutex_group");
        int num_facts;
        in >> num_facts;
        vector<pair<int, int> > invariant_group;
        invariant_group.reserve(num_facts);
        for (size_t j = 0; j < num_facts; ++j) {
            int var, val;
            in >> var >> val;
            invariant_group.push_back(make_pair(var, val));
        }
        check_magic(in, "end_mutex_group");
        for (size_t j = 0; j < invariant_group.size(); ++j) {
            const pair<int, int> &fact1 = invariant_group[j];
            int var1 = fact1.first, val1 = fact1.second;
            for (size_t k = 0; k < invariant_group.size(); ++k) {
                const pair<int, int> &fact2 = invariant_group[k];
                int var2 = fact2.first;
                if (var1 != var2) {
                    /* The "different variable" test makes sure we
                       don't mark a fact as mutex with itself
                       (important for correctness) and don't include
                       redundant mutexes (important to conserve
                       memory). Note that the preprocessor removes
                       mutex groups that contain *only* redundant
                       mutexes, but it can of course generate mutex
                       groups which lead to *some* redundant mutexes,
                       where some but not all facts talk about the
                       same variable. */
                    g_inconsistent_facts[var1][val1].insert(fact2);
                }
            }
        }
    }
}

void read_goal(istream &in) {
    check_magic(in, "begin_goal");
    int count;
    in >> count;
    for (int i = 0; i < count; i++) {
        int var, val;
        in >> var >> val;
        g_goal.push_back(make_pair(var, val));
    }
    check_magic(in, "end_goal");
}

void dump_goal() {
    cout << "Goal Conditions:" << endl;
    for (int i = 0; i < g_goal.size(); i++)
        cout << "  " << g_variable_name[g_goal[i].first] << ": "
             << g_goal[i].second << endl;
}

void read_operators(istream &in) {
    int count;
    in >> count;
    for (int i = 0; i < count; i++)
        g_operators.push_back(Operator(in, false));
}

void read_axioms(istream &in) {
    int count;
    in >> count;
    for (int i = 0; i < count; i++)
        g_axioms.push_back(Operator(in, true));

    g_axiom_evaluator = new AxiomEvaluator;
    g_axiom_evaluator->evaluate(*g_initial_state);
}

void read_everything(istream &in) {
    read_and_verify_version(in);
    read_metric(in);
    read_variables(in);
    read_mutexes(in);
    g_initial_state = new State(in);
    read_goal(in);
    read_operators(in);
    read_axioms(in);
    check_magic(in, "begin_SG");
    g_successor_generator = read_successor_generator(in);
    check_magic(in, "end_SG");
    DomainTransitionGraph::read_all(in);
    g_legacy_causal_graph = new LegacyCausalGraph(in);

    // NOTE: causal graph is computed from the problem specification,
    // so must be built after the problem has been read in.
    g_causal_graph = new CausalGraph;
}

void dump_everything() {
    cout << "Use metric? " << g_use_metric << endl;
    cout << "Min Action Cost: " << g_min_action_cost << endl;
    cout << "Max Action Cost: " << g_max_action_cost << endl;
    // TODO: Dump the actual fact names.
    cout << "Variables (" << g_variable_name.size() << "):" << endl;
    for (int i = 0; i < g_variable_name.size(); i++)
        cout << "  " << g_variable_name[i]
             << " (range " << g_variable_domain[i] << ")" << endl;
    cout << "Initial State (PDDL):" << endl;
    g_initial_state->dump_pddl();
    cout << "Initial State (FDR):" << endl;
    g_initial_state->dump_fdr();
    dump_goal();
    /*
    cout << "Successor Generator:" << endl;
    g_successor_generator->dump();
    for(int i = 0; i < g_variable_domain.size(); i++)
      g_transition_graphs[i]->dump();
    */
}

void verify_no_axioms_no_cond_effects() {
    if (!g_axioms.empty()) {
        cerr << "Heuristic does not support axioms!" << endl << "Terminating."
             << endl;
        exit(1);
    }

    for (int i = 0; i < g_operators.size(); i++) {
        const vector<PrePost> &pre_post = g_operators[i].get_pre_post();
        for (int j = 0; j < pre_post.size(); j++) {
            const vector<Prevail> &cond = pre_post[j].cond;
            if (cond.empty())
                continue;
            // Accept conditions that are redundant, but nothing else.
            // In a better world, these would never be included in the
            // input in the first place.
            int var = pre_post[j].var;
            int pre = pre_post[j].pre;
            int post = pre_post[j].post;
            if (pre == -1 && cond.size() == 1 && cond[0].var == var
                && cond[0].prev != post && g_variable_domain[var] == 2)
                continue;

            cerr << "Heuristic does not support conditional effects "
                 << "(operator " << g_operators[i].get_name() << ")" << endl
                 << "Terminating." << endl;
            exit(1);
        }
    }
}

bool are_mutex(const pair<int, int> &a, const pair<int, int> &b) {
    if (a.first == b.first) // same variable: mutex iff different value
        return a.second != b.second;
    return bool(g_inconsistent_facts[a.first][a.second].count(b));
}

double comb(unsigned n, unsigned m)
{
  //cout<<"\t\t\tcalling comb with n:"<<n<<",m:"<<m<<endl;
  if(n==0||m>n){
    //cout<<"N can not be 0!, exiting from combination calculator"<<endl;
    return 0;
  }
  else if(m==0){
    cerr<<"m can not be 0!, exiting from combination calculator"<<endl;
    exit(EXIT_FAILURE);
  }
    double cnm = 1.0;
    
    if (m * 2 > n) 
        m = n - m;
    
    for (unsigned i = 1; i <= m; n--, i++)
    {
        cnm /= i;
        cnm *= n;
    }
    return cnm;
}

bool g_use_metric;
int g_min_action_cost = numeric_limits<int>::max();
int g_max_action_cost = 0;
vector<string> g_variable_name;
vector<int> g_variable_domain;
vector<vector<string> > g_fact_names;
vector<int> g_axiom_layers;
vector<int> g_default_axiom_values;
State *g_initial_state;
vector<pair<int, int> > g_goal;
vector<Operator> g_operators;
vector<Operator> g_axioms;
AxiomEvaluator *g_axiom_evaluator;
SuccessorGenerator *g_successor_generator;
vector<DomainTransitionGraph *> g_transition_graphs;
CausalGraph *g_causal_graph;
LegacyCausalGraph *g_legacy_causal_graph;

Timer g_timer;
Timer search_timer;
double F_boundary_time=0;
string g_plan_filename = "sas_plan";
RandomNumberGenerator g_rng(2011); // Use an arbitrary default seed.
vector<bool> h_capped;
double time_limit=1800;
unsigned problem_index=0;
double memory_limit=3145728;
double node_gen_and_exp_cost=0;
bool early_termination=false;
bool global_duplicate_check=false;
bool cycle_duplicate_check=true;
bool random_sampling=false;
bool random_heur_selec=false;
int leaves_to_sample=1000;
int hoff_root_F=0;
string chosen_heurs;
double leaf_reval_percent=1.0;
bool draw_graph=false;
//enum RIDA_PHASES Current_RIDA_Phase=SOLVING_PHASE;
enum RIDA_PHASES Current_RIDA_Phase=SAMPLING_PHASE;
//enum RIDA_PHASES Current_RIDA_Phase=RANDOM_HEUR_SELEC;
double total_IDA_iter_sampling_timer=0;
double total_credit_assignment_timer=0;
double total_solving_timer=0;
unsigned Degree=15;
int g_random_seed=0;
double leaf_selection_ratio=0;
//vector< vector< vector<int> > > PDB_list;
bool revaluation_random_sampled_leaves=false;
bool check_consistency=true;
map<int,int> real_heur_pos;
set<int> strong_heur;
set<int> initial_strong_heur;
long maximum_combination_limit=20000;//100000 heur combined to 15 degrees
long RANDOM_CAP=1000;//100000 heur combined to 15 degrees
//int memory_limit=6340032;
//int memory_limit=63400;
double sampling_thresshold=100.0;
//double sampling_time_limit=0.20;//with respect to previous F-boundary
double sampling_time_limit=20;//seconds
double gen_to_eval_ratio=1.0;
double gen_to_exp_ratio=1.0;
map<int, int> nodes_expanded_by_level;//Number of nodes expanded by level
map<int, int> nodes_generated_by_level;//Number of nodes generated by level
map<int, vector<long> > mapv_f;
string domain_name = "temp";
string problem_name2 = "temp";
string heuristic_name2 = "temp";
int ss_probes = 0;
int node_time_adjusted_reval=0;
bool one_time_sampling=true;
bool full_sampling=false;
int argc_copy; 
char **argv_copy;
long CAPPING_LIMIT=60;
vector<long> capping_limit;
string log_file="used_pdbs.dat";
int random_comb_index=-1;
int last_full_f_boundary=0;
long last_gen_nodes=0;
double last_f_boundary_time=0;
bool problem_was_solved=false;
bool use_saved_pdbs=false;
bool evaluate_randomization=true;
string problem_name="temp";
double overall_original_pdbs_time=0;
boost::dynamic_bitset<> prunning_id;
vector<unsigned> culprit_chains;
long states_generated_lower_depth=0;
bool adding_found_lower_depth=false;
int limited_expansion=INT_MAX/2;
vector<double> selec_probs;
    
int pdb_lp_max_memory=512000;//50MB
int canonical_max_memory=204800;//50MB
int pdb_lp_starting_memory=0;
int pdb_lp_time_limit=20;
bool no_more_ga_pdbs=false;
bool incremental_memory_limit=false;
int F_bound_to_print=0;
int common_sampling_F_boundary=0;
