#include "zero_one_pdbs_heuristic.h"

#include "pdb_heuristic.h"
#include "util.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"
#include "../utilities.h"

#include <vector>

using namespace std;

ZeroOnePDBsHeuristic::ZeroOnePDBsHeuristic(
    const Options &opts,
    const vector<int> &op_costs)
    : Heuristic(opts) {
    gapdb_heur_TPN=0.00000011;
    bool temp(opts.get<bool>("disjoint"));disjoint_patterns=temp;
    double temp2(opts.get<double>("mutation_rate"));mutation_rate=temp2;
    //cout<<"gapdb_heur_TPN:"<<gapdb_heur_TPN<<endl;
    vector<int> operator_costs;
    if (op_costs.empty()) { // if no operator costs are specified, use default operator costs
      cout<<"using default operator costs"<<endl;
        operator_costs.reserve(g_operators.size());
        for (size_t i = 0; i < g_operators.size(); ++i)
            operator_costs.push_back(get_adjusted_cost(g_operators[i]));
    } else {
      cout<<"operator_costs provided"<<endl;
        assert(op_costs.size() == g_operators.size());
        operator_costs = op_costs;
    }
    
    
    const vector<vector<int> > &pattern_collection(opts.get_list<vector<int> >("patterns"));
    /*vector<vector<int> >pattern_collection;
    pattern_collection.resize(4);
    int myints1[] = {1,2,4,5,6,8,9,11,15,17,28,29,33,42,43,47,48};
    std::vector<int> myint_vect (myints1, myints1 + sizeof(myints1) / sizeof(int) );
    pattern_collection.at(0)=myint_vect;
    int myints2[] = {7,12,14,18,20,21,22,23,24,26,27,31,32,36,38,39,41};
    std::vector<int> myint_vect2 (myints2, myints2 + sizeof(myints2) / sizeof(int) );
    pattern_collection.at(1)=myint_vect2;
    int myints3[] = {0,3,13,19,25,34,35,37,40,44,45,49};
    std::vector<int> myint_vect3 (myints3, myints3 + sizeof(myints3) / sizeof(int) );
    pattern_collection.at(2)=myint_vect3;
    int myints4[] = {10,16,30,46};
    std::vector<int> myint_vect4 (myints4, myints4 + sizeof(myints4) / sizeof(int) );
    pattern_collection.at(3)=myint_vect4;*/
    
    
    //Timer timer;
    approx_mean_finite_h = 0;
    pattern_databases.reserve(pattern_collection.size());
    for (size_t i = 0; i < pattern_collection.size(); ++i) {
        Options opts;
        opts.set<int>("cost_type", cost_type);
        opts.set<vector<int> >("pattern", pattern_collection[i]);
        PDBHeuristic *pdb_heuristic = new PDBHeuristic(opts, false, operator_costs);
        pattern_databases.push_back(pdb_heuristic);

        // get used operators and set their cost for further iterations to 0 (action cost partitioning)
        const vector<bool> &used_ops = pdb_heuristic->get_relevant_operators();
        assert(used_ops.size() == operator_costs.size());
        for (size_t k = 0; k < used_ops.size(); ++k) {
            if (used_ops[k])
                operator_costs[k] = 0;
        }

        approx_mean_finite_h += pdb_heuristic->compute_mean_finite_h();
    }
    string patterns_string;
    get_patterns(patterns_string);
    cout<<"PATTERNS:"<<patterns_string<<",mean_h:"<<approx_mean_finite_h<<",initial_value:"<<compute_heuristic(*g_initial_state)<<endl;
    //cout << "All or nothing PDB collection construction time: " <<
    //timer << endl;
}

ZeroOnePDBsHeuristic::~ZeroOnePDBsHeuristic() {
    for (size_t i = 0; i < pattern_databases.size(); ++i) {
        delete pattern_databases[i];
    }
}

void ZeroOnePDBsHeuristic::initialize() {
}
void ZeroOnePDBsHeuristic::free_up_memory(){
  cout<<"\tmemory before deleting ZeroOnePDBSheuristic pdbs:"<< get_memory_VmRSS() << " KB" << endl;
    for (size_t i = 0; i < pattern_databases.size(); ++i) {
        delete pattern_databases[i];
    }
  cout<<"\tmemory after deleting ZeroOnePDBSheuristic pdbs:"<< get_memory_VmRSS() << " KB" << endl;
}

int ZeroOnePDBsHeuristic::compute_heuristic(const State &state) {
    // since we use action cost partitioning, we can simply add up all h-values
    // from the patterns in the pattern collection
    int h_val = 0;
    for (size_t i = 0; i < pattern_databases.size(); ++i) {
        pattern_databases[i]->evaluate(state);
        if (pattern_databases[i]->is_dead_end())
            return -1;
        h_val += pattern_databases[i]->get_heuristic();
    }
    return h_val;
}

void ZeroOnePDBsHeuristic::dump() const {
  //cout<<"pattern_database.size:"<<pattern_databases.size();fflush(stdout);
    for (size_t i = 0; i < pattern_databases.size(); ++i) {
        cout << pattern_databases[i]->get_pattern() << "-";
    }
}
void ZeroOnePDBsHeuristic::get_patterns(string &patterns){
  //patterns="[";
    for (size_t i = 0; i < pattern_databases.size(); ++i) {
      patterns+="[";
      patterns+=pattern_databases[i]->get_pattern_string();
      patterns+="]";
      //if(i<(pattern_databases.size()-1)){
	patterns+="-";
      //}
    }
  //patterns+="]-";
}
/*  string ZeroOnePDBsHeuristic::get_heur_call_name() {
  string patterns;
  get_patterns(patterns);
  string call="zopdbs(patterns=";call+=patterns;
  call+=")";
  return call;
}*/

static ScalarEvaluator *_parse(OptionParser &parser) {
    parser.add_option<bool>("disjoint", false, "using disjoint variables in the patterns of a collection");
    parser.add_option<double>("mutation_rate", 0.01, "probability between 0 and 1 for flipping a bit");
    Heuristic::add_options_to_parser(parser);
    Options opts;
    parse_patterns(parser, opts);

    if (parser.dry_run())
        return 0;

    return new ZeroOnePDBsHeuristic(opts);
}

static Plugin<ScalarEvaluator> _plugin("zopdbs", _parse);
