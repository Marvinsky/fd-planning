#include "pdb_lp_heuristic.h"
#include "globals.h"
#include "operator.h"
#include "operator_cost.h"
#include "pdbs/pattern_generation_haslum.h"
#include "pdbs/pattern_generation_systematic.h"
#include "pdbs/canonical_pdbs_heuristic.h"
#include "pdbs/pdb_heuristic.h"
#include "plugin.h"
#include <set>

using namespace std;
bool no_more_pdb_lp=false;
int pdb_lp_max_patterns=1000000;
PDBLPHeuristic::PDBLPHeuristic(const Options &opts)
    : Heuristic(opts), canonical(0), size(0), time_limit(opts.get<int>("time_limit")) {
    vector<vector<int> > patterns;
    lp_solver_interface = new OsiXxxSolverInterface();
    CoinMessageHandler *lp_solver_message_handler = new CoinMessageHandler();
    lp_solver_message_handler->setLogLevel(0);
    lp_solver_interface->passInMessageHandler(lp_solver_message_handler);

    timer.reset();
    vector<int> pattern_vector;
    pdb_lp_starting_memory=get_memory_VmRSS();
    if(opts.contains("systematic")){
      opts.get<int>("systematic");
      int temp2(opts.get<int>("systematic"));pattern_max_size=temp2;
    }
    
    if(no_more_pdb_lp){
      set_stop_using(true);
      cout<<"Not generating more lp_pdbs, last one took too long"<<endl;
      timer.stop();
      return;
    }


    cout <<get_heur_name()<< ",Starting VmRSS memory: " << get_memory_VmRSS() << " KB" << endl;
    pdb_lp_max_memory=512000;//500MB


    // All of the following options for the lp solver are not yet completely evaluated
    // but some initial tests have shown no evidence of improvement.
/*
    OsiClpSolverInterface * osiclp = dynamic_cast< OsiClpSolverInterface*> (lp_solver_interface);
    if (osiclp) {
        osiclp->setupForRepeatedUse(0,1);
        ClpSolve solve_options;
        solve_options.setSolveType(ClpSolve::useDual);
        osiclp->setSolveOptions(solve_options);
    }
*/
    if (opts.contains("patterns")) {
        patterns = opts.get<vector<vector<int> > >("patterns");
    } else if (opts.contains("systematic") && opts.get<int>("systematic")) {
        Options generator_opts;
	int temp2(opts.get<int>("systematic"));pattern_max_size=temp2;
        generator_opts.set<int>("pattern_max_size", opts.get<int>("systematic"));
        generator_opts.set<bool>("dominance_pruning", opts.get<bool>("dominance_pruning"));
        if (opts.contains("prune_irrelevant_patterns") && opts.get<bool>("prune_irrelevant_patterns")) {
            PatternGenerationSystematic pattern_generator(generator_opts);
            patterns = pattern_generator.get_patterns();
        } else {
            PatternGenerationSystematicNaive pattern_generator(generator_opts);
            patterns = pattern_generator.get_patterns();
        }
    }

    if(patterns.size()>pdb_lp_max_patterns){
      //cout<<"This is the last lp_pdb, pdb_lp_max_patterns("<<pdb_lp_max_patterns<<") patterns is the maximum allowed"<<endl;
      cout<<"eliminating lp_pdb,systematic"<<pattern_max_size<<", pdb_lp_max_patterns("<<pdb_lp_max_patterns<<") patterns is the maximum allowed"<<endl;
      set_stop_using(true);
      no_more_pdb_lp=true;
      timer.stop();
      return;
      for(int i=0;i<pdb_lp_max_patterns;i++){
	pattern_vector.push_back(i);
      }
    }
    if (!patterns.empty()) {
        for (size_t i = 0; i < patterns.size(); ++i){ 
            if (i % 1000 == 0 && i > 0) {
                cout <<get_heur_name()<< ",Generated " << i << "/" << patterns.size() << " PDBs" << endl;
            }
            Options pdb_opts;
            pdb_opts.set<int>("cost_type", cost_type);
            //pdb_opts.set<vector<int> >("pattern", patterns[i]);
	    if(patterns.size()>pdb_lp_max_patterns){
	      size_t chosen=rand() %pattern_vector.size();
	      pdb_opts.set<vector<int> >("pattern", patterns.at(chosen));//randomizing pattern selection
	      //cout<<"selected pattern:"<<patterns.at(chosen)<<endl;
	      pattern_vector.erase(pattern_vector.begin()+chosen);
	      //cout<<"pattern erased"<<patterns.at(chosen)<<",size:"<<pattern_vector.size()<<endl;
	    }
	    else{
	      pdb_opts.set<vector<int> >("pattern", patterns[i]);//randomizing pattern selection
	    }
            PDBHeuristic *pdb = new PDBHeuristic(pdb_opts, false);
	   if(get_memory_VmRSS()>(pdb_lp_starting_memory+pdb_lp_max_memory)){
	       cout<<"reached max memory("<<pdb_lp_max_memory<<") for individual lp so no more patterns!"<<endl;
	      no_more_pdb_lp=true;
	      set_stop_using(true);
	       break;
	   }
            size += pdb->get_size();
            heuristics.push_back(pdb);
            borrowed_heuristics = false;
	    if(i>=(pdb_lp_max_patterns-1)){
	      cout<<pdb_lp_max_patterns<<" patterns reached so exiting"<<endl;
	      set_stop_using(true);
	      no_more_pdb_lp=true;
	      timer.stop();
	      return ;
	      //patterns.resize(i);
	      //break;
	    }
	    if (time_limit > 0 && timer() > time_limit) {
	      cout << "Time limit(,"<<time_limit<<",) reached. Aborting pattern_generation for pdb_lp,systematic=,"<<pattern_max_size<<",#patterns processed:"<<i<<endl;
	      no_more_pdb_lp=true;
	      set_stop_using(true);
	      timer.stop();
	      return;
	      //set_stop_using(true);
	      //patterns.resize(i);
	      //break;
	    }
        }
    } else {
        // compute pattern collection
        PatternGenerationHaslum pgh(opts);
        canonical = pgh.get_pattern_collection_heuristic();
        heuristics = canonical->get_pattern_databases();
        borrowed_heuristics = true;
        size = canonical->get_size();
	if (time_limit > 0 && timer() > time_limit) {
	  cout << "Time limit(,"<<time_limit<<",) reached. Aborting pattern_generation for pdb_lp,systematic=,"<<pattern_max_size<<endl;
	  set_stop_using(true);
	  no_more_pdb_lp=true;
	  timer.stop();
	  return;
	}
    }
    cout<<"finished generating patterns in,"<<timer()<<",secs"<<endl;
    double time_gen_patterns=timer();
    if(is_using()){
      generateLP();
      cout<<"Variables:"<<patterns.size()<<",finished generate_LP in,"<<timer()-time_gen_patterns<<",secs"<<endl;
    }
}
void PDBLPHeuristic::free_up_memory() {
  cout<<endl<<get_heur_name()<<",VmRSS memory available before freeing up memory"<<get_memory_VmRSS();fflush(stdout);
    delete lp_solver_interface;
    clear_pdbs();
    cout<<",VmRSS memory available after freeing up memory"<<get_memory_VmRSS()<<endl;fflush(stdout);
}

PDBLPHeuristic::~PDBLPHeuristic() {
    delete lp_solver_interface;
    clear_pdbs();
}

void PDBLPHeuristic::initialize() {
}


void PDBLPHeuristic::clear_pdbs() {
    delete canonical;
    canonical = 0;
    if (!borrowed_heuristics) {
        for (size_t i = 0; i < heuristics.size(); ++i) {
            delete heuristics[i];
        }
    }
    heuristics.clear();
    size = 0;
}


void PDBLPHeuristic::set_pdbs(std::vector<PDBHeuristic*> heuristics_, bool borrowed_heuristics_) {
    clear_pdbs();
    borrowed_heuristics = borrowed_heuristics_;
    heuristics.insert(heuristics.end(), heuristics_.begin(), heuristics_.end());
    for (size_t i = 0; i < heuristics.size(); ++i) {
        size += heuristics[i]->get_size();
    }
    generateLP();
}

void PDBLPHeuristic::generateLP() {
    // compute operator partitioning
    int num_partitions = 0;
    vector<int> partitioning = vector<int>(g_operators.size(), 0);
    for (size_t i = 0; i < heuristics.size(); ++i) {
        vector<int> block_mapping_rel = vector<int>(g_operators.size(), -1);
        vector<int> block_mapping_not_rel = vector<int>(g_operators.size(), -1);
        PDBHeuristic *h = heuristics[i];
        num_partitions = 0;
        const std::vector<bool> &rel_ops = h->get_relevant_operators();
        for (size_t op_id = 0; op_id < g_operators.size(); ++op_id) {
            if (rel_ops[op_id]) {
                int curr_block = partitioning[op_id];
                if (block_mapping_rel[curr_block] == -1) {
                    block_mapping_rel[curr_block] = num_partitions;
                    ++num_partitions;
                }
                partitioning[op_id] = block_mapping_rel[curr_block];
            } else {
                int curr_block = partitioning[op_id];
                if (block_mapping_not_rel[curr_block] == -1) {
                    block_mapping_not_rel[curr_block] = num_partitions;
                    ++num_partitions;
                }
                partitioning[op_id] = block_mapping_not_rel[curr_block];
            }
        }
    }

    int n_cols = num_partitions;
    int n_rows = heuristics.size();
    double *objective    = new double[n_cols]; //objective coefficients
    double *col_lb       = new double[n_cols]; //column lower bounds
    double *col_ub       = new double[n_cols]; //column upper bounds

    // Minimize x_0 + ... + x_{n_cols - 1}
    fill(objective, objective + n_cols, 1);

    // Variable lower/upper bounds
    fill(col_lb, col_lb + n_cols, 0);
    fill(col_ub, col_ub + n_cols, lp_solver_interface->getInfinity());

    //Define the constraint matrix.
    CoinPackedMatrix *matrix = new CoinPackedMatrix(false,0,0);
    matrix->setDimensions(0, n_cols);
    double *row_entries = new double[n_cols];
    for (size_t i = 0; i < n_rows; ++i) {
      if (time_limit > 0 && timer() > time_limit) {
	  cout << "Time limit(,"<<time_limit<<",) reached. Aborting pattern_generation for pdb_lp,systematic=,"<<pattern_max_size<<",#rows processed:"<<i<<endl;
	  no_more_pdb_lp=true;
	  set_stop_using(true);
	  timer.stop();
	  return;
      }
        PDBHeuristic *h = heuristics[i];
        const std::vector<bool> &rel_ops = h->get_relevant_operators();
        CoinPackedVector row;
        fill(row_entries, row_entries + n_cols, 0.0);
        for (size_t op_id = 0; op_id < g_operators.size(); ++op_id) {
            if (rel_ops[op_id])
                row_entries[partitioning[op_id]] = 1.0;
        }
        row.setFullNonZero(n_cols, row_entries);
        matrix->appendRow(row);
    }

    double *row_ub = new double[n_rows]; //the row upper bounds
    fill(row_ub, row_ub + n_rows, lp_solver_interface->getInfinity());

    double *row_lb = new double[n_rows]; //the row lower bounds
    fill(row_lb, row_lb + n_rows, 0);

    lp_solver_interface->loadProblem(*matrix, col_lb, col_ub, objective,
                                     row_lb, row_ub);
    // after an initial solve we can always use resolve for solving a modified
    // version of the LP
    lp_solver_interface->initialSolve();

    // Clean up
    delete[] objective;
    delete[] col_lb;
    delete[] col_ub;
    delete[] row_entries;
    delete[] row_ub;
    delete[] row_lb;
    delete matrix;
}

void PDBLPHeuristic::add_pattern(const std::vector<int> &pattern) {
    Options pdb_opts;
    pdb_opts.set<int>("cost_type", cost_type);
    pdb_opts.set<vector<int> >("pattern", pattern);
    PDBHeuristic *pdb = new PDBHeuristic(pdb_opts, false);
    size += pdb->get_size();
    heuristics.push_back(pdb);
    generateLP();
}

int PDBLPHeuristic::compute_heuristic(const State& state) {
  //cout<<"pattern_max_size:"<<pattern_max_size<<endl;
    for (size_t i = 0; i < heuristics.size(); ++i) {
        PDBHeuristic *h = heuristics[i];
        h->evaluate(state);
        if (h->is_dead_end())
            return DEAD_END;

        int h_val = h->get_heuristic();
        lp_solver_interface->setRowLower(i, h_val);
    }
    lp_solver_interface->resolve();
    double h_val = lp_solver_interface->getObjValue();
    // TODO avoid code duplication with landmark count heuristic
    double epsilon = 0.01;
    return ceil(h_val - epsilon);
}

static ScalarEvaluator *_parse(OptionParser &parser) {
    PatternGenerationHaslum::create_options(parser);
    parser.add_option<int>("systematic", 0, "Systematically generate all patterns with up to n variables instead of using PatternGenerationHaslum.");
    parser.add_option<bool>("prune_irrelevant_patterns", true, "Prune irrelevant patterns before building the LP.");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.help_mode())
        return 0;

    PatternGenerationHaslum::sanity_check_options(parser, opts);

    if (parser.dry_run())
        return 0;

    return new PDBLPHeuristic(opts);
}

static Plugin<ScalarEvaluator> _plugin("pdb_lp", _parse);
