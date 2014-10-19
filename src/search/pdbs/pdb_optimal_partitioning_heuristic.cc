#include "pdb_optimal_partitioning_heuristic.h"

#include "pattern_generation_systematic.h"
#include "pattern_generation_haslum.h"
#include "canonical_pdbs_heuristic.h"
#include "util.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"
#include "../timer.h"
#include "../utilities.h"

#include <cassert>

using namespace std;

PDBOptimalPartitioningHeuristic::PDBOptimalPartitioningHeuristic(const Options &opts)
    : Heuristic(opts) {

    Timer timer;
    lp_solver_interface = new OsiXxxSolverInterface();
    CoinMessageHandler *lp_solver_message_handler = new CoinMessageHandler();
    lp_solver_message_handler->setLogLevel(0);
    lp_solver_interface->passInMessageHandler(lp_solver_message_handler);
    generateLP(opts.get<vector<vector<int> > >("patterns"));
    cout << "LP construction time: " << timer << endl;
    cout << "LP variables: " << variable_count << endl;
    cout << "LP constraints: " << constraint_count << endl;

    timer.reset();
    // After an initial solve we can always use resolve for solving a modified
    // version of the LP.
    lp_solver_interface->initialSolve();
    cout << "LP initial solve time: " << timer << endl;
    current_abstract_state_vars.resize(pattern_databases.size());
    for (size_t pdb_id = 0; pdb_id < pattern_databases.size(); ++pdb_id) {
        PDBHeuristic *h = pattern_databases[pdb_id];
        int initial_state_index = h->hash_index(*g_initial_state);
        current_abstract_state_vars[pdb_id] = distance_variables[pdb_id][initial_state_index];
    }
}

PDBOptimalPartitioningHeuristic::~PDBOptimalPartitioningHeuristic() {
    clear_pdbs();
    delete lp_solver_interface;
}

void PDBOptimalPartitioningHeuristic::clear_pdbs() {
    for (size_t i = 0; i < pattern_databases.size(); ++i) {
        delete pattern_databases[i];
    }
    heuristic_variables.clear();
    distance_variables.clear();
    action_cost_variables.clear();
    variable_count = 0;
    constraint_count = 0;
}

void PDBOptimalPartitioningHeuristic::initialize() {
}

int PDBOptimalPartitioningHeuristic::compute_heuristic(const State &state) {
    // Set upper bound for distance of current abstract states to 0 and for all other
    // abstract states to infinity.
    for (size_t pdb_id = 0; pdb_id < pattern_databases.size(); ++pdb_id) {
        PDBHeuristic *h = pattern_databases[pdb_id];
        h->evaluate(state);
        if (h->is_dead_end()) {
            return DEAD_END;
        }
        int old_state_var = current_abstract_state_vars[pdb_id];
        lp_solver_interface->setColUpper(old_state_var, lp_solver_interface->getInfinity());
        int new_state_var = distance_variables[pdb_id][h->hash_index(state)];
        lp_solver_interface->setColUpper(new_state_var, 0);
        current_abstract_state_vars[pdb_id] = new_state_var;
    }
    lp_solver_interface->resolve();
    double h_val = lp_solver_interface->getObjValue();
    // TODO avoid code duplication with landmark count heuristic
    double epsilon = 0.01;
    return ceil(h_val - epsilon);
}

void PDBOptimalPartitioningHeuristic::generateLP(const vector<vector<int> > &patterns) {
    // Build the following LP
    //
    // Variables:
    //  * heuristic[p] for each p in PDBs
    //  * distance[p][s'] for each p in PDBs and each s' in the abstract states of p
    //  * action_cost[p][a] for each p in PDBs and each action a
    //
    // Objective Function: MAX sum_{p in PDBs} heuristic[p]
    //
    // Constraints:
    //  * For p in PDBs
    //    * For <s'', a, s'> in abstract transitions of PDB p
    //        distance[p][s'] <= distance[p][s''] + action_cost[p][a]
    //    * For each abstract goal state s' of PDB p
    //        heuristic[p] <= distance[p][s']
    //  * For a in actions
    //        sum_{p in PDBs} action_cost[p][a] <= a.cost
    //
    // Lower bounds: All varaibles must be non-negative
    // Upper bounds:
    //  * heuristic[p] <= \infty
    //  * action_cost[p][a] <= \infty (we could also use a.cost but this information
    //                                 is already contained in the constraints)
    //  * (Only) the bounds for distance[p][s'] depend on the current state s
    //    and will be changed for every evaluation
    //    * distance[p][s'] <= 0       if the abstraction of s in p is s'
    //    * distance[p][s'] <= \infty  otherwise
    clear_pdbs();
    vector<MatrixEntry> matrix_entries;
    vector<double> constraint_upper_bounds;
    // No need to store lower bounds (they are all 0)

    for (size_t i = 0; i < patterns.size(); ++i) {
        Options pdb_opts;
        pdb_opts.set<int>("cost_type", cost_type);
        pdb_opts.set<vector<int> >("pattern", patterns[i]);
        pdb_opts.set<bool>("store_transition_system", true);
        PDBHeuristic *h = new PDBHeuristic(pdb_opts, false);
        introduce_pdb_variables(h, i);
        add_pdb_constraints(h, i, matrix_entries, constraint_upper_bounds);
        h->clear_transition_system();
        pattern_databases.push_back(h);
    }
    add_action_cost_constraints(matrix_entries, constraint_upper_bounds);

    int matrix_entry_count = matrix_entries.size();
    int *rowIndices = new int[matrix_entry_count];
    int *colIndices = new int[matrix_entry_count];
    double *elements = new double[matrix_entry_count];
    for (int i = 0; i < matrix_entry_count; ++i) {
        rowIndices[i] = matrix_entries[i].row;
        colIndices[i] = matrix_entries[i].col;
        elements[i] = matrix_entries[i].element;
    }
    CoinPackedMatrix matrix(false, rowIndices, colIndices, elements, matrix_entry_count);

    int n_rows = constraint_count;
    int n_cols = variable_count;

    // Maxmize: sum heuristic_p
    double *objective    = new double[n_cols]; //objective coefficients
    fill(objective, objective + n_cols, 0);
    for (size_t pdb_id = 0; pdb_id < heuristic_variables.size(); ++pdb_id) {
        objective[heuristic_variables[pdb_id]] = 1;
    }
    lp_solver_interface->setObjSense(-1);

    // lower/upper bounds
    double *col_lb       = new double[n_cols]; //column lower bounds
    fill(col_lb, col_lb + n_cols, 0);
    double *col_ub       = new double[n_cols]; //column upper bounds
    fill(col_ub, col_ub + n_cols, lp_solver_interface->getInfinity());
    double *row_lb = new double[n_rows]; //the row lower bounds
    fill(row_lb, row_lb + n_rows, 0);
    double *row_ub = new double[n_rows]; //the row upper bounds
    for (size_t i = 0; i < constraint_upper_bounds.size(); ++i) {
        row_ub[i] = constraint_upper_bounds[i];
    }

    lp_solver_interface->loadProblem(matrix, col_lb, col_ub, objective,
                                     row_lb, row_ub);
    // Clean up
    delete[] rowIndices;
    delete[] colIndices;
    delete[] elements;
    delete[] objective;
    delete[] col_lb;
    delete[] col_ub;
    delete[] row_ub;
    delete[] row_lb;
}

void PDBOptimalPartitioningHeuristic::introduce_pdb_variables(const PDBHeuristic *h, int pdb_id) {
    assert(heuristic_variables.size() == pdb_id);
    assert(distance_variables.size() == pdb_id);
    assert(action_cost_variables.size() == pdb_id);
    heuristic_variables.push_back(variable_count++);
    int abstract_state_count = h->get_size();
    distance_variables.push_back(vector<int>(abstract_state_count));
    for (size_t state_id = 0; state_id < abstract_state_count; ++state_id) {
        distance_variables[pdb_id][state_id] = variable_count++;
    }
    action_cost_variables.push_back(vector<int>(g_operators.size()));
    for (size_t op_id = 0; op_id < g_operators.size(); ++op_id) {
        action_cost_variables[pdb_id][op_id] = variable_count++;
    }
}

void PDBOptimalPartitioningHeuristic::add_pdb_constraints(const PDBHeuristic *h, int pdb_id,
                                                          vector<MatrixEntry> &matrix_entries,
                                                          vector<double> &constraint_upper_bounds) {
    //    * For <s'', a, s'> in abstract transitions of PDB p
    //        distance[p][s'] <= distance[p][s''] + action_cost[p][a]
    //        0 <= distance[p][s''] + action_cost[p][a] - distance[p][s'] <= \infty
    const vector<AbstractPDBTransition> *transitions = h->get_abstract_transitions();
    assert(transitions);
    for (int t_id = 0; t_id < transitions->size(); ++t_id) {
        int row = constraint_count++;
        const AbstractPDBTransition &transition = (*transitions)[t_id];
        int from_col = distance_variables[pdb_id][transition.from_state_index];
        int op_col = action_cost_variables[pdb_id][transition.op_id];
        int to_col = distance_variables[pdb_id][transition.to_state_index];
        matrix_entries.push_back(MatrixEntry(row, from_col, 1));
        matrix_entries.push_back(MatrixEntry(row, op_col, 1));
        matrix_entries.push_back(MatrixEntry(row, to_col, -1));
        constraint_upper_bounds.push_back(lp_solver_interface->getInfinity());
    }

    //    * For each abstract goal state s' of PDB p
    //        heuristic[p] <= distance[p][s']
    //        0 <= distance[p][s'] - heuristic[p] <= \infty
    const vector<int> *goal_states = h->get_abstract_goal_states();
    assert(goal_states);
    int heuristic_col = heuristic_variables[pdb_id];
    for (size_t i = 0; i < goal_states->size(); ++i) {
        int row = constraint_count++;
        int goal_col = distance_variables[pdb_id][(*goal_states)[i]];
        matrix_entries.push_back(MatrixEntry(row, goal_col, 1));
        matrix_entries.push_back(MatrixEntry(row, heuristic_col, -1));
        constraint_upper_bounds.push_back(lp_solver_interface->getInfinity());
    }
}

void PDBOptimalPartitioningHeuristic::add_action_cost_constraints(vector<MatrixEntry> &matrix_entries,
                                                                  vector<double> &constraint_upper_bounds) {
    //  * For a in actions
    //        0 <= sum_{p in PDBs} action_cost[p][a] <= a.cost
    for (size_t op_id = 0; op_id < g_operators.size(); ++op_id) {
        int row = constraint_count++;
        for (size_t pdb_id = 0; pdb_id < action_cost_variables.size(); ++pdb_id) {
            int pdb_col = action_cost_variables[pdb_id][op_id];
            matrix_entries.push_back(MatrixEntry(row, pdb_col, 1));
        }
        constraint_upper_bounds.push_back(g_operators[op_id].get_cost());
    }
}

static ScalarEvaluator *_parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    parser.add_option<int>("systematic", 1, "Systematically generate all patterns with up to n variables.");
    parser.add_option<bool>("prune_irrelevant_patterns", true,
                            "Prune irrelevant patterns before building the LP.");
    Options opts = parser.parse();

    if (parser.dry_run())
        return 0;

    Options heuristic_opts;
    heuristic_opts.set<int>("cost_type", OperatorCost(opts.get<int>("cost_type")));
    Options generator_opts;
    generator_opts.set<int>("pattern_max_size", opts.get<int>("systematic"));
    if (opts.contains("prune_irrelevant_patterns") && opts.get<bool>("prune_irrelevant_patterns")) {
        PatternGenerationSystematic pattern_generator(generator_opts);
        heuristic_opts.set<vector<vector<int> > >("patterns", pattern_generator.get_patterns());
    } else {
        PatternGenerationSystematicNaive pattern_generator(generator_opts);
        heuristic_opts.set<vector<vector<int> > >("patterns", pattern_generator.get_patterns());
    }

    return new PDBOptimalPartitioningHeuristic(heuristic_opts);
}

static ScalarEvaluator *_parse_with_ipdb_patterns(OptionParser &parser) {
    PatternGenerationHaslum::create_options(parser);
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    PatternGenerationHaslum::sanity_check_options(parser, opts);

    if (parser.dry_run())
        return 0;

    PatternGenerationHaslum pgh(opts);

    CanonicalPDBsHeuristic *canonical = pgh.get_pattern_collection_heuristic();
    const vector<PDBHeuristic *> &pdbs = canonical->get_pattern_databases();
    vector<vector<int> > patterns;
    for (size_t i = 0; i < pdbs.size(); ++i) {
        patterns.push_back(pdbs[i]->get_pattern());
    }
    Options heuristic_opts;
    heuristic_opts.set<int>("cost_type", OperatorCost(opts.get<int>("cost_type")));
    heuristic_opts.set<vector<vector<int> > >("patterns", patterns);

    return new PDBOptimalPartitioningHeuristic(heuristic_opts);
}

static Plugin<ScalarEvaluator> _plugin("pdb_opt", _parse);
static Plugin<ScalarEvaluator> _plugin_ipdb("pdb_opt_with_ipdb_patterns", _parse_with_ipdb_patterns);
