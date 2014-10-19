#ifndef PATTERN_GENERATION_LP_H
#define PATTERN_GENERATION_LP_H

#include "../operator_cost.h"
#include "../option_parser.h"

#include <map>
#include <vector>

class Options;
class PDBLPHeuristic;
class PDBHeuristic;
class State;

class PatternGenerationLP {
    const int pdb_max_size; // maximum number of states for each pdb
    const int collection_max_size; // maximum added size of all pdbs
    const int num_samples;
    const int min_improvement; // minimal improvement required for hill climbing to continue search
    const OperatorCost cost_type;
    PDBLPHeuristic *current_heuristic;
    PDBLPHeuristic *temporary_heuristic; // used to avoid recomputation during sample evaluation

    int num_rejected; // for stats only

    /* For the given pattern, all possible extensions of the pattern by one relevant variable
       are inserted into candidate_patterns. This may generate duplicated patterns. */
    void generate_candidate_patterns(const std::vector<int> &pattern,
                                     std::vector<std::vector<int> > &candidate_patterns);

    /* Performs num_samples random walks with a length (different for each random walk) chosen
       according to a binomial distribution with n = 4 * solution depth estimate and p = 0.5,
       starting from the initial state. In each step of a random walk, a random operator is taken
       and applied to the current state. If a dead end is reached or no more operators are
       applicable, the walk starts over again from the initial state. At the end of each random
       walk, the last state visited is taken as a sample state, thus totalling exactly
       num_samples of sample states. */
    void sample_states(std::vector<State> &samples, double average_operator_costs);

    /* Returns true iff the h-value of the current heuristic is smaller than the
       h-value of the current heuristic after adding pdb_heuristic. */
    bool is_heuristic_improved(PDBHeuristic *pdb_heuristic, const State &sample);

    /* This is the core algorithm of this class. As soon as after an iteration, the improvement (according
       to the "counting approximation") is smaller than the minimal required improvement, the search is
       stopped. This method uses a vector to store PDBs to avoid recomputation of the same PDBs later.
       This is quite a large time gain, but may use too much memory. Also a set is used to store all
       patterns in their "normal form" for duplicate detection. */
    void hill_climbing(double average_operator_costs, std::vector<std::vector<int> > &initial_candidate_patterns);

    /* Initializes everything for the hill climbing algorithm. Note that the initial pattern collection
       (consisting of exactly one PDB for each goal variable) may break the maximum collection size limit,
       if the latter is set too small or if there are many goal variables with a large domain. */
    void initialize();
public:
    PatternGenerationLP(const Options &opts);
    virtual ~PatternGenerationLP();

    /* Returns the PDBLPHeuristic created by PatternGenerationLP.
       Important: caller owns the returned pointer and has to take care of its deletion. */
    PDBLPHeuristic *get_pattern_collection_heuristic() const { return current_heuristic; }
    static void create_options(OptionParser &parser);
    static void sanity_check_options(OptionParser &parser, Options &opts);
};


#endif
