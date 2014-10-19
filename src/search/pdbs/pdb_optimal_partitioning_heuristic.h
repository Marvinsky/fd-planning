#ifndef PDBS_PDB_OPTIMAL_PARTITIONING_HEURISTIC_H
#define PDBS_PDB_OPTIMAL_PARTITIONING_HEURISTIC_H

#include "pdb_heuristic.h"

#include "../heuristic.h"
#include "../option_parser.h"

#include <vector>

#ifdef USE_LP
#pragma GCC diagnostic ignored "-Wunused-parameter"
#ifdef COIN_USE_CLP
#include "OsiClpSolverInterface.hpp"
typedef OsiClpSolverInterface OsiXxxSolverInterface;
#endif

#ifdef COIN_USE_OSL
#include "OsiOslSolverInterface.hpp"
typedef OsiOslSolverInterface OsiXxxSolverInterface;
#include "ekk_c_api.h"
#endif

#ifdef COIN_USE_CPX
#include "OsiCpxSolverInterface.hpp"
typedef OsiCpxSolverInterface OsiXxxSolverInterface;
#endif

#include "CoinPackedVector.hpp"
#include "CoinPackedMatrix.hpp"
#include <sys/times.h>
#endif

class PDBHeuristic;
class PDBOptimalPartitioningHeuristic : public Heuristic {
    class MatrixEntry {
    public:
        int row;
        int col;
        double element;
        MatrixEntry(int row_, int col_, double element_)
            : row(row_), col(col_), element(element_) {
        }
    };

    std::vector<PDBHeuristic *> pattern_databases;
#ifdef USE_LP
    OsiXxxSolverInterface *lp_solver_interface;
#endif
    // Column indices for heuristic variables indexed by PDB id.
    // The variable with id heuristic_variables[p] encodes the shortest distance
    // of the current abstract state to its nearest abstract goal state in pdb p
    // using the cost partitioning.
    std::vector<int> heuristic_variables;
    // Column indices for distance variables indexed by PDB id and abstract state id.
    // The variable with id distance_variables[p][s] encodes the distance of abstract
    // state s in pdb p from the current abstract state using the cost partitioning.
    std::vector<std::vector<int> > distance_variables;
    // Column indices for action cost variables indexed by PDB id and operator id.
    // The variable with id action_cost_variables[p][a] encodes cost action a should
    // have in pdb p.
    std::vector<std::vector<int> > action_cost_variables;

    int variable_count;
    int constraint_count;

    // Cache the variables corresponding to the current state in all pdbs.
    // This makes it easier to reset the bounds in each step.
    std::vector<int> current_abstract_state_vars;

    void generateLP(const std::vector<std::vector<int> > &patterns);
    void introduce_pdb_variables(const PDBHeuristic *h, int pdb_id);
    void add_pdb_constraints(const PDBHeuristic *h, int pdb_id,
                             std::vector<MatrixEntry> &matrix_entries,
                             std::vector<double> &constraint_upper_bounds);
    void add_action_cost_constraints(std::vector<MatrixEntry> &matrix_entries,
                                     std::vector<double> &constraint_upper_bounds);
    void clear_pdbs();
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    PDBOptimalPartitioningHeuristic(const Options &opts);
    ~PDBOptimalPartitioningHeuristic();
};

#endif
