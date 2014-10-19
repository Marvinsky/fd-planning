#ifndef PDB_LP_HEURISTIC_H
#define PDB_LP_HEURISTIC_H

#include "heuristic.h"
#include "option_parser.h"
#include "timer.h"
#include <sstream>

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

class CanonicalPDBsHeuristic;
class PDBHeuristic;

class PDBLPHeuristic : public Heuristic {
    std::vector<PDBHeuristic*> heuristics;
    bool borrowed_heuristics;
    CanonicalPDBsHeuristic *canonical;
    int size;
    int pattern_max_size;
    const int time_limit;
    Timer timer;
#ifdef USE_LP
    OsiXxxSolverInterface *lp_solver_interface;
#endif
    void generateLP();

protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    PDBLPHeuristic(const Options &opts);
    ~PDBLPHeuristic();

    void clear_pdbs();
    void set_pdbs(std::vector<PDBHeuristic*> heuristics, bool borrowed_heuristics);
    const std::vector<PDBHeuristic *> &get_pattern_databases() const { return heuristics; }
    int get_size() const { return size; }
    // WARNING: For now, this is not incremental and just recomputes everything from scratch.
    // This could be easily changed to accept a PDBHeuristic instead but for now we prefer the
    // better comparability with iPDB.
    void add_pattern(const std::vector<int> &pattern);
    virtual void print_heur_name(){cout<<"heur:,lp_pdb,systematic,"<<pattern_max_size;}
    virtual string get_heur_name(){string temp="heur:,lp_pdb,systematic:";stringstream out;out<<pattern_max_size;temp+=out.str();return temp;}
    virtual string get_heur_call_name(){string temp="pdb_lp(systematic=";stringstream out;out<<pattern_max_size<<",time_limit="<<time_limit<<")";temp+=out.str();return temp;}
    virtual int get_systematic_index(){return pattern_max_size;};
    virtual void free_up_memory();
};

#endif
