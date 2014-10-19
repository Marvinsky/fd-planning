#ifndef PDBS_ZERO_ONE_PDBS_HEURISTIC_H
#define PDBS_ZERO_ONE_PDBS_HEURISTIC_H

#include "../heuristic.h"

#include <vector>
//#include  <boost/lexical_cast.hpp>
#include <sstream>
#include <iomanip>



class PDBHeuristic;

class ZeroOnePDBsHeuristic : public Heuristic {
    // summed up mean finite h-values of all PDBs - this is an approximation only, see get-method
    double approx_mean_finite_h;
    std::vector<PDBHeuristic *> pattern_databases; // final pattern databases
    double gapdb_heur_TPN;
    bool disjoint_patterns;
    double mutation_rate;
protected:
    virtual void initialize();
public:
    virtual int compute_heuristic(const State &state);
    ZeroOnePDBsHeuristic(const Options &opts,
                         const std::vector<int> &op_costs = std::vector<int>());
    virtual ~ZeroOnePDBsHeuristic();
    /* Returns the sum of all mean finite h-values of every PDB.
       This is an approximation of the real mean finite h-value of the Heuristic, because dead-ends are ignored for
       the computation of the mean finite h-values for a PDB. As a consequence, if different PDBs have different states
       which are dead-end, we do not calculate the real mean h-value for these states. */
    double get_approx_mean_finite_h() const {return approx_mean_finite_h; }
    void dump() const;
    virtual void print_heur_name() {cout<<"heur is GAPDB,mutation_rate:"<<mutation_rate;if(disjoint_patterns){cout<<",with disjoint patterns";}else{cout<<",without disjoint patterns";}};
    virtual string get_heur_name() {string temp="heur is GAPDB,mutation_rate:";
      std::ostringstream ss;ss << std::fixed << std::setprecision(2);ss<<mutation_rate;temp+=ss.str();
      if(disjoint_patterns){temp+=",with disjoint patterns";}else{temp+=",without disjoint patterns";};return temp;}
virtual string get_heur_call_name() {string temp="gapdb(mutation_probability=";
    std::ostringstream ss;ss << std::fixed << std::setprecision(2);ss<<mutation_rate;temp+=ss.str();
    temp+=",pdb_max_size=200000,num_episodes=30,num_collections=5";if(disjoint_patterns){temp+=",disjoint=true)";}else{temp+=")";};return temp;}
//    virtual string get_heur_call_name(); 
    virtual void get_patterns(string &patterns);
    virtual void free_up_memory();
};

#endif
