#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "scalar_evaluator.h"
#include "operator_cost.h"
#include "search_space.h"

#include <map>
#include <set>
#include <string>
#include <vector>
#include <iostream>
using namespace std;

class Operator;
class State;
class OptionParser;
class Options;

class Heuristic : public ScalarEvaluator {
    enum {NOT_INITIALIZED = -2};
    int heuristic;
    int evaluator_value; // usually equal to heuristic but can be different
    // if set with set_evaluator_value which is done if we use precalculated
    // estimates, eg. when re-opening a search node

    std::vector<const Operator *> preferred_operators;
    bool is_unit_cost;
    double measured_TPN;
    bool stop_using;
protected:
    OperatorCost cost_type;
    enum {DEAD_END = -1};
    virtual void initialize() {}
    virtual int compute_heuristic(const State &state) = 0;
    // Usage note: It's OK to set the same operator as preferred
    // multiple times -- it will still only appear in the list of
    // preferred operators for this heuristic once.
    void set_preferred(const Operator *op);
    int get_adjusted_cost(const Operator &op) const;
    bool is_unit_cost_problem() const {
        return is_unit_cost;
    }
public:
    void set_stop_using(bool status);
    bool is_using();
    virtual double get_approx_mean_finite_h() {return 0;};
    virtual double get_time_cost(){return 0.00000011;};
    virtual string get_pattern_id(){return "";};
    Heuristic(const Options &options);
    virtual ~Heuristic();

    void evaluate(const State &state);
    bool is_dead_end() const;
    int get_heuristic();
    // changed to virtual, so HeuristicProxy can delegate this:
    virtual void get_preferred_operators(std::vector<const Operator *> &result);
    virtual bool dead_ends_are_reliable() const {return true; }
    virtual bool reach_state(const State &parent_state, const Operator &op,
                             const State &state);
    // notify heuristic that this state will not be needed again (possible for
    // depth first searches)
    // a node on the frontier is pruned in IDA* because its f value is too high
    // if the search runs for another iteration it might be expanded
    virtual void finished_state(const State &/*state*/, int /*f*/,
                                bool /*is_on_frontier*/) {}
    // for abstract parent ScalarEvaluator
    virtual void backup_landmarks(){} ;
    virtual void erase_all_landmarks(){} ;
    virtual void erase_backup() {};
    virtual void restore_backed_up_landmarks() {};
    int get_value() const;
    void evaluate(int g, bool preferred);
    bool dead_end_is_reliable() const;
    void set_evaluator_value(int val);
    void get_involved_heuristics(std::set<Heuristic *> &hset) {hset.insert(this); }
    virtual void reset() {}
    virtual void print_heur_name() {cout<<"heur is not named";}
    virtual string get_heur_name() {string temp="No Name";return temp;}
    virtual string get_heur_call_name() {string temp="No Name";return temp;}
    OperatorCost get_cost_type() const {return cost_type; }
    virtual void free_up_memory(SearchSpace & /*search_space*/) {}
    virtual void free_up_memory() {}
    virtual int get_systematic_index(){return 0;};
    void set_measured_TPN(double input_TPN);
    double get_measured_TPN();
    virtual void set_keep_frontier(bool status) {if(status){}}//need to do something with status or we get unused error!

    static void add_options_to_parser(OptionParser &parser);
    static Options default_options();
    virtual void erase_current_landmarks() {};
    virtual void initialize_hack() {initialize();}
    virtual void get_patterns(string &patterns){patterns="";};
};

#endif
