#ifndef OPEN_LISTS_TIEBREAKING_OPEN_LIST_H
#define OPEN_LISTS_TIEBREAKING_OPEN_LIST_H

#include "open_list.h"
#include "../evaluator.h"

#include <deque>
#include <map>
#include <vector>
#include <utility>

class ScalarEvaluator;
class Options;
class OptionParser;

template<class Entry>
class TieBreakingOpenList : public OpenList<Entry> {
    typedef std::deque<Entry> Bucket;

    std::map<const std::vector<int>, Bucket> buckets;
    int size;

    std::vector<ScalarEvaluator *> evaluators;
    std::vector<int> last_evaluated_value;
    bool last_preferred;
    bool dead_end;
    bool first_is_dead_end;
    bool dead_end_reliable;
    bool allow_unsafe_pruning; // don't insert if main evaluator
    // says dead end, even if not reliably

    const std::vector<int> &get_value(); // currently not used
    int dimension() const;
protected:
    Evaluator *get_evaluator() {return this; }

public:
    TieBreakingOpenList(const Options &opts);
    TieBreakingOpenList(const std::vector<ScalarEvaluator *> &evals,
                        bool preferred_only, bool unsafe_pruning);
    ~TieBreakingOpenList();

    // open list interface
    int insert(const Entry &entry);
    void open_list_boundary_print(int f_boundary);
    int open_list_get_boundary_range();
    int open_list_get_next_boundary_range();
    std::pair<Entry,int> get_specific_f_boundary_states_and_depth(int position);
    std::pair<Entry,int> get_specific_next_f_boundary_states_and_depth(int position,int next_f_boundary);
    int get_F_boundaries_size();
    std::pair<Entry,int> get_specific_all_boundaries_state_and_depth(int position);
    int get_next_f_boundary();
    Entry remove_min(std::vector<int> *key = 0);
    bool empty() const;
    void clear();

    // tuple evaluator interface
    void evaluate(int g, bool preferred);
    void evaluate2(int g, int h);
    bool is_dead_end() const;
    bool dead_end_is_reliable() const;
    void get_involved_heuristics(std::set<Heuristic *> &hset);

    static OpenList<Entry> *_parse(OptionParser &parser);
};

#include "tiebreaking_open_list.cc"

// HACK! Need a better strategy of dealing with templates, also in the Makefile.

#endif
