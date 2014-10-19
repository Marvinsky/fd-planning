#include "initial_state_statistics.h"

#include "option_parser.h"
#include "plugin.h"
#include "exact_timer.h"

#include <cassert>
#include <set>

InitialStateStatistics::InitialStateStatistics(const Options &opts)
    : SearchEngine(opts),
      evaluator(opts.get<ScalarEvaluator *>("eval")) {
}

void InitialStateStatistics::initialize(){
    set<Heuristic *> hset;
    assert(evaluator);
    evaluator->get_involved_heuristics(hset);
    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
        heuristics.push_back(*it);
        search_progress.add_heuristic(*it);
    }

    assert(!heuristics.empty());

    ExactTimer timer;
    for (size_t i = 0; i < heuristics.size(); i++)
        heuristics[i]->evaluate(*g_initial_state);
    evaluator->evaluate(0, false);
    cout << "Initial state evaluation time: " << timer << endl;
    search_progress.inc_evaluated_states();
    search_progress.inc_evaluations(heuristics.size());

    if (evaluator->is_dead_end()) {
        cout << "Initial state is a dead end." << endl;
    } else {
        search_progress.get_initial_h_values();
    }
}

void InitialStateStatistics::statistics() const {
    search_progress.print_statistics();
}

static SearchEngine *_parse(OptionParser &parser) {
    parser.add_option<ScalarEvaluator *>("eval");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    InitialStateStatistics *engine = 0;
    if (!parser.dry_run()) {
        engine = new InitialStateStatistics(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin_astar("initial_state_statistics", _parse);
