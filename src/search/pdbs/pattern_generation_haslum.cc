#include "pattern_generation_haslum.h"

#include "canonical_pdbs_heuristic.h"
#include "pdb_heuristic.h"

#include "../globals.h"
#include "../legacy_causal_graph.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../rng.h"
#include "../state.h"
#include "../successor_generator.h"
#include "../timer.h"
#include "../utilities.h"
#include "../blind_search_heuristic.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

using namespace std;
    
bool ipdb_interruped=false;

PatternGenerationHaslum::PatternGenerationHaslum(const Options &opts)
    : pdb_max_size(opts.get<int>("pdb_max_size")),
      collection_max_size(opts.get<int>("collection_max_size")),
      num_samples(opts.get<int>("num_samples")),
      min_improvement(opts.get<int>("min_improvement")),
      cost_type(OperatorCost(opts.get<int>("cost_type"))),
      dominance_pruning(opts.get<bool>("dominance_pruning")),
      time_limit(opts.get<int>("time_limit")) {
    Timer timer;
    initialize();
    cout << "Pattern generation (Haslum et al.) time: " << timer << endl;
}

PatternGenerationHaslum::~PatternGenerationHaslum() {
}

void PatternGenerationHaslum::generate_candidate_patterns(const vector<int> &pattern,
                                                          set<vector<int> > &candidate_patterns) {
    int current_size = current_heuristic->get_size();
    for (size_t i = 0; i < pattern.size(); ++i) {
        // causally relevant variables for current variable from pattern
        vector<int> rel_vars = g_legacy_causal_graph->get_predecessors(pattern[i]);
        sort(rel_vars.begin(), rel_vars.end());
        vector<int> relevant_vars;
        assert_sorted_unique(rel_vars);
        assert_sorted_unique(pattern);
        // make sure we only use relevant variables which are not already included in pattern
        set_difference(rel_vars.begin(), rel_vars.end(), pattern.begin(), pattern.end(), back_inserter(relevant_vars));

        // add goal variables that occur as common effects with some variable in the pattern
        const vector<int> &common_goals = goal_common_action_effects[pattern[i]];
        vector<int> relevant_goals;
        assert_sorted_unique(common_goals);
        assert_sorted_unique(pattern);
        set_difference(common_goals.begin(), common_goals.end(), pattern.begin(), pattern.end(), back_inserter(relevant_goals));

        vector<int> interesting_vars;
        assert_sorted_unique(relevant_vars);
        assert_sorted_unique(relevant_goals);
        set_union(relevant_vars.begin(), relevant_vars.end(), relevant_goals.begin(), relevant_goals.end(), back_inserter(interesting_vars));


        for (size_t j = 0; j < interesting_vars.size(); ++j) {
            // test against overflow and pdb_max_size
            if (current_size <= pdb_max_size / g_variable_domain[interesting_vars[j]]) {
                // current_size * g_variable_domain[relevant_vars[j]] <= pdb_max_size
                vector<int> new_pattern(pattern);
                new_pattern.push_back(interesting_vars[j]);
                sort(new_pattern.begin(), new_pattern.end());
                candidate_patterns.insert(new_pattern);
            } else {
                // [commented out the message because it might be too verbose]
                // cout << "ignoring new pattern as candidate because it is too large" << endl;
                num_rejected += 1;
            }
        }
    }
}

void PatternGenerationHaslum::sample_states(vector<State> &samples, double average_operator_cost) {
    Timer timer;
    current_heuristic->evaluate(*g_initial_state);
    assert(!current_heuristic->is_dead_end());

    int h = current_heuristic->get_heuristic();
    int n;
    if (h == 0) {
        n = 10;
    } else {
        // Convert heuristic value into an approximate number of actions
        // (does nothing on unit-cost problems).
        // average_operator_cost cannot equal 0, as in this case, all operators
        // must have costs of 0 and in this case the if-clause triggers.
        int solution_steps_estimate = int((h / average_operator_cost) + 0.5);
        n = 4 * solution_steps_estimate;
    }
    double p = 0.5;
    // The expected walk length is np = 2 * estimated number of solution steps.
    // (We multiply by 2 because the heuristic is underestimating.)

    samples.reserve(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        // calculate length of random walk accoring to a binomial distribution
        int length = 0;
        for (int j = 0; j < n; ++j) {
            double random = g_rng(); // [0..1)
            if (random < p)
                ++length;
        }
	
	if (time_limit > 0 && timer() > (time_limit/2)) {
	  cout << "1/2 Time limit reached. No more sample generation." << endl;
	  break;
	}
       //cout<<"sample:"<<i<<",legnth:"<<length<<endl;	

        // random walk of length length
        State current_state(*g_initial_state);
        for (int j = 0; j < length; ++j) {
	  if (time_limit > 0 && timer() > (time_limit/2)) {
	    cout << "1/2 Time limit reached. No more sample generation." << endl;
	    break;
	  }
            vector<const Operator *> applicable_ops;
            g_successor_generator->generate_applicable_ops(current_state, applicable_ops);
            // if there are no applicable operators --> do not walk further
            if (applicable_ops.empty()) {
                break;
            } else {
                int random = g_rng.next(applicable_ops.size()); // [0..applicable_os.size())
                assert(applicable_ops[random]->is_applicable(current_state));
                current_state = State(current_state, *applicable_ops[random]);
                // if current state is dead-end, then restart with initial state
                current_heuristic->evaluate(current_state);
                if (current_heuristic->is_dead_end())
                    current_state = *g_initial_state;
            }
        }
        // last state of the random walk is used as sample
        samples.push_back(current_state);
    }
}

bool PatternGenerationHaslum::is_heuristic_improved(PDBHeuristic *pdb_heuristic,
                                                    const State &sample) {
    pdb_heuristic->evaluate(sample);
    if (pdb_heuristic->is_dead_end()) {
        return true;
    }
    int h_pattern = pdb_heuristic->get_heuristic(); // h-value of the new pattern
    vector<vector<PDBHeuristic *> > max_additive_subsets;
    current_heuristic->get_max_additive_subsets(pdb_heuristic->get_pattern(), max_additive_subsets);
    current_heuristic->evaluate(sample);
    int h_collection = current_heuristic->get_heuristic(); // h-value of the current collection heuristic
    for (size_t k = 0; k < max_additive_subsets.size(); ++k) { // for each max additive subset...
        int h_subset = 0;
        for (size_t l = 0; l < max_additive_subsets[k].size(); ++l) { // ...calculate its h-value
            max_additive_subsets[k][l]->evaluate(sample);
            assert(!max_additive_subsets[k][l]->is_dead_end());
            h_subset += max_additive_subsets[k][l]->get_heuristic();
        }
        if (h_pattern + h_subset > h_collection) {
            // return true if one max additive subest is found for which the condition is met
            return true;
        }
    }
    return false;
}

void PatternGenerationHaslum::hill_climbing(double average_operator_cost,
                                            set<vector<int> > &initial_candidate_patterns) {
    Timer timer;
    // stores all candidate patterns generated so far in order to avoid duplicates
    set<vector<int> > generated_patterns;
    // new_candidates is the set of new pattern candidates from the last call to generate_candidate_patterns
    set<vector<int> > &new_candidates = initial_candidate_patterns;
    // all candidate patterns are converted into pdbs once and stored
    vector<PDBHeuristic *> candidate_pdbs;
    int num_iterations = 0;
    size_t max_pdb_size = 0;
    num_rejected = 0;
    cout<<"Calling iPDB with time_limit:"<<time_limit<<endl;
    while (true) {
        num_iterations += 1;
        cout << "current collection size is " << current_heuristic->get_size() << endl;
        current_heuristic->evaluate(*g_initial_state);
        cout << "current initial h value: ";
        if (current_heuristic->is_dead_end()) {
            cout << "infinite => stopping hill-climbing" << endl;
            break;
        } else {
            cout << current_heuristic->get_heuristic() << endl;
        }
	cout<<"starting generating samples"<<endl;

        vector<State> samples;
        sample_states(samples, average_operator_cost);

        // For the new candidate patterns check whether they already have been candidates before and
        // thus already a PDB has been created an inserted into candidate_pdbs.
        for (set<vector<int> >::iterator it = new_candidates.begin(); it != new_candidates.end(); ++it) {
            const vector<int> &new_candidate = *it;
            if (generated_patterns.count(new_candidate) == 0) {
                Options opts;
                opts.set<int>("cost_type", cost_type);
                opts.set<vector<int> >("pattern", new_candidate);
                candidate_pdbs.push_back(new PDBHeuristic(opts, false));
                max_pdb_size = max(max_pdb_size,
                                   candidate_pdbs.back()->get_size());
                generated_patterns.insert(new_candidate);
            }
	    if (time_limit > 0 && timer() > (time_limit/2)) {
	      cout << "1/2 Time limit reached. No more candidate generation." << endl;
	      break;
	    } 
        }

        // TODO: The original implementation by Haslum et al. uses astar to compute h values for
        // the sample states only instead of generating all PDBs.
        int improvement = 0; // best improvement (= hightest count) for a pattern so far
        int best_pdb_index = 0;
	cout<<"\t starting improvement phase"<<endl;

        // Iterate over all candidates and search for the best improving pattern/pdb
        for (size_t i = 0; i < candidate_pdbs.size(); ++i) {
            PDBHeuristic *pdb_heuristic = candidate_pdbs[i];
            if (pdb_heuristic == 0) { // candidate pattern is too large
	      if (time_limit > 0 && timer() > (time_limit)) {
		cout << " Time limit reached. No more candidate generation." << endl;
		break;
	      } 
                continue;
            }
            // If a candidate's size added to the current collection's size exceeds the maximum
            // collection size, then delete the PDB and let the PDB's entry point to a null reference
            if (current_heuristic->get_size() + pdb_heuristic->get_size() > collection_max_size) {
                delete pdb_heuristic;
                candidate_pdbs[i] = 0;
                continue;
            }

            // Calculate the "counting approximation" for all sample states: count the number of
            // samples for which the current pattern collection heuristic would be improved
            // if the new pattern was included into it.
            // TODO: The original implementation by Haslum et al. uses m/t as a statistical
            // confidence intervall to stop the astar-search (which they use, see above) earlier.
            int count = 0;
            for (size_t j = 0; j < samples.size(); ++j) {
                if (is_heuristic_improved(pdb_heuristic, samples[j]))
                    ++count;
            }
            if (count > improvement) {
                improvement = count;
                best_pdb_index = i;
            }
            if (count > 0) {
                cout << "pattern: " << candidate_pdbs[i]->get_pattern()
                     << " - improvement: " << count << endl;
            }
	    if (time_limit > 0 && timer() > (time_limit)) {
	      cout << " Time limit reached. No more candidate generation." << endl;
	      break;
	    } 
        }
        bool stop_hill_climbing = true;
        if (improvement < min_improvement) {
            cout << "Improvement below threshold. Aborting hill climbing." << endl;
        } else if (time_limit > 0 && timer() > time_limit) {
            cout << "Time limit reached. Aborting hill climbing." << endl;
        } else {
            stop_hill_climbing = false;
        }
        if (stop_hill_climbing) { // end hill climbing algorithm
            // Note that using dominance pruning during hill-climbing could lead to
            // fewer discovered patterns and pattern collections.
            // A dominated pattern (collection) might no longer be dominated
            // after more patterns are added.
            if (dominance_pruning) {
                current_heuristic->dominance_pruning();
            }
            cout << "iPDB: iterations = " << num_iterations << endl;
            cout << "iPDB: num_patterns = "
                 << current_heuristic->get_pattern_databases().size() << endl;
            cout << "iPDB: size = " << current_heuristic->get_size() << endl;
            cout << "iPDB: improvement = " << improvement << endl;
            cout << "iPDB: generated = " << generated_patterns.size() << endl;
            cout << "iPDB: rejected = " << num_rejected << endl;
            cout << "iPDB: max_pdb_size = " << max_pdb_size << endl;
            break;
        }

        // add the best pattern to the CanonicalPDBsHeuristic
        const vector<int> &best_pattern = candidate_pdbs[best_pdb_index]->get_pattern();
        cout << "found a better pattern with improvement " << improvement << endl;
        cout << "pattern: " << best_pattern << endl;
        current_heuristic->add_pattern(best_pattern);

        // clear current new_candidates and get successors for next iteration
        new_candidates.clear();
        generate_candidate_patterns(best_pattern, new_candidates);

        // remove from candidate_pdbs the added PDB
        delete candidate_pdbs[best_pdb_index];
        candidate_pdbs[best_pdb_index] = 0;

        cout << "Hill-climbing time so far: " << timer << endl;
    }

    // delete all created PDB-pointer
    for (size_t i = 0; i < candidate_pdbs.size(); ++i) {
        delete candidate_pdbs[i];
    }
}

void PatternGenerationHaslum::initialize() {
    // calculate goal variables that occur as common effects with other variables
    goal_common_action_effects = vector<vector<int> >(g_variable_name.size());
    vector<bool> is_goal(g_variable_name.size(), false);
    for (size_t i = 0; i < g_goal.size(); ++i) {
        is_goal[g_goal[i].first] = true;
    }
    for (size_t i = 0; i < g_operators.size(); ++i) {
        const vector<PrePost> &pre_posts = g_operators[i].get_pre_post();
        for(size_t j = 0; j < pre_posts.size(); ++j) {
            int var1 = pre_posts[j].var;
            if (!is_goal[var1]) {
                continue;
            }
            for(size_t k = 0; k < pre_posts.size(); ++k) {
                int var2 = pre_posts[k].var;
                if (var1 == var2) {
                    continue;
                }
                vector<int> &common_eff = goal_common_action_effects[var2];
                // TODO avoid O(n) find
                if (find(common_eff.begin(), common_eff.end(), var1) == common_eff.end()) {
                    common_eff.push_back(var1);
                }
            }
        }
    }
    for (size_t i = 0; i < g_variable_name.size(); ++i) {
        vector<int> &common_eff = goal_common_action_effects[i];
        sort(common_eff.begin(), common_eff.end());
    }

    // calculate average operator costs
    double average_operator_cost = 0;
    for (size_t i = 0; i < g_operators.size(); ++i) {
        average_operator_cost += get_adjusted_action_cost(g_operators[i], cost_type);
    }
    average_operator_cost /= g_operators.size();
    cout << "Average operator cost: " << average_operator_cost << endl;

    // initial collection: a pdb for each goal variable
    vector<vector<int> > initial_pattern_collection;
    for (size_t i = 0; i < g_goal.size(); ++i) {
        initial_pattern_collection.push_back(vector<int>(1, g_goal[i].first));
    }
    Options opts;
    opts.set<int>("cost_type", cost_type);
    opts.set<vector<vector<int> > >("patterns", initial_pattern_collection);
    current_heuristic = new CanonicalPDBsHeuristic(opts);
    //The clique graph can bee too big, in that case get out.  Note that we stop expanding cliques once memory above canonical_max_memory+=initial_memory_usage
    if(get_memory_VmRSS()>canonical_max_memory){
      cout<<"stopping using canonical, memory usage is too high:"<<get_memory_VmRSS()<<",maximum:"<<canonical_max_memory<<endl;
      //set_stop_using(true);
      //note that if we completely delete current_heursitic object,we get core dump when calling get_involved_heuristic, HACK:we create empty canconical pdb. 
      delete current_heuristic;
      ipdb_interruped=true;
      Options opts2;
      opts2.set<int>("cost_type", cost_type);
      initial_pattern_collection.clear();//initial_pattern_collection.push_back(vector<int>(1, g_goal[0].first));
      opts2.set<vector<vector<int> > >("patterns", initial_pattern_collection);
      current_heuristic = new CanonicalPDBsHeuristic(opts2);
      cout<<"memory usage after deleting current_heuristic:"<<get_memory_VmRSS()<<endl;fflush(stdout);
      cout<<"Need to fix the need to create dummy current_heuristic"<<endl;
      return;
    }
    current_heuristic->evaluate(*g_initial_state);
    if (current_heuristic->is_dead_end())
        return;

    // initial candidate patterns, computed separately for each pattern from the initial collection
    set<vector<int> > initial_candidate_patterns;
    for (size_t i = 0; i < current_heuristic->get_pattern_databases().size(); ++i) {
        const vector<int> &current_pattern = current_heuristic->get_pattern_databases()[i]->get_pattern();
        generate_candidate_patterns(current_pattern, initial_candidate_patterns);
    }
    cout << "done calculating initial pattern collection and candidate patterns for the search" << endl;

    // call to this method modifies initial_candidate_patterns (contains the new_candidates
    // after each call to generate_candidate_patterns)
    hill_climbing(average_operator_cost, initial_candidate_patterns);
}

void PatternGenerationHaslum::create_options(OptionParser &parser) {
    parser.add_option<int>("pdb_max_size", 2000000,
                           "max number of states per pdb");
    parser.add_option<int>("collection_max_size", 20000000,
                           "max number of states for collection");
    parser.add_option<int>("num_samples", 1000, "number of samples");
    parser.add_option<int>("min_improvement", 10,
                           "minimum improvement while hill climbing");
    parser.add_option<bool>("dominance_pruning", true,
                            "Use dominance pruning to reduce number of patterns after hill climbing.");
    parser.add_option<int>("time_limit", 0,
                            "Time in seconds that should be spent on hill climbing. Use 0 (default) for no limit.");
}

void PatternGenerationHaslum::sanity_check_options(OptionParser &parser, Options &opts) {
    if (opts.get<int>("pdb_max_size") < 1)
        parser.error("size per pdb must be at least 1");
    if (opts.get<int>("collection_max_size") < 1)
        parser.error("total pdb collection size must be at least 1");
    if (opts.get<int>("min_improvement") < 1)
        parser.error("minimum improvement must be at least 1");
    if (opts.get<int>("min_improvement") > opts.get<int>("num_samples"))
        parser.error("minimum improvement must not be higher than number of samples");
}

static ScalarEvaluator *_parse(OptionParser &parser) {
    PatternGenerationHaslum::create_options(parser);
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    PatternGenerationHaslum::sanity_check_options(parser, opts);

    if (parser.dry_run())
        return 0;

    PatternGenerationHaslum pgh(opts);
    if(ipdb_interruped){
      cout<<"stopping using canonical, memory usage was too high:"<<endl;
      Heuristic::add_options_to_parser(parser);
      Options opts = parser.parse();
      BlindSearchHeuristic *dummy_heur =
		  new BlindSearchHeuristic(opts);
      dummy_heur->set_stop_using(true);
      return dummy_heur; 
    }
    else{
      return pgh.get_pattern_collection_heuristic();
    }
}


static Plugin<ScalarEvaluator> _plugin("ipdb", _parse);
