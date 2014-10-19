#include "incremental_lm_cut_heuristic.h"

#include "globals.h"
#include "operator.h"
#include "option_parser.h"
#include "plugin.h"
#include "state.h"
#include "state_var_t.h"

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>
#include "utilities.h"
using namespace std;

Landmark::Landmark(vector<RelaxedOperator *> &operators, int cost) :
    vector<RelaxedOperator *, MemoryTrackingAllocator<RelaxedOperator *> >(operators.begin(), operators.end()),
    cost(cost) {
}

// construction and destruction
IncrementalLandmarkCutHeuristic::IncrementalLandmarkCutHeuristic(const Options &opts)
    : LandmarkCutHeuristic(opts),
      keep_frontier(opts.get<bool>("keep_frontier")),
      reevaluate_parent(opts.get<bool>("reevaluate_parent")),
      memory_limit_lmcut(opts.get<unsigned int>("memory_limit_lmcut")) {
}

IncrementalLandmarkCutHeuristic::~IncrementalLandmarkCutHeuristic() {
}

void IncrementalLandmarkCutHeuristic::initialize() {
    LandmarkCutHeuristic::initialize();

    cout << "Landmarks for node on the search frontier will "
         << (keep_frontier ? "not " : "") << "be removed" << endl;

    cout << "For nodes without saved parent landmarks the parent node will "
         << (reevaluate_parent ? "" : "not ") << "be reevaluated" << endl;

    if (memory_limit_lmcut == 0) {
        memory_limit_lmcut = numeric_limits<unsigned long>::max();
        cout << "Using no memory limit" << endl;
    } else {
      incremental_memory_limit=true;//so we need to check for every new node in each search algorithm
        // convert MB in byte
        cout << "Memory limit is " << memory_limit_lmcut * 1024 * 1024 << " bytes (" << memory_limit_lmcut << " MB). Meta information will be pruned if it takes up more space." << endl;
        memory_limit_lmcut *= 1024 * 1024;
    }
}

void IncrementalLandmarkCutHeuristic::reset_operator_costs(const State &state) {
    // Redo cost changes of current landmarks (if there are no current landmarks,
    // this will just reset all costs to base costs)
    LandmarkCutHeuristic::reset_operator_costs(state);
    SavedLandmarks &current_landmarks = saved_landmarks_map[StateProxy(&state)];
    for (SavedLandmarks::iterator it_landmark = current_landmarks.begin(); it_landmark != current_landmarks.end(); ++it_landmark) {
        LandmarkPtr landmark = *it_landmark;
        for (Landmark::iterator it_op = landmark->begin(); it_op != landmark->end(); ++it_op) {
            (*it_op)->cost -= landmark->cost;
        }
    }
}

bool IncrementalLandmarkCutHeuristic::reach_state(const State &parent_state, const Operator &op, const State &state) {
    StateProxy parent_proxy = StateProxy(&parent_state);
    SavedLandmarksMap::iterator parent_node_info_it = saved_landmarks_map.find(parent_proxy);
    if (parent_node_info_it == saved_landmarks_map.end()) {
        if (reevaluate_parent) {
            compute_heuristic(parent_state);
            parent_node_info_it = saved_landmarks_map.find(parent_proxy);
            assert(parent_node_info_it != saved_landmarks_map.end());
            // now the information for the parent node has to exist. continue as planned
        } else {
            return true;
        }
    }

    bool h_dirty = false;

    SavedLandmarks &parent_landmarks = parent_node_info_it->second;
    SavedLandmarks &current_landmarks = saved_landmarks_map[StateProxy(&state)];

    // Copy all landmarks that do not mention op
    for (SavedLandmarks::iterator it_landmark = parent_landmarks.begin(); it_landmark != parent_landmarks.end(); ++it_landmark) {
        LandmarkPtr landmark = *it_landmark;
        bool copy_landmark = true;
        for (Landmark::iterator it_op = landmark->begin(); it_op != landmark->end(); ++it_op) {
            const Operator *base_operator = (*it_op)->op;
            if (base_operator == &op) {
                copy_landmark = false;
                h_dirty = true;
                break;
            }
        }
        if (copy_landmark) {
            current_landmarks.push_back(landmark);
        }
    }
    return h_dirty;
}

void IncrementalLandmarkCutHeuristic::finished_state(const State &state, int /*f*/,
                                                     bool is_on_frontier) {
  if(!is_using()){
    return;
  }
    StateProxy proxy(&state);
    if (is_on_frontier && keep_frontier) {
        // TODO could also keep only a fixed amount of states (keep lower f values)
        return;
    } else {
        remove_meta_information(proxy);
    }
}

void IncrementalLandmarkCutHeuristic::discovered_landmark(const State &state, vector<RelaxedOperator *> &landmark, int cost) {
    LandmarkPtr createdLandmark(new Landmark(landmark, cost));
    saved_landmarks_map[StateProxy(&state)].push_back(createdLandmark);
}

int IncrementalLandmarkCutHeuristic::compute_heuristic(const State &state) {
      /* if(Current_RIDA_Phase==SOLVING_PHASE){
	cout<<"starting compute incremental lmcut"<<endl;fflush(stdout);
	cout<<"State:"<<endl;fflush(stdout);state.dump_pddl();fflush(stdout);
      }*/
      StateProxy state_proxy(&state);
      /*  if(Current_RIDA_Phase==SOLVING_PHASE){
	cout<<"Checking for dead end:"<<endl;fflush(stdout);
      }*/
    if (LandmarkCutHeuristic::compute_heuristic(state) == DEAD_END) {
      /*  if(Current_RIDA_Phase==SOLVING_PHASE){
	cout<<"heuristic is dead_end"<<endl;fflush(stdout);
      }*/
        remove_meta_information(state_proxy);
	//cout<<"meta-information removed"<<endl;fflush(stdout);
        return DEAD_END;
    }

    int result = 0;
    SavedLandmarks &landmarks = saved_landmarks_map[state_proxy];
      /*  if(Current_RIDA_Phase==SOLVING_PHASE){
	cout<<"landmarks size"<<landmarks.size()<<endl;fflush(stdout);
      }*/
    for (SavedLandmarks::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
      /*  if(Current_RIDA_Phase==SOLVING_PHASE){
	//cout<<"result:"<<result<<endl;fflush(stdout);
      }*/
      //HACK, we need to find out what is happening that very rarely results in deleted landmarks while doing IDA* sampling
      //For the IPC, we simply use regular lmcut as a return value for this state
      /*  if((*it)==NULL){
	cout<<"Pointer to saved landmark is null!, using instead regular lmcut"<<endl;fflush(stdout);
	result=LandmarkCutHeuristic::compute_heuristic(state);
	cout<<"final result:"<<result<<endl;
	return result;
      }*/
      /*  if(Current_RIDA_Phase==SOLVING_PHASE){
	cout<<"result:"<<result<<endl;fflush(stdout);
	cout<<"(*it)->cost:";fflush(stdout);cout<<(*it)->cost<<endl;fflush(stdout);
      }*/
        result += (*it)->cost;
    }
    /*  if(Current_RIDA_Phase==SOLVING_PHASE){
      cout<<"final result:"<<result<<endl;fflush(stdout);
    }*/
    return result;
}

void IncrementalLandmarkCutHeuristic::free_up_memory(SearchSpace &search_space) {
    if (g_memory_tracking_allocated < memory_limit_lmcut) {
        return;
    }
    int n = saved_landmarks_map.size();
    cout << "Freeing memory by deleting meta information ... ";fflush(stdout);
    // try to find the nodes with maximal f value
    vector<pair<int, StateProxy> > states_by_f_value;
    states_by_f_value.reserve(n);
    for (SavedLandmarksMap::iterator it = saved_landmarks_map.begin(); it != saved_landmarks_map.end(); ++it) {
        StateProxy s = it->first;
        SearchNode n = search_space.get_node(State(s.state_data));
        // TODO this could lead to problem with multiple heuristics because n.get_h(); will return the value of only one of the heuristics
        int f = n.get_real_g() + n.get_h();
        pair<int, StateProxy> p = make_pair(f, s);
        states_by_f_value.push_back(p);
    }
    sort(states_by_f_value.begin(), states_by_f_value.end(), ComparePairByFirstComponent<int, StateProxy>());
    for (int i = states_by_f_value.size() - 1; i >= 0 && g_memory_tracking_allocated > memory_limit_lmcut / 2; --i) {
        StateProxy s = states_by_f_value[i].second;
        remove_meta_information(s);
    }
    cout << "done (removed info for " << n - saved_landmarks_map.size() << " of " << n << " states)" << endl;
}
void IncrementalLandmarkCutHeuristic::free_up_memory() {
  cout<<"\tmemory before deleting saved landmarks"<< get_memory_VmRSS() << " KB" << endl;
  erase_all_landmarks();
  cout<<"\tmemory after deleting saved landmarks"<< get_memory_VmRSS() << " KB" << endl;
}


static ScalarEvaluator *_parse(OptionParser &parser) {
    parser.add_option<unsigned int>("memory_limit_lmcut",  0, "Maximum amount of memory (in MB) used for storing meta information. Use 0 (default) for no limit.");
    parser.add_option<bool>("keep_frontier",     false, "Landmarks for nodes on the search frontier are not removed if the node is closed.");
    parser.add_option<bool>("reevaluate_parent", false, "If a node is encountered where the parent node has no saved landmarks, the heuristic for the parent is recomputed. This is based on the assumption that siblings of this node will be expanded soon and will profit from the calculated parent info");
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new IncrementalLandmarkCutHeuristic(opts);
}


static Plugin<ScalarEvaluator> _plugin("incremental_lmcut", _parse);
