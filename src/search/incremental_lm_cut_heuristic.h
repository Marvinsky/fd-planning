#ifndef INCREMENTAL_LM_CUT_HEURISTIC_H
#define INCREMENTAL_LM_CUT_HEURISTIC_H

#include "lm_cut_heuristic.h"
#include "state_proxy.h"
#include "search_space.h"
#include "memory_tracking_allocator.h"

#include <boost/shared_ptr.hpp>
#include <ext/hash_map>

// wraps vector<RelaxedOperator *> to add costs
// adding or removing operators after creation does *not* update costs (not implemented)
class Landmark : public vector<RelaxedOperator *, MemoryTrackingAllocator<RelaxedOperator *> > {
public:
    int cost;
    Landmark(vector<RelaxedOperator *> &operators, int cost);
};

typedef boost::shared_ptr<Landmark> LandmarkPtr;
typedef vector<LandmarkPtr, MemoryTrackingAllocator<LandmarkPtr> > SavedLandmarks;



typedef __gnu_cxx::hash_map<StateProxy,
                            SavedLandmarks,
                            __gnu_cxx::hash<StateProxy>,
                            equal_to<StateProxy>,
                            MemoryTrackingAllocator<std::pair<const StateProxy,
                                                              SavedLandmarks> > >
        SavedLandmarksMap;

template<class T1, class T2>
struct ComparePairByFirstComponent {
    bool operator()(pair<T1, T2> const &lhs, pair<T1, T2> const &rhs) {
        return lhs.first < rhs.first;
    }
};

class IncrementalLandmarkCutHeuristic : public LandmarkCutHeuristic {
private:
    bool keep_frontier;
    bool reevaluate_parent;
    unsigned long memory_limit_lmcut;
    SavedLandmarksMap saved_landmarks_map;
    SavedLandmarksMap saved_landmarks_map2;
    void remove_meta_information(StateProxy &state_proxy) {
        saved_landmarks_map.erase(state_proxy);
    }
protected:
    virtual void initialize();
    virtual bool reach_state(const State &parent_state, const Operator &op,
                             const State &state);
    virtual void finished_state(const State &state, int f, bool is_on_frontier);

    virtual void discovered_landmark(const State &state, vector<RelaxedOperator *> &landmark, int cost);
    virtual void reset_operator_costs(const State &state);
    virtual int compute_heuristic(const State &state);
public:
    IncrementalLandmarkCutHeuristic(const Options &opts);
    virtual ~IncrementalLandmarkCutHeuristic();
    virtual void free_up_memory(SearchSpace &search_space);
    virtual void free_up_memory();
    virtual void print_heur_name() {cout<<",heur:incremental_lmcut"<<",keep_frontier:"<<keep_frontier<<",reevaluate_parent"<<reevaluate_parent<<",memory_limit:"<<memory_limit_lmcut<<endl;}
    virtual string get_heur_name() {string temp="incremental_lmcut";return temp;}
    virtual string get_heur_call_name() {string temp="incremental_lmcut(reevaluate_parent=true)";return temp;}
    virtual void backup_landmarks() {
      cout<<"backing_up saved_landmarks_map"<<saved_landmarks_map.size()<<endl;
        saved_landmarks_map2=saved_landmarks_map;
    }
    virtual void erase_all_landmarks() {
        saved_landmarks_map2.clear();
        saved_landmarks_map.clear();
    }
    virtual void erase_current_landmarks() {
        saved_landmarks_map.clear();
    }
    virtual void erase_backup() {
      saved_landmarks_map2.clear();
    }  
    virtual void restore_backed_up_landmarks() {
      cout<<"restore saved_landmarks_map to pre-sampling size "<<saved_landmarks_map2.size()<<endl;
      saved_landmarks_map=saved_landmarks_map2;
      saved_landmarks_map2.clear();
    }
    virtual void set_keep_frontier(bool status) {keep_frontier=status;}
};


#endif
