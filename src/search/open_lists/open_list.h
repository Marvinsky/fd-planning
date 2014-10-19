#ifndef OPEN_LISTS_OPEN_LIST_H
#define OPEN_LISTS_OPEN_LIST_H
#pragma GCC diagnostic ignored "-Wuninitialized"

#include "../evaluator.h"
#include <vector>
#include <utility>
#include <iostream>
using namespace std;

template<class Entry>
class OpenList : public Evaluator {
protected:
    virtual Evaluator *get_evaluator() = 0;
    bool only_preferred;

public:
    OpenList(bool preferred_only = false) : only_preferred(preferred_only) {}
    virtual ~OpenList() {}

    virtual int insert(const Entry &entry) = 0;
    virtual void open_list_boundary_print(int f_boundary){if(f_boundary<0) return;return;}
    virtual int open_list_get_boundary_range(){return 0;};
    virtual int open_list_get_next_boundary_range(){return 0;};
    virtual void evaluate2(int g,int h){if(g+h>0) cout<<"calling virtual evaluate2,implement method"<<std::endl;return;return;};
    virtual std::pair<Entry,int> get_specific_f_boundary_states_and_depth(int position=0){
      Entry temp_entry;
      return std::make_pair(temp_entry,position);
    }
    virtual std::pair<Entry,int> get_specific_next_f_boundary_states_and_depth(int position=0,int f_boundary=0){
      Entry temp_entry;
      if(f_boundary){}//so not unused variable error
      return std::make_pair(temp_entry,position);
    }
    virtual std::pair<Entry,int> get_specific_all_boundaries_state_and_depth(int position=0){
      Entry temp_entry;
      return std::make_pair(temp_entry,position);
    }
    virtual int get_F_boundaries_size(){return 0;};
    virtual Entry remove_min(std::vector<int> *key = 0) = 0;
    // If key is non-null, it must point to an empty vector.
    // Then remove_min stores the key for the popped element there.
    // TODO: We might want to solve this differently eventually;
    //       see msg639 in the tracker.
    virtual bool empty() const = 0;
    virtual void clear() = 0;
    bool only_preferred_states() const {return only_preferred; }
    // should only be used within alternation open lists
    // a search does not have to care about this because
    // it is handled by the open list whether the entry will
    // be inserted

    virtual int boost_preferred() {return 0; }
    virtual void boost_last_used_list() {return; }
    virtual int get_next_f_boundary(){return 0;};
};

#endif
