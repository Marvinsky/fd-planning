// HACK! Ignore this if used as a top-level compile target.
#ifdef OPEN_LISTS_TIEBREAKING_OPEN_LIST_H

#include <iostream>
#include <cassert>
#include <limits>
#include <cstdio>
#include "../scalar_evaluator.h"
#include "../option_parser.h"
using namespace std;

/*
  Bucket-based implementation of a open list.
  Nodes with identical heuristic value are expanded in FIFO order.
*/

template<class Entry>
OpenList<Entry> *TieBreakingOpenList<Entry>::_parse(OptionParser &parser) {
    parser.add_list_option<ScalarEvaluator *>("evals");
    parser.add_option<bool>(
        "pref_only", false,
        "insert only preferred operators");
    parser.add_option<bool>(
        "unsafe_pruning", true,
        "allow unsafe pruning when the main evaluator regards a state a dead end");
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new TieBreakingOpenList<Entry>(opts);
}

template<class Entry>
TieBreakingOpenList<Entry>::TieBreakingOpenList(const Options &opts)
    : OpenList<Entry>(opts.get<bool>("pref_only")),
      size(0), evaluators(opts.get_list<ScalarEvaluator *>("evals")),
      allow_unsafe_pruning(opts.get<bool>("unsafe_pruning")) {
    last_evaluated_value.resize(evaluators.size());
}

template<class Entry>
TieBreakingOpenList<Entry>::TieBreakingOpenList(
    const std::vector<ScalarEvaluator *> &evals,
    bool preferred_only, bool unsafe_pruning)
    : OpenList<Entry>(preferred_only), size(0), evaluators(evals),
      allow_unsafe_pruning(unsafe_pruning) {
    last_evaluated_value.resize(evaluators.size());
}

template<class Entry>
TieBreakingOpenList<Entry>::~TieBreakingOpenList() {
}

template<class Entry>
int TieBreakingOpenList<Entry>::insert(const Entry &entry) {
    if (OpenList<Entry>::only_preferred && !last_preferred)
        return 0;
    if (first_is_dead_end && allow_unsafe_pruning) {
        return 0;
    }
    const std::vector<int> &key = last_evaluated_value;
    buckets[key].push_back(entry);
    size++;
    return 1;
}
template<class Entry>
void TieBreakingOpenList<Entry>::open_list_boundary_print(int f_boundary){
  typename std::map<const std::vector<int>, Bucket>::iterator it;
  for ( it = buckets.begin(); it != buckets.end();it++){
    if(it->first.at(0)!=f_boundary){
      continue;
    }
    cout<<"Open_List,F="<<it->first.at(0)<<",h="<<it->first.at(1)<<",size:"<<it->second.size()<<endl;
  }
}
template<class Entry>
int TieBreakingOpenList<Entry>::get_next_f_boundary(){
  if(buckets.size()==0){
    return 0;
  }
  typename std::map<const std::vector<int>, Bucket>::iterator it;
  
  it = buckets.begin();
  if(it->first.size()==0){
    return 0;
  }
  int f_boundary=it->first.at(0);
  cout<<"current_F_boundary:"<<f_boundary;fflush(stdout);
  for ( it = buckets.begin(); it != buckets.end();it++){
    if(it->first.size()==0){
      continue;
    }
    if(it->first.at(0)!=f_boundary){
      //cout<<"\tcounter:"<<counter<<",out of loop"<<endl;
      return it->first.at(0);
    }
  }
  return 0;
}
template<class Entry>
int TieBreakingOpenList<Entry>::open_list_get_boundary_range(){
  if(buckets.size()==0){
    return 0;
  }

  typename std::map<const std::vector<int>, Bucket>::iterator it;
  
  it = buckets.begin();
  if(it->first.size()==0){
    return 0;
  }
  int f_boundary=it->first.at(0);
  cout<<"F:"<<f_boundary<<endl;fflush(stdout);
  int counter=0;
  for ( it = buckets.begin(); it != buckets.end();it++){
    if(it->first.size()==0){
      continue;
    }
    if(it->first.at(0)!=f_boundary){
      //cout<<"\tcounter:"<<counter<<",out of loop"<<endl;
      break;
    }
    counter+=it->second.size();
  }
  //cout<<",range:"<<counter<<endl;
  return counter;
}
template<class Entry>
int TieBreakingOpenList<Entry>::open_list_get_next_boundary_range(){
  if(buckets.size()==0){
    return 0;
  }

  typename std::map<const std::vector<int>, Bucket>::iterator it;
  
  it = buckets.begin();
  int current_f_boundary=it->first.at(0);
  int next_f_boundary=current_f_boundary;
  for ( it = buckets.begin(); it != buckets.end();it++){
    next_f_boundary=it->first.at(0);
    if(next_f_boundary!=current_f_boundary){
      break;
    }
  }
  if(next_f_boundary==current_f_boundary){
    cout<<"next f-boundary not populated, still at:"<<next_f_boundary<<endl;
    return 0;
  }
  cout<<"Current f-boundary:"<<current_f_boundary<<",next_f_boundary"<<next_f_boundary<<endl;//open_list_boundary_print(next_f_boundary);
    
  if(it->first.size()==0){
    return 0;
  }
  int f_boundary=it->first.at(0);
  cout<<"F:"<<f_boundary<<",getting range"<<endl;fflush(stdout);
  int counter=0;
  for ( it = it; it != buckets.end();it++){
    if(it->first.size()==0){
      continue;
    }
    if(it->first.at(0)!=f_boundary){
      //cout<<"\tcounter:"<<counter<<",out of loop"<<endl;
      break;
    }
    counter+=it->second.size();
  }
  //cout<<",range:"<<counter<<endl;
  return counter;
}

template<class Entry>
std::pair<Entry,int> TieBreakingOpenList<Entry>::get_specific_f_boundary_states_and_depth(int position){
  typename std::map<const std::vector<int>, Bucket>::iterator it;
  it = buckets.begin();
  int f_boundary=it->first.at(0);
  for ( it = buckets.begin(); it != buckets.end();it++){
    if(it->first.at(0)!=f_boundary){
      cout<<"Range is wrong!"<<endl;exit(0);
    }
    if(position<it->second.size()){
      break;//we are finished, this is the right bucket
    }
    else{//need to keep lopping until we find the correct bucket to get the state from
      position-=it->second.size();
    }
  }
  Entry result = it->second.at(position);
  int g=it->first.at(0)-it->first.at(1);
  //cout<<"\tf:"<<it->first.at(0)<<",h:"<<it->first.at(1)<<",g:"<<g<<endl;
  return std::make_pair(result,g);
}
template<class Entry>
int TieBreakingOpenList<Entry>::get_F_boundaries_size(){
  int range=0;
  typename std::map<const std::vector<int>, Bucket>::iterator it;
  it = buckets.begin();
  for ( it = buckets.begin(); it != buckets.end();it++){
    range+=it->second.size();
  }
  return range;
}
template<class Entry>
std::pair<Entry,int> TieBreakingOpenList<Entry>::get_specific_all_boundaries_state_and_depth(int position){
  typename std::map<const std::vector<int>, Bucket>::iterator it;
  it = buckets.begin();
  for ( it = buckets.begin(); it != buckets.end();it++){
    if(position<it->second.size()){
      break;//we are finished, this is the right bucket
    }
    else{//need to keep lopping until we find the correct bucket to get the state from
      position-=it->second.size();
    }
  }
  Entry result = it->second.at(position);
  int g=it->first.at(0)-it->first.at(1);
  //cout<<"\tf:"<<it->first.at(0)<<",h:"<<it->first.at(1)<<",g:"<<g<<endl;
  return std::make_pair(result,g);
		
}
template<class Entry>
std::pair<Entry,int> TieBreakingOpenList<Entry>::get_specific_next_f_boundary_states_and_depth(int position,int next_f_boundary){
  typename std::map<const std::vector<int>, Bucket>::iterator it;
  it = buckets.begin();
  for ( it = buckets.begin(); it != buckets.end();it++){
    if(it->first.at(0)!=next_f_boundary){
      continue;
    }
    if(position<it->second.size()){
      break;//we are finished, this is the right bucket
    }
    else{//need to keep lopping until we find the correct bucket to get the state from
      position-=it->second.size();
    }
  }
  Entry result = it->second.at(position);
  int g=it->first.at(0)-it->first.at(1);
  //cout<<"\tf:"<<it->first.at(0)<<",h:"<<it->first.at(1)<<",g:"<<g<<endl;
  return std::make_pair(result,g);
}

template<class Entry>
Entry TieBreakingOpenList<Entry>::remove_min(vector<int> *key) {
    assert(size > 0);
    typename std::map<const std::vector<int>, Bucket>::iterator it;
    it = buckets.begin();
    assert(it != buckets.end());
    assert(!it->second.empty());
    size--;
    if (key) {
        assert(key->empty());
        *key = it->first;
    }
    Entry result = it->second.front();
    it->second.pop_front();
    if (it->second.empty())
        buckets.erase(it);
    return result;
}

template<class Entry>
bool TieBreakingOpenList<Entry>::empty() const {
    return size == 0;
}

template<class Entry>
void TieBreakingOpenList<Entry>::clear() {
    buckets.clear();
    size = 0;
}

template<class Entry>
void TieBreakingOpenList<Entry>::evaluate(int g, bool preferred) {
    dead_end = false;
    dead_end_reliable = false;

    for (unsigned int i = 0; i < evaluators.size(); i++) {
        evaluators[i]->evaluate(g, preferred);

        // check for dead end
        if (evaluators[i]->is_dead_end()) {
            last_evaluated_value[i] = std::numeric_limits<int>::max();
            dead_end = true;
            if (evaluators[i]->dead_end_is_reliable()) {
                dead_end_reliable = true;
            }
        } else { // add value if no dead end
            last_evaluated_value[i] = evaluators[i]->get_value();
        }
    }
    first_is_dead_end = evaluators[0]->is_dead_end();
    last_preferred = preferred;
}
template<class Entry>
void TieBreakingOpenList<Entry>::evaluate2(int g, int h) {
    dead_end = false;
    dead_end_reliable = false;
    //cout<<"calling TieBreakingOpenList::Evaluate2"<<endl;
    last_evaluated_value[0] = g+h;
    //cout<<"\tlast_evaluated_value[0]"<<last_evaluated_value[0]<<endl;
    last_evaluated_value[1]=h;
    //cout<<"\tlast_evaluated_value[1]"<<last_evaluated_value[1]<<endl;
    first_is_dead_end = false;
    last_preferred = false;
    /*  for (unsigned int i = 0; i < evaluators.size(); i++) {
        if (evaluators[i]->is_dead_end()) {
	  evaluators[i]->print_heur_name();cout<<"says this node is dead_end"<<endl;
            dead_end = true;
            if (evaluators[i]->dead_end_is_reliable()) {
                dead_end_reliable = true;
            }
	}
    }*/

    /*for (unsigned int i = 0; i < evaluators.size(); i++) {
	cout<<"doing evaluation["<<i<<"]"<<endl;
        evaluators[i]->evaluate(g, preferred);

        // check for dead end
        if (evaluators[i]->is_dead_end()) {
            last_evaluated_value[i] = std::numeric_limits<int>::max();
            dead_end = true;
            if (evaluators[i]->dead_end_is_reliable()) {
                dead_end_reliable = true;
            }
        } else { // add value if no dead end
            last_evaluated_value[i] = evaluators[i]->get_value();
        }
    }*/
    //first_is_dead_end = evaluators[0]->is_dead_end();
    //last_preferred = preferred;
    //cout<<"finished TieBreakingOpenList::Evaluate"<<endl;
}

template<class Entry>
bool TieBreakingOpenList<Entry>::is_dead_end() const {
    return dead_end;
}

template<class Entry>
bool TieBreakingOpenList<Entry>::dead_end_is_reliable() const {
    return dead_end_reliable;
}

template<class Entry>
const std::vector<int> &TieBreakingOpenList<Entry>::get_value() {
    return last_evaluated_value;
}

template<class Entry>
int TieBreakingOpenList<Entry>::dimension() const {
    return evaluators.size();
}

template<class Entry>
void TieBreakingOpenList<Entry>::get_involved_heuristics(std::set<Heuristic *> &hset) {
    for (unsigned int i = 0; i < evaluators.size(); i++) {
        evaluators[i]->get_involved_heuristics(hset);
    }
}
#endif
