#include "state.h"

#include "axioms.h"
#include "globals.h"
#include "operator.h"
#include "utilities.h"

#include <algorithm>
#include <iostream>
#include <cassert>
#include <map>
#include <stdio.h>    
using namespace std;

void State::_allocate() {
    borrowed_buffer = false;
    vars = new state_var_t[g_variable_domain.size()];
}

void State::_deallocate() {
    if (!borrowed_buffer)
        delete[] vars;
}

void State::_copy_buffer_from_state(const State &state) {
    // TODO: Profile if memcpy could speed this up significantly,
    //       e.g. if we do blind A* search.
    for (int i = 0; i < g_variable_domain.size(); i++)
        vars[i] = state.vars[i];
}

State & State::operator=(const State &other) {
    if (this != &other) {
        if (borrowed_buffer)
            _allocate();
        _copy_buffer_from_state(other);
    }
    return *this;
}

State::State(istream &in) {
    _allocate();
    check_magic(in, "begin_state");
    for (int i = 0; i < g_variable_domain.size(); i++) {
        int var;
        in >> var;
        vars[i] = var;
    }
    check_magic(in, "end_state");

    g_default_axiom_values.assign(vars, vars + g_variable_domain.size());
}

State::State(const State &state) {
    _allocate();
    _copy_buffer_from_state(state);
}

State::State(const State &predecessor, const Operator &op) {
    assert(!op.is_axiom());
    _allocate();
    _copy_buffer_from_state(predecessor);
    // Update values affected by operator.
    for (int i = 0; i < op.get_pre_post().size(); i++) {
        const PrePost &pre_post = op.get_pre_post()[i];
        if (pre_post.does_fire(predecessor))
            vars[pre_post.var] = pre_post.post;
    }

    g_axiom_evaluator->evaluate(*this);
}

//Need next function for cases on which the state needs to be used outside conditional loops on which it is generated
void State::gen_next_state(const State &predecessor, const Operator &op){
    assert(!op.is_axiom());
    //_allocate();//It is already allocated!
    _copy_buffer_from_state(predecessor);
    // Update values affected by operator.
    for (int i = 0; i < op.get_pre_post().size(); i++) {
        const PrePost &pre_post = op.get_pre_post()[i];
        if (pre_post.does_fire(predecessor))
            vars[pre_post.var] = pre_post.post;
    }
    g_axiom_evaluator->evaluate(*this);
}
State::~State() {
    _deallocate();
}

void State::dump_pddl() const {
    for (int i = 0; i < g_variable_domain.size(); i++) {
        const string &fact_name = g_fact_names[i][vars[i]];
        if (fact_name != "<none of those>")
            cout << fact_name << endl;
    }
}

void State::dump_fdr() const {
    // We cast the values to int since we'd get bad output otherwise
    // if state_var_t == char.
    for (int i = 0; i < g_variable_domain.size(); i++)
        cout << "  #" << i << " [" << g_variable_name[i] << "] -> "
             << static_cast<int>(vars[i]) << endl;
}
void State::inline_dump() const {
  cout<<"S:";
  map<int,int> variables;
  int numb=0;
    for (int i = 0; i < g_variable_domain.size(); i++){
      //istringstream ((g_variable_name[i]) ) >> numb;
      string str_temp=g_variable_name[i].substr(3);
      //cout<<"str_temp:"<<str_temp<<endl;
      numb=atoi(str_temp.c_str());
      variables[numb]=static_cast<int>(vars[i]);
      //cout<<"variables["<<numb<<"]:"<<variables[numb]<<endl;
    }
    for (int i = 0; i < g_variable_domain.size(); i++){
      cout<<variables[i]<<",";
    }
}
void State::get_state_id_string(string *state_id) const{
  state_id->clear();
  map<int,int> variables;
  int numb=0;
    for (int i = 0; i < g_variable_domain.size(); i++){
      //istringstream ((g_variable_name[i]) ) >> numb;
      string str_temp=g_variable_name[i].substr(3);
      //cout<<"str_temp:"<<str_temp<<endl;
      numb=atoi(str_temp.c_str());
      variables[numb]=static_cast<int>(vars[i]);
      //cout<<"variables["<<numb<<"]:"<<variables[numb]<<endl;
    }
    char temp_char[3];
    for (int i = 0; i < g_variable_domain.size(); i++){
      sprintf(temp_char,"%d",variables[i]);
      *state_id+=temp_char;
      *state_id+=',';
    }
   cout<<"state_id:"<<state_id->c_str()<<endl;
}

bool State::operator==(const State &other) const {
    int size = g_variable_domain.size();
    return ::equal(vars, vars + size, other.vars);
}

bool State::operator<(const State &other) const {
    int size = g_variable_domain.size();
    return ::lexicographical_compare(vars, vars + size,
                                     other.vars, other.vars + size);
}

size_t State::hash() const {
    return ::hash_number_sequence(vars, g_variable_domain.size());
}
