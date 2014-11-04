#include "search_progress.h"
#include "utilities.h"

#include <iostream>
#include <fstream>
using namespace std;

SearchProgress::SearchProgress() {
    expanded_states = 0;
    reopened_states = 0;
    evaluated_states = 0;
    evaluations = 0;
    generated_states = 0;
    dead_end_states = 0;
    generated_ops = 0;
    pathmax_corrections = 0;

    lastjump_expanded_states = 0;
    lastjump_reopened_states = 0;
    lastjump_evaluated_states = 0;
    lastjump_generated_states = 0;

    lastjump_f_value = -1;
}

SearchProgress::~SearchProgress() {
}

void SearchProgress::add_heuristic(Heuristic *h) {
    heuristics.push_back(h);
    best_heuristic_values.push_back(-1);
}

void SearchProgress::report_f_value(int f) {
    if (f > lastjump_f_value) {
        lastjump_f_value = f;
	cout<<"generated states = "<<generated_states<<endl;
	cout<<"evaluated states = "<<evaluated_states<<endl;
	cout<<"expanded states = "<<expanded_states<<endl;
	cout<<"reopened states = "<<reopened_states<<endl;
	cout<<"lastjump generated states = "<<lastjump_generated_states<<endl;
	cout<<"lastjump evaluated states = "<<lastjump_evaluated_states<<endl;
	cout<<"lastjump expanded states = "<<lastjump_expanded_states<<endl;
	cout<<"lastjump reopened states = "<<lastjump_reopened_states<<endl;
	cout<<"generated_states - lastjump_generated_states = "<<generated_states - lastjump_generated_states<<endl;

        int expanded_by_level = expanded_states - lastjump_expanded_states;

	cout<<"expanded states - lastjump expanded states = "<<expanded_by_level<<endl;
	nodes_expanded_by_level.insert(pair<int, int>(f, expanded_by_level));
        
        print_f_line();

	gen_to_exp_ratio=double(generated_states-lastjump_generated_states)/double(expanded_states-lastjump_expanded_states);
	cout<<"gen_to_exp_ratio: "<<gen_to_exp_ratio<<endl;
        lastjump_expanded_states = expanded_states;
        lastjump_reopened_states = reopened_states;
        lastjump_evaluated_states = evaluated_states;
        lastjump_generated_states = generated_states;
	cout<<"F_bound:,"<<f<<",Peak memory=,"<<get_peak_memory_in_kb()/1024.0<<endl;
    }
}
bool SearchProgress::updated_lastjump_f_value(int f) {
  if (f > lastjump_f_value) {
    return true;
  }
  else{
    return false;
  }
}
int SearchProgress::return_lastjump_f_value(){
  return lastjump_f_value;
}

void SearchProgress::get_initial_h_values() {
    for (unsigned int i = 0; i < heuristics.size(); i++) {
        initial_h_values.push_back(heuristics[i]->get_heuristic());
    }
}

bool SearchProgress::check_h_progress(int g) {
    bool progress = false;
    for (int i = 0; i < heuristics.size(); i++) {
        if (heuristics[i]->is_dead_end())
            continue;
        int h = heuristics[i]->get_heuristic();
        int &best_h = best_heuristic_values[i];
        if (best_h == -1 || h < best_h) {
            best_h = h;
            progress = true;
        }
    }
    if (progress) {
        print_h_line(g);
    }
    return progress;
}

void SearchProgress::print_f_line() const {
  static long prev_generated_states=0;
  static bool first_reported=false;
  static long nodes_from_prev_F_bound=0;
  static bool original_common_F_bound_found=false;
   
    cout<<"fnivel: "<<lastjump_f_value<<endl;
    cout<<"nodesGeneratedByLevel: "<<generated_states-prev_generated_states<<endl;
    nodes_generated_by_level.insert(pair<int, int>(lastjump_f_value, prev_generated_states)); 

    cout<<" time0: "<<g_timer()<<endl;
    cout<<"nodesGeneratedToTheLevel: "<<generated_states<<endl;
    //print_time_line();
    cout << "f: " << lastjump_f_value
         << " [";
    print_line();
    cout <<",generated_states:,"<<generated_states<<",additional_states:,"<<generated_states-prev_generated_states<<",]" << ",random_comb_index:"<<random_comb_index<< endl;
    if(common_sampling_F_boundary!=0){//so it has been populated, the sampling f-boundary was modified upwards
      if(lastjump_f_value==F_bound_to_print){
	original_common_F_bound_found=true;
      }
      if(lastjump_f_value>=F_bound_to_print&&lastjump_f_value<common_sampling_F_boundary){
	nodes_from_prev_F_bound+=generated_states-prev_generated_states;
	cout<<"nodes_from_prev_F_bound="<<nodes_from_prev_F_bound<<endl;
      }
      else if(!first_reported&&lastjump_f_value>=common_sampling_F_boundary){//we are past te common_f_bounadry, this is the reported f-boundary
	if(original_common_F_bound_found){
	  ofstream outputFile;
	  outputFile.open("sampling_accuracy.txt",ios::app);
	  outputFile<<"\t"<<generated_states-prev_generated_states+nodes_from_prev_F_bound<<"\t"<<generated_states;
	  outputFile.close();
	  first_reported=true;
	}
	else{
	  original_common_F_bound_found=true;//so it gets printed in the next F-boundary
	}
      }
    }
    else if(lastjump_f_value==F_bound_to_print){
      original_common_F_bound_found=true;
    }
    else if(Current_RIDA_Phase==SOLVING_PHASE&&F_bound_to_print<lastjump_f_value&&(!first_reported)){//we want to record for statistics the accuracy of RIDA* predition
      if(original_common_F_bound_found){
	ofstream outputFile;
	outputFile.open("sampling_accuracy.txt",ios::app);
	outputFile<<"\t"<<generated_states-prev_generated_states+nodes_from_prev_F_bound<<"\t"<<generated_states;
	outputFile.close();
	first_reported=true;
      }
      else{
	original_common_F_bound_found=true;//not really but we know for whichever value is the next F=bound it is the one we made a prediction for, this current F-value is the not-common F-boundary which corrensponds to where the initial predictions was made, the net boundary tells us the actual nodes we were predicting for.
      }
    }
    else{
      cout<<"last_jump:"<<lastjump_f_value<<",F_bound_to_print:"<<F_bound_to_print<<endl;
    }
      
    prev_generated_states=generated_states;
    last_full_f_boundary=lastjump_f_value;
    last_gen_nodes=generated_states;
}

void SearchProgress::print_h_line(int g) const {
    cout << "Best heuristic value: ";
    for (int i = 0; i < heuristics.size(); i++) {
        cout << best_heuristic_values[i];
        if (i != heuristics.size() - 1)
            cout << "/";
    }
    cout << " [g=" << g << ", ";
    print_line();
    cout << "]" << endl;
}

void SearchProgress::print_line() const {
    static double prev_F_boundary_time=0;
    cout << evaluated_states << " evaluated, "
         << expanded_states << " expanded, ";
    if (reopened_states > 0) {
        cout << reopened_states << " reopened, ";
    }
    F_boundary_time=g_timer()-prev_F_boundary_time;//F-boundary reduction
    cout << "t=" << g_timer;
    prev_F_boundary_time=g_timer();
    last_f_boundary_time=g_timer();
}


void SearchProgress::print_time_line() const {
    static double prev_F_boundary_time2 = 0;
    F_boundary_time = g_timer() - prev_F_boundary_time2;
    cout<<"time0: "<<F_boundary_time<<endl;
    prev_F_boundary_time2=g_timer();
}


void SearchProgress::print_statistics() const {
    if (!initial_h_values.empty()) {
        // This will be skipped in the cumulative statistics of an
        // iterated search, which do not have initial h values.
        cout << "Initial state h value: ";
        for (int i = 0; i < initial_h_values.size(); i++) {
            cout << initial_h_values[i];
            if (i != initial_h_values.size() - 1)
                cout << "/";
        }
        cout << "." << endl;
    }

    cout << "Expanded " << expanded_states << " state(s)." << endl;
    cout << "Reopened " << reopened_states << " state(s)." << endl;
    cout << "Evaluated " << evaluated_states << " state(s)." << endl;
    cout << "Evaluations: " << evaluations << endl;
    cout << "Generated " << generated_states << " state(s)." << endl;
    cout << "Dead ends: " << dead_end_states << " state(s)." << endl;
    if (pathmax_corrections > 0) {
        cout << "Pathmax corrections: " << pathmax_corrections << endl;
    }

    if (lastjump_f_value >= 0) {
        cout << "Expanded until last jump: "
             << lastjump_expanded_states << " state(s)." << endl;
        cout << "Reopened until last jump: "
             << lastjump_reopened_states << " state(s)." << endl;
        cout << "Evaluated until last jump: "
             << lastjump_evaluated_states << " state(s)." << endl;
        cout << "Generated until last jump: "
             << lastjump_generated_states << " state(s)." << endl;
    }
}
void SearchProgress::reset(){
  expanded_states=0;
  reopened_states=0;
  evaluated_states=0;
  evaluations=0;
  generated_states=0;
  dead_end_states=0;
}
