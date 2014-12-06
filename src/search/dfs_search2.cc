#include "dfs_search.h"

#include "globals.h"
#include "heuristic.h"
#include "utilities.h"
#include "option_parser.h"
#include "successor_generator.h"
#include "g_evaluator.h"
#include "sum_evaluator.h"
#include "max_evaluator.h"
#include "plugin.h"

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <set>
#include <iostream>
#include <fstream>

#include <map>
#include <dirent.h>

using namespace std;


HST lsearch_space5(10000, 1);
static bool time_limit_node_adjusted=false;
DFSSearch::DFSSearch(
	const Options &opts) 
	: SearchEngine(opts),
	reopen_closed_nodes(opts.get<bool>("reopen_closed")),
	do_pathmax(opts.get<bool>("pathmax")),
	use_multi_path_dependence(opts.get<bool>("mpd")),
	mark_children_as_finished(opts.get<bool>("mark_children_as_finished")),
	open_list(opts.get<OpenList<state_var_t *> *>("open")), 
	f_evaluator(opts.get<ScalarEvaluator *>("f_eval")) {
	if (opts.contains("preferred")) {
		preferred_operator_heuristics = 
			opts.get_list<Heuristic *>("preferred");
		for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
			cout<<i<<endl;
		}
        }
}

void DFSSearch::output_problem_results() {
	ofstream outputFile;
	outputFile.open("results.txt", ios::app);
	outputFile<<g_plan_filename<<"\t"<<"0"<<"\t"<<search_progress.get_generated()<<"\t"<<search_timer();
	outputFile<<"\t"<<g_timer()<<endl;
	outputFile.close();
}



void DFSSearch::initialize() {
	cout<<" __________________________________________"<<endl;
	cout<<"|  initialize() - dfs_search.cc             |"<<endl;
	cout<<" __________________________________________"<<endl;
	//use basic_ios::imbue
	std::cout.imbue(std::locale(std::cout.getloc(), new punct_facet<char, ','>));

	cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
         cout<<"first_sample set to true"<<endl;
	first_sample=true;
        counter = 0;
        cout<<"do_pathmax "<<do_pathmax<<endl;
        cout<<"use_multi_path_dependence = "<<use_multi_path_dependence<<endl;
        cout<<"mark_children_as_finished = "<<mark_children_as_finished<<endl; 
	if (do_pathmax)
        	cout << "Using pathmax correction" << endl;
    	if (use_multi_path_dependence)
        	cout << "Using multi-path dependence (LM-A*)" << endl;
    	assert(open_list != NULL);
	int i;
	cout<<"something here"<<endl;
	cout<<"argc_copy = "<<argc_copy<<endl;
    	for (i = 0; i < argc_copy; ++i) {
      		puts(argv_copy[i]);
      		cout<<"i:"<<i<<","<<argv_copy[i]<<endl;
    	}
	cout<<"Initial state:"<<endl;
	//g_initial_state->inline_dump();
	//cout<<"Goal state:"<<g_goal<<endl;

	set<Heuristic *> hset;
    	cout<<"calling get_involved heuristics"<<endl;
	fflush(stdout);
    	open_list->get_involved_heuristics(hset);
    	cout<<"got involved heuristics"<<endl;
	fflush(stdout);
    	total_sampling_timer=0;	
	for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
        	estimate_heuristics.push_back(*it);
		search_progress.add_heuristic(*it);
    	}
	
	// add heuristics that are used for preferred operators (in case they are
    	// not also used in the open list)
    	hset.insert(preferred_operator_heuristics.begin(),
                preferred_operator_heuristics.end());

    	// add heuristics that are used in the f_evaluator. They are usually also
    	// used in the open list and hence already be included, but we want to be
    	// sure.
    	if (f_evaluator) {
        	f_evaluator->get_involved_heuristics(hset);
    	}
    	cout<<"f_evaluator populated"<<endl;fflush(stdout);



	for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
        	heuristics.push_back(*it);
    	}
    	orig_heuristics=heuristics;
    	cout<<"# heuristics: "<<heuristics.size()<<endl;
	fflush(stdout);

	bool conditional_effects_present=false;
    	for (int i = 0; i < g_operators.size(); ++i) {
		//cout<<i<<endl;
        	const vector<PrePost> &pre_post = g_operators[i].get_pre_post();
       		for (int j = 0; j < pre_post.size(); ++j) {
		    const vector<Prevail> &cond = pre_post[j].cond;
		    if (cond.empty()) {
	            	continue;
                    }
		    int var = pre_post[j].var;
		    int pre = pre_post[j].pre;
		    int post = pre_post[j].post;
	            if (pre == -1 && cond.size() == 1 && 
                        cond[0].var == var && cond[0].prev != post &&
                        g_variable_domain[var] == 2){
			cout<<"entra aqui"<<endl;
			cout<<"var = "<<var<<endl;
		    	cout<<"pre = "<<pre<<endl;
                    	cout<<"post = "<<post<<endl;
		    }
		    cout<<"Conditional effects present "<<endl;
                    fflush(stdout);
		    conditional_effects_present=true;
                    break;
		}
		if (conditional_effects_present) {
	            break;
		}
    	}
	cout<<"conditional_effects_present = "<<conditional_effects_present<<endl;
	//change to conditional_effects_present
        cout<<"Conditional effects present = "<<conditional_effects_present<<endl;
	if (conditional_effects_present) {
            orig_heuristics=heuristics;
	    heuristics.clear();
            for (size_t i = 0; i < orig_heuristics.size(); i++) {
		string heur_name=heuristics[i]->get_heur_name();
		cout<<"heuristic name = "<<heur_name<<endl;
		if (heur_name.find("hmax") != string::npos) {
	           heuristics.push_back(orig_heuristics[i]);
		   cout<<"hmax can handle condition effects, so keeping"<<endl;
		} else if (heur_name.find("blind") != string::npos) {
		   heuristics.push_back(orig_heuristics[i]);
                   cout<<"blind can handle condition effects, so keeping"<<endl;
		   continue;
		} else {
		   cout<<"skipping because condition effects heuristic "<<orig_heuristics[i]->get_heur_name()<<endl;
		}
	    }
	    cout<<"# heuristics after eliminating those not supporting conditionn effects: "<<heuristics.size()<<endl;
	}
	cout<<"starting timings "<<endl;
        fflush(stdout);

	//timing node generation time
	Timer heur_timings;
        double node_counter=0;
	cout<<"initial state = "<<g_initial_state<<endl;
        State s(*g_initial_state);
	vector<const Operator *> applicable_ops;
	const Operator *op;
	while (heur_timings() < 1.0) {
          node_counter++;
          g_successor_generator->generate_applicable_ops(s, applicable_ops);
          //cout<<"applicable_ops = "<<applicable_ops<<endl;
	  op = applicable_ops[rand()%applicable_ops.size()];
	  State succ_state(s, *op);
	  s = succ_state;
          //cout<<"State s = "<<s<<endl;
          //cout<<"g_successor_generator = "<<g_successor_generator<<endl;
	}
	cout<<"node_counter = "<<node_counter<<endl;
	node_gen_and_exp_cost=heur_timings.stop()/node_counter;
	cout<<"node_gen_and_exp_cost = "<<node_gen_and_exp_cost<<endl;
	node_time_adjusted_reval=0.5/node_gen_and_exp_cost;
	assert(!heuristics.empty());
	cout<<"active heuristics size : "<<heuristics.size()<<endl;

	int max_h=0; 
        bool dead_end=false;
	cout<<"# heristics = "<<heuristics.size()<<endl;
	for (size_t i = 0; i < heuristics.size(); i++) {
	     dead_end=heuristics[i]->is_dead_end();
	     cout<<"dead_end = "<<dead_end<<endl;
	     if (dead_end) {
	     	break;
             }
              cout<<"heuristics["<<i<<"]->get_heuristic() = "<<heuristics[i]->get_heuristic()<<endl;
	     max_h = max(max_h, heuristics[i]->get_heuristic());
             cout<<"max_h = "<<max_h<<endl; 
        }
        open_list->evaluate2(0, max_h);
	search_progress.inc_evaluated_states();
	search_progress.inc_evaluations(heuristics.size());
	
	if (dead_end) {
            cout<<"Initial state is a dead end."<<endl;
	    search_progress.inc_dead_ends();
	} else {
	    search_progress.get_initial_h_values();
	    cout<<"is not dead end"<<endl;
	    if (f_evaluator) {
	        f_evaluator->evaluate(0, false);
		cout<<"f_evaluator = "<<f_evaluator->get_value()<<endl;
		search_progress.report_f_value(f_evaluator->get_value());
	}
	search_progress.check_h_progress(0);
	SearchNode node = search_space.get_node(*g_initial_state);
	heuristic_name = heuristics[0]->get_heur_name();
        cout<<"heuristics[0]->get_value() = "<<heuristics[0]->get_value()<<endl;
	node.open_initial(heuristics[0]->get_value());
        cout<<"node.get_state_buffer() = "<<node.get_state_buffer()<<endl;
        open_list->insert(node.get_state_buffer());	
	}
	cout<<"starting timing individual heuristics."<<endl;
	fflush(stdout);
	//time each heuristic
	double max_TPN=0;
	double aggr_TPN=0;

	int lp_pdbs_max_h=0;
	int max_lp_pattern_size=0;
	vector<int> initial_h_values;
	for (size_t i = 0; i < heuristics.size(); i++) {
	     cout<<"SAMPLING_PHASE = "<<SAMPLING_PHASE<<endl;
	     cout<<"Current_RIDA_Phase = "<<Current_RIDA_Phase<<endl;
	     if (Current_RIDA_Phase == SAMPLING_PHASE) {
		 heur_timings.reset();
		 heur_timings.resume();
		 double counter = 0;
                 while (heur_timings()<0.1) { //in situ measure heuristic evaluation cost
		    counter++;
                    heuristics[i]->evaluate(*g_initial_state);	    
		 }
		 double TPN=heur_timings.stop()/counter;
                 max_TPN=max(TPN, max_TPN);
		 aggr_TPN+=TPN;

		 string heur_name=heuristics[i]->get_heur_name();
		 cout<<"heur_name = "<<heur_name<<endl;
		 if (heur_name.substr(0, 13) == "heur:,lp_pdb") {
	             cout<<"heuristic "<<i<<" is a lp_pdb"<<endl;
		     if (lp_pdbs_max_h < heuristics[i]->get_heuristic()) {
			max_lp_pattern_size=heuristics[i]->get_systematic_index();
                        cout<<"new max_lp_pattern_size: "<<max_lp_pattern_size<<endl;
                     }
		     lp_pdbs_max_h=max(lp_pdbs_max_h, heuristics[i]->get_heuristic());
  		 }
                 heuristics[i]->set_measured_TPN(TPN);
		 cout<<"h[,"<<i<<",] is:,"; 
                 heuristics[i]->print_heur_name();
                 cout<<",measured time cost: "<<heuristics[i]->get_measured_TPN()<<",h:"<<heuristics[i]->get_heuristic()<<endl;
                 initial_h_values.push_back(heuristics[i]->get_heuristic()); 
	     } else {
		max_h=max(max_h, heuristics[i]->get_heuristic());
		heuristics[i]->evaluate(*g_initial_state);
      		cout<<"h[,"<<i<<",] is:,";
                heuristics[i]->print_heur_name();
                fflush(stdout);
                cout<<",h:"<<heuristics[i]->get_heuristic()<<endl;
                fflush(stdout);
             }
	     max_h = max(max_h, heuristics[i]->get_heuristic());
	}

	if (Current_RIDA_Phase==SAMPLING_PHASE) {
            for (size_t i = 0; i < heuristics.size(); i++) {
	        string heur_name = heuristics[i]->get_heur_name();
		if (heur_name.substr(0, 13) == "heur:,lp_pdb,") {
                    if (heuristics[i]->get_systematic_index() > max_lp_pattern_size) {
		        heuristics[i]->set_stop_using(true);
		    }
                } 
            }
            //remove any heuristics which have been set to stop_using
 	    heuristics.clear();
	    cout<<"memory before deleting databases: "<<endl;
            cout<<"VmRSS memory: "<<get_memory_VmRSS()<< " KB"<<endl;

            for (size_t i = 0; i < orig_heuristics.size(); i++) {
	         if (!orig_heuristics[i]->is_using()) {
                     cout<<"removing initial heur ";
                     orig_heuristics[i]->print_heur_name();
                     cout<<endl;
		     orig_heuristics[i]->free_up_memory(search_space);
                     orig_heuristics[i]->free_up_memory();
                     continue;
                 } else {
                   orig_heuristics[i]->evaluate(*g_initial_state);
                   heuristics.push_back(orig_heuristics.at(i));
                 }
            }
            cout<<"memory after deleting all databases: "<<endl;
	    cout<<"VmRSS memory: "<<get_memory_VmRSS()<< " KB"<<endl;
	}
        orig_heuristics.clear();

        orig_heuristics=heuristics; //Update the original heuristics by removing those eliminated in the initialization phase
	cout<<"Remaining heuristics:"<<endl;
        for (size_t i = 0; i < orig_heuristics.size(); i++) {
 	     cout<<"remaining initial heur ";
             orig_heuristics[i]->print_heur_name();
             cout<<endl;
        }

	node_time_adjusted_reval= 2.0/(node_gen_and_exp_cost+max_TPN);
	node_time_adjusted_reval= min(max(node_time_adjusted_reval, 10), 1000);
        cout<<"node_time_adjusted_reval based on the min of half second node_gen and the most expensive heuristic or 1000 nodes: "<<node_time_adjusted_reval<<endl;
        fflush(stdout);

	cout<<"Implementar SS Heuristic."<<endl;

}


void DFSSearch::statistics() const {
	search_progress.print_statistics();
	search_space.statistics();
}



void DFSSearch::DFS(SearchNode nodecp) {
      cout<<"iter: "<<counter<<endl;
      counter++;
      cout<<"\n\tNode Raiz = "<<nodecp.get_h()<<", g = "<<nodecp.get_real_g()<<", f = "<<nodecp.get_h() + nodecp.get_real_g()<<"\n";
       nodecp.set_visited(); 
       State newState = nodecp.get_state();

       vector<const Operator *> applicable_ops;
       set<const Operator *> preferred_ops;
 
       g_successor_generator->generate_applicable_ops(newState, applicable_ops);
       for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
           Heuristic *h = preferred_operator_heuristics[i];
           h->evaluate(newState);
           if (!h->is_dead_end()) {
              vector<const Operator *> preferred;
              h->get_preferred_operators(preferred);
              preferred_ops.insert(preferred.begin(), preferred.end());
           }
       }

       search_progress.inc_evaluations(preferred_operator_heuristics.size());
       cout<<"\tHow many? "<<applicable_ops.size()<<endl;
       for (int i = 0; i < applicable_ops.size(); i++) {
          
           
           const Operator *op = applicable_ops[i];
           State succ_state(newState, *op);
           search_progress.inc_generated();
           SearchNode succ_node = search_space.get_node(succ_state);
           if (true) {
              int succ_h = 0;
              bool dead_end = false;
              for (size_t i = 0; i < heuristics.size(); i++) {
                  heuristics[i]->evaluate(succ_state);
                  dead_end = heuristics[i]->is_dead_end();
                  if (dead_end) {
                     if (Current_RIDA_Phase==SOLVING_PHASE) {
                        break;
                     } else {
                        succ_h = INT_MAX/2;
                     }
                  }
                  succ_h = max(succ_h, heuristics[i]->get_heuristic());
              }
              succ_node.open(succ_h, nodecp, op);
             // heuristics[0]->evaluate(succ_state);
             // int succ_h = heuristics[0]->get_heuristic();
             
             if (!succ_node.is_visited()) {
                 if (succ_node.get_real_g() <=  12) {
                    succ_node.open(succ_h, nodecp, op);
                    cout<<"\t\tNode Child = "<<succ_node.get_h()<<", g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<"\n";
                    S.push(succ_node);
                    DFS(succ_node);
                 }
             } 
           }
         }  
}


int DFSSearch::step() {
    cout<<" ____________________________"<<endl;
    cout<<"|       step process         |"<<endl;
    cout<<" ____________________________"<<endl;
    cout<<"search_space size () = "<<search_space.size()<<endl; 
    SearchNode node = search_space.get_node(*g_initial_state);

    node.open_initial(heuristics[0]->get_value());
    int depth = 2*node.get_h();
    
    node.setL(0);   
    S.push(node);
    P.push(node);
  
    v_f.push_back(node.get_h() + node.get_real_g());
    v_g.push_back(node.get_real_g());
    v_h.push_back(node.get_h());
    cout<<"Raiz Node h = "<<node.get_h()<<", g = "<<node.get_real_g()<<", f = "<<node.get_h() + node.get_real_g()<<endl;
    //DFS(node\);
    
    //int iter = 0;
    while (!S.empty()) {
        cout<<"iter = "<<counter<<endl;
        counter++;
        printStack2(S); 
     
        SearchNode nodecp = S.top();
        //nodecp.set_as_new_node();
        int g = nodecp.getL();
        cout<<"Raiz h = "<<nodecp.get_h()<<", g = "<<nodecp.get_real_g()<<", f = "<<nodecp.get_h() + nodecp.get_real_g()<<endl;
        S.pop();
        //printStack(S);  

        //Every 2 secs aprox we check if we have done search for too long without selecting a subset
        //Note that timer checks can actually be quite expensive when node generation cost microseconds or less, that is why we only do this check 
        //every time we have generated enough nodes to cover approx 2 secs.  Modulus operation is very cheap.
        if(Current_RIDA_Phase==SAMPLING_PHASE){
          if(search_progress.get_generated()%node_time_adjusted_reval==0) {
	    if(search_timer()>200.0){
	      cout<<"sample_frontier_now, actual time above the 200 secs limit maximizing all heuristics"<<",overall time:"<<g_timer()<<",search time:"<<search_timer()<<endl;
	      if(gen_to_eval_ratio==0){
	        cout<<"setting gen_to_eval as the first F-boundary was not completed, doing early sampling"<<endl;
	        gen_to_eval_ratio=double(search_progress.get_generated())/double(search_progress.get_evaluated_states());
	        cout<<"gen_to_eval_ratio:"<<gen_to_eval_ratio<<endl;
	      }
              cout<<"IF IN SOME MOMENT ENTER HERE."<<endl;
	      sample_frontier_now(nodecp.get_g()+nodecp.get_h());
            } 
          }
        }

       State newState = nodecp.get_state();

       vector<const Operator *> applicable_ops;
       set<const Operator *> preferred_ops;
 
       g_successor_generator->generate_applicable_ops(newState, applicable_ops);
       for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
           Heuristic *h = preferred_operator_heuristics[i];
           h->evaluate(newState);
           if (!h->is_dead_end()) {
              vector<const Operator *> preferred;
              h->get_preferred_operators(preferred);
              preferred_ops.insert(preferred.begin(), preferred.end());
           }
       }

       search_progress.inc_evaluations(preferred_operator_heuristics.size());
       cout<<"nodecp.is_visited() = "<<nodecp.is_visited()<<endl;
       if (!nodecp.is_visited()) {
          nodecp.set_visited();
          for (int i = 0; i < applicable_ops.size(); i++) {
           if (incremental_memory_limit) {
              for (int j = 0; j < heuristics.size(); j++) {
                  Heuristic *h = heuristics[j];
                  h->free_up_memory(search_space);
              }
           }    
        
           const Operator *op = applicable_ops[i];
           if ((nodecp.get_real_g() + op->get_cost()) >= bound) {
              continue;
           }
  
           State succ_state(newState, *op);
           search_progress.inc_generated();
           SearchNode succ_node = search_space.get_node(succ_state);
           if (succ_node.is_dead_end()) {
              continue;
           }
           //update new path
           if (use_multi_path_dependence || succ_node.is_new()) {
              bool h_is_dirty = false;
              for (size_t i = 0; i < heuristics.size(); i++) {
                   if (heuristics[i]->reach_state(newState, *op, succ_node.get_state())) {
                      h_is_dirty = true;
                   }
              }
              if (h_is_dirty && use_multi_path_dependence) {
                 succ_node.set_h_dirty();
              }
           }
           if (true) {
              int succ_h = 0;
              bool dead_end = false;
              for (size_t i = 0; i < heuristics.size(); i++) {
                  heuristics[i]->evaluate(succ_state);
                  dead_end = heuristics[i]->is_dead_end();
                  if (dead_end) {
                     if (Current_RIDA_Phase==SOLVING_PHASE) {
                        break;
                     } else {
                        succ_h = INT_MAX/2;
                     }
                  }
                  succ_h = max(succ_h, heuristics[i]->get_heuristic());
              }
             // heuristics[0]->evaluate(succ_state);
             // int succ_h = heuristics[0]->get_heuristic();
              
              succ_node.open(succ_h, nodecp, op);
             
              if (mark_children_as_finished) {
                 for (size_t i = 0; i < heuristics.size(); i++) {
                     heuristics[i]->finished_state(succ_node.get_state(), succ_node.get_real_g()+ succ_node.get_h(), true);
                 }
              }

              cout<<"\tNodes generated:  h = "<<succ_node.get_h()<<", g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<endl;


              int succ_h2 = succ_node.get_h();
              int succ_g = succ_node.get_real_g();
              succ_node.setL(g+1);
              if (succ_g <= depth) {
                   cout<<"\n\t\tChild added :  h = "<<succ_node.get_h()<<", g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<"\n"<<endl;
                   P.push(succ_node);
                   S.push(succ_node);
                   P2.push(succ_node);
                   Pint.push(succ_node.get_h() + succ_node.get_real_g());
                  
                   v_f.push_back(succ_h2 + succ_g);
                   v_g.push_back(succ_g);
                   v_h.push_back(succ_h2); 
              } // end if prunning g <= depth
           } //is new
       } // enf for applicable
       

      }//end if is visited



       /*for (int i = 0; i < applicable_ops.size(); i++) {
           if (incremental_memory_limit) {
              for (int j = 0; j < heuristics.size(); j++) {
                  Heuristic *h = heuristics[j];
                  h->free_up_memory(search_space);
              }
           }    
        
           const Operator *op = applicable_ops[i];
           if ((nodecp.get_real_g() + op->get_cost()) >= bound) {
              continue;
           }
  
           State succ_state(newState, *op);
           search_progress.inc_generated();
           SearchNode succ_node = search_space.get_node(succ_state);
           if (succ_node.is_dead_end()) {
              continue;
           }
           //update new path
           if (use_multi_path_dependence || succ_node.is_new()) {
              bool h_is_dirty = false;
              for (size_t i = 0; i < heuristics.size(); i++) {
                   if (heuristics[i]->reach_state(newState, *op, succ_node.get_state())) {
                      h_is_dirty = true;
                   }
              }
              if (h_is_dirty && use_multi_path_dependence) {
                 succ_node.set_h_dirty();
              }
           }
           if (succ_node.is_new()) {
              int succ_h = 0;
              bool dead_end = false;
              for (size_t i = 0; i < heuristics.size(); i++) {
                  heuristics[i]->evaluate(succ_state);
                  dead_end = heuristics[i]->is_dead_end();
                  if (dead_end) {
                     if (Current_RIDA_Phase==SOLVING_PHASE) {
                        break;
                     } else {
                        succ_h = INT_MAX/2;
                     }
                  }
                  succ_h = max(succ_h, heuristics[i]->get_heuristic());
              }
             // heuristics[0]->evaluate(succ_state);
             // int succ_h = heuristics[0]->get_heuristic();
              
              succ_node.open(succ_h, nodecp, op);
             
              if (mark_children_as_finished) {
                 for (size_t i = 0; i < heuristics.size(); i++) {
                     heuristics[i]->finished_state(succ_node.get_state(), succ_node.get_real_g()+ succ_node.get_h(), true);
                 }
              }

              cout<<"\tNodes generated:  h = "<<succ_node.get_h()<<", g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<endl;


              int succ_h2 = succ_node.get_h();
              int succ_g = succ_node.get_real_g();
              succ_node.setL(g+1);
              if (succ_g <= depth) {
                   cout<<"\n\t\tChild added :  h = "<<succ_node.get_h()<<", g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<"\n"<<endl;
                   P.push(succ_node);
                   S.push(succ_node);

                   v_f.push_back(succ_h2 + succ_g);
                   v_g.push_back(succ_g);
                   v_h.push_back(succ_h2); 
              } // end if prunning g <= depth
           }
       }*/ //end for applicable
    } // end while
    cout<<"Print P."<<endl;
    printStack(P);
    cout<<"Print P2."<<endl;
    printStack2(P2);
    cout<<"Print Pint."<<endl;
    printStack3(Pint);
 
    cout<<"\nVector."<<endl;
    cout<<"v_f.size() = "<<v_f.size()<<endl;
    for (int i = 0; i < v_f.size(); i++) {
        cout<<"\t\t h = "<<v_h.at(i)<<", g = "<<v_g.at(i)<<", f = "<<v_h.at(i) + v_g.at(i)<<endl;
    }
    generateReport(v_h, v_g);    
    
   return SOLVED;
}

void DFSSearch::printStack(stack<SearchNode> S) {
    cout<<"**********************************************************************"<<endl;
    stack<SearchNode> A;
    while (!S.empty()) {
       SearchNode n = S.top();
       cout<<"\t\t h = "<<n.get_h()<<", g = "<<n.get_real_g()<<", f = "<<n.get_h() + n.get_real_g()<<endl;
       S.pop();
       A.push(n);
    }
    while (!A.empty()) {
       SearchNode n = A.top();
       A.pop();
       S.push(n);
    }
    cout<<"**********************************************************************"<<endl;
}


void DFSSearch::printStack2(Stack<SearchNode> S) {
    cout<<"**********************************************************************"<<endl;
    Stack<SearchNode> A;
    while (!S.empty()) {
       SearchNode n = S.pop();
       cout<<"\t\t h = "<<n.get_h()<<", g = "<<n.get_real_g()<<", f = "<<n.get_h() + n.get_real_g()<<endl;
       A.push(n);
    }
    while (!A.empty()) {
       S.push(A.pop());
    }
    cout<<"**********************************************************************"<<endl;
}


void DFSSearch::printStack3(Stack<int> S) {
    cout<<"**********************************************************************"<<endl;
    Stack<int> A;
    while (!S.empty()) {
       int n = S.pop();
       cout<<"\t\t f = "<<n<<endl;
       A.push(n);
    }
    while (!A.empty()) {
       S.push(A.pop());
    }
    cout<<"**********************************************************************"<<endl;
}


map<int, int> DFSSearch::getFDistribution(vector<int> v_f_value) { 
	map<int, int> m;
	for (int i = 0; i < v_f_value.size(); i++) {
	    int a = v_f_value.at(i);
	    int k = 1;
	    for (int j = 0; j < v_f_value.size(); j++) {
		int b = v_f_value.at(j);
		if (i != j) {
		   if (a==b) {
		      k++;	
 		   }
		}
	    }
	    map<int, int>::iterator mIter = m.find(a);
	    if (mIter != m.end()) {
                //TODO        
	    } else {
		 m.insert(pair<int, int>(a, k));
	    }
	}
	return m;
}


void DFSSearch::generateReport(vector<int> v_h, vector<int> v_g) {

    map<int, int> g;
    map<int, vector<int> > mapv_f;
    for (int i = 0; i < v_g.size(); i++) {
        int a = v_g.at(i);
        int k = 1;
        for (int j = 0; j <  v_g.size(); j++) {
            int b = v_g.at(j);
            if (i != j) {
               if (a==b) {
                  k++;
               }
            }
        }
        map<int, int>::iterator mIter = g.find(a);
        if (mIter == g.end()) {
           g.insert(pair<int, int>(a, k));
        }
    }
    cout<<"g.size() = "<<g.size()<<endl;
    int r = 0;
    cout<<"Display."<<endl;
    vector<int> f_exp;
    for (map<int, int>::iterator iter = g.begin(); iter != g.end(); iter++) {
        int level = iter->first;
        int q = iter->second;

        cout<<"g = "<<level<<endl;
        vector<int> v;
        for (int i = 0; i < q; i++) {
            int f = v_h.at(r) + v_g.at(r);
            v.push_back(f);
            f_exp.push_back(f);
            r++;
        }
        for (int i = 0; i < v.size(); i++) {
            cout<<v.at(i)<<" ";
        }
        cout<<"\n\n";
        mapv_f.insert(pair<int, vector<int> >(level, v));
    }
    
    cout<<"f_exp.size() = "<<f_exp.size()<<endl;
    map<int, int> dist = getFDistribution(f_exp);
    cout<<"f(camada)\t#nodes expanded"<<endl;
    for (map<int, int>::iterator iter =  dist.begin(); iter != dist.end(); iter++) {
        int f = iter->first;
        int q = iter->second;
        cout<<f<<"\t"<<q<<"\n";
    }
    cout<<"\n";


    ofstream output;
    vector<string> vs = readFile();
    string dominio = vs.at(0);
    string tarefa = vs.at(1);
    string heuristica = vs.at(2);
    cout<<"dominio = "<<dominio<<endl;
    cout<<"tarefa = "<<tarefa<<endl;
    cout<<"heuristica = "<<heuristica<<endl;

    string outputFile = "/home/marvin/marvin/testdfs/"+heuristica+"/reportdfs/"+dominio+"/fdist/"+tarefa;

    output.open(outputFile.c_str());
    output<<"\t\ttitle\n";
    output<<"totalniveles: 1\n";
    output<<"threshold: 12\n";

   cout<<"f-dist"<<endl;
   for (map<int, vector<int> >::iterator iter = mapv_f.begin(); iter != mapv_f.end(); iter++) {
       cout<<"g = "<<iter->first<<endl;
       output<<"g:"<<iter->first<<"\n";
       vector<int> v = iter->second;

       map<int, int> m = getFDistribution(v);
       output<<"size: "<<m.size()<<"\n";
       for (map<int, int>::iterator ite = m.begin(); ite != m.end(); ite++) {
           cout<<"f: "<<ite->first<<" q: "<<ite->second<<"\n";
           output<<"\tf: "<<ite->first<<" q: "<<ite->second<<"\n";
       }
       cout<<"\n";
   }  

   output.close();
}

vector<string> DFSSearch::readFile() {
        vector<string> vs;
        string path;
     
        char input[] = "/home/marvin/fd/src/translate/arquivos/";  
        DIR *dir;
        struct dirent *ent;

        dir = opendir(input);
        if (dir != NULL) {
	   while ((ent = readdir(dir)) != NULL) {
	      string fileName = ent->d_name;
              cout<<"fileName size () = "<<fileName.size()<<endl;
              if ((fileName.size() == 1) || (fileName.size() == 2)) {
	         //TODO 
	      } else {
	         path = fileName;			
	      }
           }
           closedir(dir);
        } else {
	   cout<<"directory does not exists."<<endl;
        }
    
        cout<<"Path in the ss = "<<path<<endl;
    
        string rutaT = "/home/marvin/fd/src/translate/arquivos/"+path; 
        ifstream fileT(rutaT.c_str());

        string dominio;
        string tarefa;
        string heuristica;

        fileT>>dominio;
        fileT>>tarefa;
        fileT>>heuristica;
        
        vs.push_back(dominio.c_str());
        vs.push_back(tarefa.c_str());
        vs.push_back(heuristica.c_str());
        return vs;
}

int DFSSearch::getMax_gvalue(vector<int> v_g) {
	int winner = v_g.at(0);
	for (int i = 0; i < v_g.size(); i++) {
	     if (winner < v_g.at(i)) {
		winner = v_g.at(i);
	     }
	}
	return winner;
}

pair<SearchNode, bool> DFSSearch::fetch_next_node() {
	cout<<" ___________________________"<<endl;
	cout<<"|     fetch_next_node       |"<<endl;
	cout<<" ___________________________"<<endl;
	while (true) {
	    cout<<"open_list->empty() = "<<open_list->empty()<<endl;
	    if (open_list->empty()) {
		cout<<"Completely explored state space -- no solution!"<<endl;
		return make_pair(search_space.get_node(*g_initial_state), false);
	    }
	    vector<int> last_key_removed;
	    State state(open_list->remove_min(
			    use_multi_path_dependence ? &last_key_removed : 0));	    SearchNode node = search_space.get_node(state);

	    if (node.is_closed()) {
		continue;
	    }
	    cout<<"use_multi_path_dependence = "<<use_multi_path_dependence<<endl;
	    if (use_multi_path_dependence) {
		cout<<"use_multi_path_dependence is on, sort out how to use multiple heurisit "<<endl;
		exit(0);
		assert(last_key_removed.size() == 2);
		int pushed_h = last_key_removed[1];
		assert(node.get_h() >= pushed_h);
		if (node.get_h() > pushed_h) {
			continue;
		}
		assert(node.get_h() == pushed_h);
		if (!node.is_closed()  && node.is_h_dirty()) {
		   for (size_t i = 0; i < heuristics.size(); i++) {
			heuristics[i]->evaluate(node.get_state());
		   }
	           node.clear_h_dirty();
	           search_progress.inc_evaluations(heuristics.size());

		   open_list->evaluate(node.get_g(), false);
		   bool dead_end = open_list->is_dead_end();
		   if (dead_end) {
		       node.mark_as_dead_end();
		       search_progress.inc_dead_ends();
		       continue;
                   }
		   int new_h = heuristics[0]->get_heuristic();
		   if (new_h > node.get_h()) {
        	       assert(node.is_open());
                       node.increase_h(new_h);
		       open_list->insert(node.get_state_buffer());
                       continue;
                   }
		}
	    }
	    node.close();
	    assert(!node.is_dead_end());
            //update_jump_statistic(node);

	    search_progress.inc_expanded();
	    return make_pair(node, true);
	}
}


void DFSSearch::reward_progress() {
     open_list->boost_preferred();
}
/*
void DFSSearch::dump_search_space() {

}
*/

void DFSSearch::sample_frontier_now(int next_f_boundary) {
	cout<<" ________________________________"<<endl;
	cout<<"|   next_f_boundary              |"<<endl;
	cout<<" ________________________________"<<endl;
		  cout<<" ________________________________"<<endl;
	  cout<<"|   next_f_boundary              |"<<endl;
	  cout<<" ________________________________"<<endl;
	
	  //int next_f_boundary=open_list->get_f_boundary();
  vector<pair<State,int> > chosen_Hoff_Roots;//populates with selected Hoff Roots
  F_boundary_time=search_timer();//for the purpose of maximum sampling time
	  //int Hoff_Root_Range=open_list->open_list_get_next_boundary_range(); 
	  int Hoff_Root_Range=open_list->open_list_get_boundary_range(); 
	//cout<<"finished open_list_get_boundary_range"<<endl;fflush(stdout);
	  if(full_sampling){
	    leaves_to_sample=Hoff_Root_Range;
	  }
	  else{
	    leaves_to_sample=0.1*Hoff_Root_Range;
	  }
	  if(leaves_to_sample<100){
	    leaves_to_sample=min(100,Hoff_Root_Range);
	  }
	  cout<<"F_bound:"<<search_progress.return_lastjump_f_value()<<"F_boundary_time:"<<F_boundary_time<<",Hoff Potential Range:"<<Hoff_Root_Range<<",leaves_to_sample:"<<leaves_to_sample<<endl;

	//int max_position=Hoff_Root_Range-1;
	  if(Hoff_Root_Range>50){
	  //if((leaves_to_sample)<Hoff_Root_Range&&(Current_RIDA_Phase==SAMPLING_PHASE)&&F_boundary_time>5.0)
	    if(one_time_sampling==true){
	      first_sample=false;
	      cout<<"only sampling one iteration, set first_sample to false"<<endl;
	    }
    
	    Current_RIDA_Phase=SOLVING_PHASE;//needs to be changed for calcultate_heuristics function in Tree.CC
	    pair<State,int> Hoff_Root(*g_initial_state,0);//Need to intialize state, constructor does not allow empty state!
	    //calculate ratio that will approximately generate  the number of random Hoff Roots in a random pass
	    if(full_sampling){
	      leaf_selection_ratio=1.0;
	    }
	    else{
	      leaf_selection_ratio=double(leaves_to_sample)/double(Hoff_Root_Range);
	    }
	    cout<<"SAMPLING_PHASE,F:,"<<search_progress.return_lastjump_f_value()<<",Hoff Potential Range:"<<Hoff_Root_Range<<",leaves_to_sample:"<<leaves_to_sample<<",leaf_selection_ratio:,"<<leaf_selection_ratio<<endl;fflush(stdout);
	    for(int i=0;i<Hoff_Root_Range;i++){
	      //Hoff_Root=open_list->get_specific_f_boundary_states_and_depth(i,next_f_boundary);
	      Hoff_Root=open_list->get_specific_f_boundary_states_and_depth(i);
	      chosen_Hoff_Roots.push_back(Hoff_Root);
	      if(leaves_to_sample<=i){
		break;
	      }

	      /*  if(fRand(0.0,1.0)<=leaf_selection_ratio){
		Hoff_Root=open_list->get_specific_f_boundary_states_and_depth(i);
		//cout<<"\tadded Hoff Root#"<<i<<":;";Hoff_Root.first.dump_pddl();cout<<"g:"<<Hoff_Root.second<<endl;
		chosen_Hoff_Roots.push_back(Hoff_Root);
	      }*/
	    }
	    heuristics=orig_heuristics;
	    IDA_iter_sampling_timer.reset();IDA_iter_sampling_timer.resume();
	    //Options opt2;
	    //OptionParser parser;
	    //SearchEngine::add_options_to_parser(parser);
	    //HustSearch lsearch_space5(&opts2);
	    //HustSearch lsearch_space5();
	    //lsearch_space.select_best_estimated_heuristic_subset(&chosen_Hoff_Roots,orig_heuristics,heuristics);
     
	    cout<<"Memory before starting sampling:"<<get_peak_memory_in_kb()<<endl;
	    lsearch_space5.select_best_estimated_heuristic_subset(&search_space,&chosen_Hoff_Roots,orig_heuristics,heuristics,next_f_boundary, Hoff_Root_Range);
	    if(!one_time_sampling){
	      Current_RIDA_Phase=SAMPLING_PHASE;//needs to be changed for calcultate_heuristics function in Tree.CC
	    }
	    total_sampling_timer+=IDA_iter_sampling_timer.stop();
	    cout<<"sampling time until now:"<<total_sampling_timer<<endl;
	    if(full_sampling&&(!time_limit_node_adjusted)){//need to remove the sampling costs from the time limit
	      time_limit+=total_sampling_timer;
	      time_limit_node_adjusted=true;
	      cout<<"time_limit set to "<<time_limit<<"because when doing full sampling we ignore sampling costs"<<endl;
	    }
	  }
	  else{
	    cout<<"Need at least 10 or more F-boundary roots"<<endl;
	    return;
	  }
	cout<<"Memory after sampling:"<<get_peak_memory_in_kb()<<",active heurs:"<<heuristics.size()<<endl;
	for(int i=0;i<heuristics.size();i++){
	  cout<<"selected_heur("<<i<<") is:";heuristics[i]->print_heur_name();cout<<endl;
	}
}

/*
void DFSSearch::update_jump_statistic(const SearchNode &node) {	
	cout<<" __________________________________"<<endl;
	cout<<"|   update jump statistic          |"<<endl;
	cout<<" __________________________________"<<endl;
	//cout<<"node = "<<node<<endl;
}


void DFSSearch::print_heuristic_values(const vector<int> &values) const {

}
*/

static SearchEngine *_parse_ss(OptionParser &parser) {
	cout<<" ______________________________"<<endl;
	cout<<"|  parse_ss - dfs_search.cc     |"<<endl;
	cout<<" ______________________________"<<endl;
	parser.add_option<ScalarEvaluator *>("eval");
   	parser.add_option<bool>("pathmax", false,
                            "use pathmax correction");
    	parser.add_option<bool>("mpd", false,
                            "use multi-path dependence (LM-A*)");
    	parser.add_option<bool>("mark_children_as_finished", false,
                            "Only usefull for incremental_lmcut at the moment. Use together with reevaluate_parent to use incremental computation only for successor generation.");
	
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();

	DFSSearch *engine = 0;
	if (!parser.dry_run()) {
		cout<<"parser is not dry_run"<<endl;
		GEvaluator *g = new GEvaluator();
		cout<<"g object = "<<g<<endl;
		vector<ScalarEvaluator *> max_evals;
		cout<<"ScalarEvaluator vector pointer "<<max_evals<<endl;
		max_evals.push_back(g);
		cout<<"ScalarEvaluator vector pointer after add g = "<<max_evals<<endl;
		ScalarEvaluator *eval = opts.get<ScalarEvaluator *>("eval");
		max_evals.push_back(eval);
		cout<<"ScalarEvaluator vector pointer after add eval = "<<max_evals<<endl;
		ScalarEvaluator *f_eval = new MaxEvaluator(max_evals);
		//use eval for tiebreaking
		std::vector<ScalarEvaluator *> evals;
		evals.push_back(f_eval);
		evals.push_back(eval);
		cout<<"new vector of ScalarEvaluator created = "<<evals<<endl;
		OpenList<state_var_t *> *open = new TieBreakingOpenList<state_var_t *>(evals, false, false);
		cout<<"OpenList vector of state_var_t objects = "<<open<<endl;
		opts.set("open", open);
		//cout<<"opts vector after set open"<<opts<<endl;
		opts.set("f_eval",f_eval);
		opts.set("reopen_closed", true); 
		engine = new DFSSearch(opts);
		cout<<"engine = "<<engine<<endl;
	}	
	return engine;
}

static Plugin<SearchEngine> _plugin_ss("dfs", _parse_ss);

