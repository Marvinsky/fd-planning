#include "eager_search_kre.h"

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
using namespace std;

HST lsearch_space3(10000,1);
static bool time_limit_node_adjusted=false;

EagerSearchKRE::EagerSearchKRE(
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
    }
}
void EagerSearchKRE::output_problem_results(){
  cout<<"output problem results:"<<endl;
  ofstream outputFile;
  outputFile.open("results.txt",ios::app);
  //outputFile<<"src/search/downward --plan-file $1 --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
  outputFile<<g_plan_filename<<"\t"<<"0"<<"\t"<<search_progress.get_generated()<<"\t"<<search_timer();
  outputFile<<"\t"<<g_timer()<<endl;
      
  outputFile.close();
}

void EagerSearchKRE::initialize() {
    //TODO children classes should output which kind of search
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    cout<<"first_sample set to true"<<endl;
    first_sample=true;
    cout<<"first_time set to false and count_last_nodes_gerados to zero."<<endl;
    first_time=false;
    count_last_nodes_gerados = 0;
    if (do_pathmax)
        cout << "Using pathmax correction" << endl;
    if (use_multi_path_dependence)
        cout << "Using multi-path dependence (LM-A*)" << endl;
    assert(open_list != NULL);
    int i;
    for (i = 0; i < argc_copy; ++i) {
      puts(argv_copy[i]);
      //cout<<"i:"<<i<<","<<argv_copy[i]<<endl;
    }
    cout<<"Initial state:";g_initial_state->inline_dump();
    //cout<<"Goal state:"<<g_goal<<endl;
    
    set<Heuristic *> hset;
    cout<<"calling get_involved heuristics"<<endl;fflush(stdout);
    open_list->get_involved_heuristics(hset);
    cout<<"got involved heuristics"<<endl;fflush(stdout);
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
    cout<<"# heuristics:"<<heuristics.size()<<endl;fflush(stdout);
    
    //If conditional effects (IPC 2014) then we are left with just either hmax or blind
    bool conditional_effects_present=false;
    for (int i = 0; i < g_operators.size(); ++i) {
        const vector<PrePost> &pre_post = g_operators[i].get_pre_post();
        for (int j = 0; j < pre_post.size(); ++j) {
            const vector<Prevail> &cond = pre_post[j].cond;
            if (cond.empty())
                continue;
            // Accept conditions that are redundant, but nothing else.
            // In a better world, these would never be included in the
            // input in the first place.
            int var = pre_post[j].var;
            int pre = pre_post[j].pre;
            int post = pre_post[j].post;
            if (pre == -1 && cond.size() == 1 &&
                cond[0].var == var && cond[0].prev != post &&
                g_variable_domain[var] == 2)
                continue;
            cout << "Conditional effects present"<< endl;fflush(stdout);
	    conditional_effects_present=true;
	    break;
        }
	if(conditional_effects_present){
	  break;
	}
    }
    if(conditional_effects_present){
      orig_heuristics=heuristics;
      heuristics.clear();
      for (size_t i = 0; i < orig_heuristics.size(); i++){
	string heur_name=heuristics[i]->get_heur_name();
	if(heur_name.find("hmax")!=string::npos){
	  heuristics.push_back(orig_heuristics[i]);
	  cout<<"hmax can handle conditional effects, so keeping"<<endl;
	}
	else if(heur_name.find("blind")!=string::npos){
	  heuristics.push_back(orig_heuristics[i]);
	  cout<<"blind can handle conditional effects, so keeping"<<endl;
	  continue;
	}
	else{
	  cout<<"skipping because of conditional effects heuristic"<<orig_heuristics[i]->get_heur_name()<<endl;
	}
      }
      cout<<"# heuristics after eliminating those not supporting conditional effects:"<<heuristics.size()<<endl;
  }
    cout<<"starting timings"<<endl;fflush(stdout);

    //Timing node generation time
    Timer heur_timings;
    double node_counter=0;
    State s(*g_initial_state);
    vector<const Operator *> applicable_ops;
    const Operator *op;
    while(heur_timings()<1.0){//in situ measure node generation costs
      node_counter++;
      g_successor_generator->generate_applicable_ops(s, applicable_ops);
      op = applicable_ops[rand()%applicable_ops.size()];//choose operator at random
      State succ_state(s, *op);
      s=succ_state;
      }
    node_gen_and_exp_cost=heur_timings.stop()/node_counter;
    cout<<"node gen_and_exp_cost:"<<node_gen_and_exp_cost<<endl;
    node_time_adjusted_reval=0.5/node_gen_and_exp_cost;

    assert(!heuristics.empty());
    cout<<"active heuristics size:"<<heuristics.size()<<endl;
    
    int max_h=0;

//    //remove any heuristics which have been set to stop_using
//    heuristics.clear();
//    for (size_t i = 0; i < orig_heuristics.size(); i++){
//	    if(!heuristics[i]->is_using()){
//	      cout<<"removing heur";heuristics[i]->print_heur_name();
//	      heuristics[i]->free_up_memory(search_space);
//	      //string heur_name=heuristics[i]->get_heur_name();
//	      //if(heur_name.substr(0,13)=="heur:,lp_pdb,"){
//		//exit(0);
//	      //}
//	      continue;
//	    }
//	    else{
//	      heuristics[i]->evaluate(*g_initial_state);
//	      heuristics.push_back(orig_heuristics.at(i));
//	    }
//    }
    bool dead_end=false;
    for (size_t i = 0; i < heuristics.size(); i++){
	heuristics[i]->evaluate(*g_initial_state);
        cout<<"**************************"<<endl;
        cout<<"Setting initial h: "<<heuristics[i]->get_value()<<endl;
        cout<<"**************************"<<endl;
        SearchNode initialNode = search_space.get_node(*g_initial_state);
        initialNode.open_initial(heuristics[i]->get_value());

        v_f.push_back(initialNode.get_h() + initialNode.get_real_g());
        v_g.push_back(initialNode.get_real_g());
        v_h.push_back(initialNode.get_h());

	dead_end=heuristics[i]->is_dead_end();
	if(dead_end){
	  break;
	}
	max_h = max(max_h,heuristics[i]->get_heuristic());
    }
    open_list->evaluate2(0, max_h);
    search_progress.inc_evaluated_states();
    search_progress.inc_evaluations(heuristics.size());

    if (dead_end) {
        cout << "Initial state is a dead end." << endl;
                search_progress.inc_dead_ends();
    } else {
        search_progress.get_initial_h_values();
        if (f_evaluator) {
            f_evaluator->evaluate(0, false);
            search_progress.report_f_value(f_evaluator->get_value());
        }
        search_progress.check_h_progress(0);
        SearchNode node = search_space.get_node(*g_initial_state);
        node.open_initial(heuristics[0]->get_value());

        open_list->insert(node.get_state_buffer());
    }
    cout<<"starting timing individual heuristics"<<endl;fflush(stdout);
    //Time each heuristic
	double max_TPN=0;
	double aggr_TPN=0;
	//vector<pair<int,int> > lp_pdbs;//we greedily remove any lp_pdb with a higher pattern_size if the initial f-value is not increased as a result
	int lp_pdbs_max_h=0;
	int max_lp_pattern_size=0;
	vector<int> initial_h_values;
	for (size_t i = 0; i < heuristics.size(); i++){
	   // heuristics[i]->evaluate(*g_initial_state);
	    //cout<<"h[,"<<i<<",] time_cost is,"<<heuristics[i]->get_time_cost()<<endl;
	    if(Current_RIDA_Phase==SAMPLING_PHASE){
	      heur_timings.reset();heur_timings.resume();
	      double counter=0;
	      while(heur_timings()<0.1){//in situ measure heuristic evaluation costs
		counter++;
		heuristics[i]->evaluate(*g_initial_state);
	      }
	    
	      double TPN=heur_timings.stop()/counter;
	      max_TPN=max(TPN,max_TPN);
	      aggr_TPN+=TPN;
	      /*if(TPN<min_TPN){
		start_heur=i;//we choose the cheapest heuristic to be the "starting one"
		cout<<"h[,"<<i<<",] name:,";heuristics[i]->print_heur_name()<<" is start_heur"<<endl;
	      }*/
	      string heur_name=heuristics[i]->get_heur_name();
	      if(heur_name.substr(0,13)=="heur:,lp_pdb,"){
		//pair<int,int> temp_pair;
		//temp_pair.first=heuristics[i]->get_systematic_index();
		//temp_pair.second=heuristics[i]->get_heuristic();
		//lp_pdbs.push_back(temp_pair);
		cout<<"heuristic"<<i<<" is a lp_pdb"<<endl;
		if(lp_pdbs_max_h<heuristics[i]->get_heuristic()){
		  max_lp_pattern_size=heuristics[i]->get_systematic_index();
		  cout<<"new max_lp_pattern_size:"<<max_lp_pattern_size<<endl;
		}
		lp_pdbs_max_h=max(lp_pdbs_max_h,heuristics[i]->get_heuristic());
	      }
	      heuristics[i]->set_measured_TPN(TPN);
	      cout<<"h[,"<<i<<",] is:,";heuristics[i]->print_heur_name();cout<<",measured time cost:"<<heuristics[i]->get_measured_TPN()<<",h:"<<heuristics[i]->get_heuristic()<<endl;
	      initial_h_values.push_back(heuristics[i]->get_heuristic());
	    }
	    else{
	      initial_h_values.push_back(heuristics[i]->get_heuristic());
	      heuristics[i]->evaluate(*g_initial_state);
	      cout<<"h[,"<<i<<",] is:,";heuristics[i]->print_heur_name();fflush(stdout);cout<<",h:"<<heuristics[i]->get_heuristic()<<endl;fflush(stdout);
	    }
	    max_h=max(max_h,heuristics[i]->get_heuristic());
	}
	if(Current_RIDA_Phase==SAMPLING_PHASE){
	  //greedy drooping any heuristic which has lower h-value than any other heuristic whose evaluation time is 10 times or less lower
	  /*for (size_t i = 0; i < heuristics.size(); i++){
	    for (size_t j = 0; j < heuristics.size(); j++){
	      if(heuristics[i]->get_measured_TPN()>(10.0*(heuristics[j]->get_measured_TPN()))){
		if(initial_h_values.at(i)<initial_h_values.at(j)){//so heurisitic is less accurate and at least one order of magnitude more expensive!
		  cout<<heuristics[i]->get_heur_name()<<"is less accurate for initial state but at least 10 times more expensive than"<<heuristics[j]->get_heur_name()<<"so we drop it (greedily)"<<endl;
		  heuristics[i]->set_stop_using(true);
		  break;
		}
	      }
	    }
	  }*/

	    //Now we remove any lps which did not increase the maximum h_value
	    for (size_t i = 0; i < heuristics.size(); i++){
	      string heur_name=heuristics[i]->get_heur_name();
	      if(heur_name.substr(0,13)=="heur:,lp_pdb,"){
		if(heuristics[i]->get_systematic_index()>max_lp_pattern_size){
		  heuristics[i]->set_stop_using(true);
		}
	    }
	  }
    //remove any heuristics which have been set to stop_using
      heuristics.clear();
      cout<<"memory before deleting databases:"<<endl;cout << "VmRSS memory: " << get_memory_VmRSS() << " KB" << endl;
	  for (size_t i = 0; i < orig_heuristics.size(); i++){
		  if(!orig_heuristics[i]->is_using()){
		    cout<<"removing initial heur";orig_heuristics[i]->print_heur_name();cout<<endl;
		    orig_heuristics[i]->free_up_memory(search_space);
		    orig_heuristics[i]->free_up_memory();
		    //delete orig_heuristics[i];
		    //string heur_name=heuristics[i]->get_heur_name();
		    //if(heur_name.substr(0,13)=="heur:,lp_pdb,"){
		      //exit(0);
		    //}
		    continue;
		  }
		  else{
		    orig_heuristics[i]->evaluate(*g_initial_state);
		    heuristics.push_back(orig_heuristics.at(i));
		  }
	  }
      cout<<"memory after deleting all databases:"<<endl;cout << "VmRSS memory: " << get_memory_VmRSS() << " KB" << endl;
	}
    orig_heuristics.clear();

    orig_heuristics=heuristics;//we update the original heuristics by removing those eliminated in the initialization phase*/
    cout<<"Remaining heuristics:"<<endl;
    for (size_t i = 0; i < orig_heuristics.size(); i++){
      cout<<"remaining initial heur";orig_heuristics[i]->print_heur_name();cout<<endl;
    }

	node_time_adjusted_reval=2.0/(node_gen_and_exp_cost+max_TPN);
	node_time_adjusted_reval=min(max(node_time_adjusted_reval,10),1000);
	cout<<"node_time_adjusted_reval based on the min of half second node_gen and the most expensive heuristic or 1000 nodes:"<<node_time_adjusted_reval<<endl;fflush(stdout);
}


void EagerSearchKRE::statistics() const {
    search_progress.print_statistics();
    search_space.statistics();
}

int EagerSearchKRE::step() {

    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
      problem_was_solved=false;
      cout<<"failed to get n!"<<endl;
        return FAILED;
    }
    SearchNode node = n.first;
    
    cout<<"Node h = "<<node.get_h()<<", g = "<<node.get_real_g()<<", f = "<<node.get_h() + node.get_real_g()<<endl;

  //Every 2 secs aprox we check if we have done search for too long without selecting a subset
  //Note that timer checks can actually be quite expensive when node generation cost microseconds or less, that is why we only do this check 
  //every time we have generated enough nodes to cover approx 2 secs.  Modulus operation is very cheap.
  if(Current_RIDA_Phase==SAMPLING_PHASE){
    if(search_progress.get_generated()%node_time_adjusted_reval==0){
      if(search_timer()>200.0){
	cout<<"sample_frontier_now, actual time above the 200 secs limit maximizing all heuristics"<<",overall time:"<<g_timer()<<",search time:"<<search_timer()<<endl;
	if(gen_to_eval_ratio==0){
	  cout<<"setting gen_to_eval as the first F-boundary was not completed, doing early sampling"<<endl;
	  gen_to_eval_ratio=double(search_progress.get_generated())/double(search_progress.get_evaluated_states());
	  cout<<"gen_to_eval_ratio:"<<gen_to_eval_ratio<<endl;
	  }
	sample_frontier_now(node.get_g()+node.get_h());
      }
    }
  }

    State s = node.get_state();
	
  
    if (check_goal_and_set_plan(s)){
      cout<<"overall generated nodes to last iter:,"<<search_progress.get_generated()<<",search_time:,"<<search_timer()<<",overall time:,"<<g_timer()<<endl;
      problem_was_solved=true;
      if(Current_RIDA_Phase==SOLVING_PHASE){
	output_problem_results();
      }
     //have to determinate the level.
     int last_level = search_progress.return_lastjump_f_value();
     nivel = last_level;
     first_time = true;
     return IN_PROGRESS;
    }


    if (first_time) {	
        int last_level = search_progress.return_lastjump_f_value();
	if (nivel == last_level) {
	    //Add the increment here because in the statistic the last one is added.
	    count_last_nodes_gerados = count_last_nodes_gerados + 1;
	    return IN_PROGRESS;
	} else {
            		
            cout<<"count_last_nodes_gerados: "<<count_last_nodes_gerados<<endl;
            generateReport(v_f, v_h, v_g, nivel);
	    return SOLVED;
	}
    }



    vector<const Operator *> applicable_ops;
    set<const Operator *> preferred_ops;

    g_successor_generator->generate_applicable_ops(s, applicable_ops);
    
    // This evaluates the expanded state (again) to get preferred ops
    for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
     
        Heuristic *h = preferred_operator_heuristics[i];
        h->evaluate(s);
        if (!h->is_dead_end()) {
            // In an alternation search with unreliable heuristics, it is
            // possible that this heuristic considers the state a dead end.
            vector<const Operator *> preferred;
            h->get_preferred_operators(preferred);
            preferred_ops.insert(preferred.begin(), preferred.end());
        }
    }
    search_progress.inc_evaluations(preferred_operator_heuristics.size());
    cout<<" applicable_ops.size() =  "<<applicable_ops.size()<<endl;

    for (int i = 0; i < applicable_ops.size(); i++) {
     
        // HACK: should call this whenever memory is about to run out
      if(incremental_memory_limit){  
	for (int j = 0; j < heuristics.size(); j++) {
	      Heuristic *h = heuristics[j];
	      h->free_up_memory(search_space);	
	  }
	}
        const Operator *op = applicable_ops[i];

        if ((node.get_real_g() + op->get_cost()) >= bound)
            continue;

        State succ_state(s, *op);
      
	
        search_progress.inc_generated();
        //bool is_preferred = (preferred_ops.find(op) != preferred_ops.end());

        SearchNode succ_node = search_space.get_node(succ_state);
	   

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end()){
	     
            continue;
	}

        // update new path
        if (use_multi_path_dependence || succ_node.is_new()) {
            bool h_is_dirty = false;
            for (size_t i = 0; i < heuristics.size(); ++i) {
	    /*  if(Current_RIDA_Phase==SOLVING_PHASE){
	      cout<<"checking if h["<<i<<"] is dirty"<<endl;fflush(stdout);
	    }*/
                /*
                  Note that we can't break out of the loop when
                  h_is_dirty is set to true or use short-circuit
                  evaluation here. We must call reach_state for each
                  heuristic for its side effects.
                */
                if (heuristics[i]->reach_state(s, *op, succ_node.get_state()))
                    h_is_dirty = true;
            }
            if (h_is_dirty && use_multi_path_dependence)
                succ_node.set_h_dirty();
        }
	   

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.
            int succ_h=0;
	    bool dead_end=false;
            for (size_t i = 0; i < heuristics.size(); i++){
                heuristics[i]->evaluate(succ_state);
		dead_end=heuristics[i]->is_dead_end();
		if(dead_end){
		  //cout<<"State:";succ_state.inline_dump();cout<<" is a dead end according to :"<<heuristics[i]->get_heur_name()<<"so not adding to open list"<<endl;
		  if(Current_RIDA_Phase==SOLVING_PHASE){
		    break;
		  }
		  else{
		    succ_h=INT_MAX/2;//need to keep nodes in case we need to sample for one of the heuristics which do not know that the node is a dead end
		  }
		}
		succ_h = max(succ_h,heuristics[i]->get_heuristic());
		  
	    } 	
	    
    
            succ_node.clear_h_dirty();
            search_progress.inc_evaluated_states();
            search_progress.inc_evaluations(heuristics.size());

            // Note that we cannot use succ_node.get_g() here as the
            // node is not yet open. Furthermore, we cannot open it
            // before having checked that we're not in a dead end. The
            // division of responsibilities is a bit tricky here -- we
            // may want to refactor this later.
            //open_list->evaluate(node.get_g() + get_adjusted_cost(*op), is_preferred);
            open_list->evaluate2(node.get_g() + get_adjusted_cost(*op),succ_h);
	   
            //bool dead_end = open_list->is_dead_end();
            if (dead_end) {
                succ_node.mark_as_dead_end();
                search_progress.inc_dead_ends();
		//if(Current_RIDA_Phase==SOLVING_PHASE){
		//  cout<<"Node is dead end"<<endl;fflush(stdout);
 		//}
                continue;
            }

            //TODO:CR - add an ID to each state, and then we can use a vector to save per-state information
            //int succ_h = heuristics[0]->get_heuristic();
            if (do_pathmax) {
                if ((node.get_h() - get_adjusted_cost(*op)) > succ_h) {
                   
                    succ_h = node.get_h() - get_adjusted_cost(*op);
                    //heuristics[0]->set_evaluator_value(succ_h);
                    //open_list->evaluate(node.get_g() + get_adjusted_cost(*op), is_preferred);
                    open_list->evaluate2(node.get_g() + get_adjusted_cost(*op), succ_h);
                    search_progress.inc_pathmax_corrections();
                }
            }
            succ_node.open(succ_h, node, op);
	

	    // HACK try to maintain only met information for boundary nodes
            // only useful for incremental lmcut atm
            if (mark_children_as_finished) {
                for (size_t i = 0; i < heuristics.size(); i++) {
                    heuristics[i]->finished_state(succ_node.get_state(), succ_node.get_real_g() + succ_node.get_h(), true);
                }
            }
	    open_list->insert(succ_node.get_state_buffer());
	    
            cout<<"\tChild node h = "<<succ_node.get_h()<<", g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<endl;
            
            v_f.push_back(succ_node.get_h() + succ_node.get_real_g());
            v_g.push_back(succ_node.get_real_g());
            v_h.push_back(succ_node.get_h());
	
	    
            if (search_progress.check_h_progress(succ_node.get_g())) {
                reward_progress();
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
            	// We found a new cheapest path to an open or closed state.
		//if(Current_RIDA_Phase==SOLVING_PHASE){
		//  cout<<"Need to reopen node"<<endl;fflush(stdout);
		//}
            if (reopen_closed_nodes) {
                //TODO:CR - test if we should add a reevaluate flag and if it helps
                // if we reopen closed nodes, do that
                if (succ_node.is_closed()) {
                    /* TODO: Verify that the heuristic is inconsistent.
                     * Otherwise, this is a bug. This is a serious
                     * assertion because it can show that a heuristic that
                     * was thought to be consistent isn't. Therefore, it
                     * should be present also in release builds, so don't
                     * use a plain assert. */
                    //TODO:CR - add a consistent flag to heuristics, and add an assert here based on it
                    search_progress.inc_reopened();
                }
                succ_node.reopen(node, op);
                //heuristics[0]->set_evaluator_value(succ_node.get_h());
                // TODO: this appears fishy to me. Why is here only heuristic[0]
                // involved? Is this still feasible in the current version?
                //open_list->evaluate(succ_node.get_g(), is_preferred);
                open_list->evaluate2(succ_node.get_g(), succ_node.get_h());
                open_list->insert(succ_node.get_state_buffer());
	    } else {
                // if we do not reopen closed nodes, we just update the parent pointers
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back
                succ_node.update_parent(node, op);
            }
        }
    }
    for (size_t i = 0; i < heuristics.size(); i++) {
		//if(Current_RIDA_Phase==SOLVING_PHASE){
		//  cout<<"Getting Boundary information form incremental_lmcut"<<endl;fflush(stdout);
		//}
        // HACK try to maintain only met information for boundary nodes
        // only useful for incremental lmcut atm
        heuristics[i]->finished_state(s, node.get_real_g() + node.get_h(), true);
    }
		//if(Current_RIDA_Phase==SOLVING_PHASE){
		//  cout<<"Returning"<<endl;fflush(stdout);
		//}

    return IN_PROGRESS;
}

map<int, int> EagerSearchKRE::getFDistribution(vector<int> v_f_value) {
	map<int, int> m;
        for (int i = 0; i < v_f_value.size(); i++) {
            int a = v_f_value.at(i);
            int k = 1;
            for (int j = 0; j < v_f_value.size(); j++) {
	        int b = v_f_value.at(j);
                if (i != j ) {
                   if (a == b) {
                      k++;
                   }
                }
            }
            map<int, int>::iterator mIter = m.find(a);
            if (mIter == m.end()) {
               m.insert(pair<int, int>(a, k));
            }
        }
        return m;
}

void EagerSearchKRE::generateReport(vector<int> v_f, vector<int> v_h, vector<int> v_g, int threshold) {
	map<int, int> g;
        map<int, vector<int> > mapv_f;
        for (int i = 0; i < v_g.size(); i++) {
            int a = v_g.at(i);
            int k = 1;
            for (int j = 0; j < v_g.size(); j++) {
                int b = v_g.at(j);
                if (i != j) {
                    if (a == b) {
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
            int quantity = iter->second;

            cout<<"g = "<<level<<endl;
            vector<int> v;
            for (int i = 0; i < quantity; i++) {
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
        cout<<"f(camada)\t#nodes expanded\n";
        for (map<int, int>::iterator iter = dist.begin(); iter != dist.end(); iter++) {
            int f = iter->first;
            int q = iter->second;
            
            cout<<f<<"\t"<<q<<"\n";
        }

        cout<<"totalniveles: "<<(threshold - v_f.at(0)) + 1<<endl;
        for (map<int, int>::iterator iter2 = dist.begin(); iter2 != dist.end(); iter2++) {
            int f = iter2->first;
            int q = iter2->second;
            if (f <= threshold) {
               cout<<"fnivel: "<<f<<endl;
               cout<<"nodesGeneratedByLevel: "<<q<<endl;
               cout<<"time0: 1\n";
               cout<<"nodesGeneratedToTheLevel: 5\n";
            }
        }
}




pair<SearchNode, bool> EagerSearchKRE::fetch_next_node() {
    /* TODO: The bulk of this code deals with multi-path dependence,
       which is a bit unfortunate since that is a special case that
       makes the common case look more complicated than it would need
       to be. We could refactor this by implementing multi-path
       dependence as a separate search algorithm that wraps the "usual"
       search algorithm and adds the extra processing in the desired
       places. I think this would lead to much cleaner code. */

  //cout<<"fetching next node"<<endl;fflush(stdout);
    while (true) {
        if (open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;
            return make_pair(search_space.get_node(*g_initial_state), false);
        }
        vector<int> last_key_removed;
        State state(open_list->remove_min(
                        use_multi_path_dependence ? &last_key_removed : 0));
        SearchNode node = search_space.get_node(state);

        if (node.is_closed())
            continue;

        if (use_multi_path_dependence) {
	  cout<<"use_multi_path_dependence is on,sort out how to use multiple heuristics"<<endl;exit(0);
            assert(last_key_removed.size() == 2);
            int pushed_h = last_key_removed[1];
            assert(node.get_h() >= pushed_h);
            if (node.get_h() > pushed_h) {
                // cout << "LM-A* skip h" << endl;
                continue;
            }
            assert(node.get_h() == pushed_h);
            if (!node.is_closed() && node.is_h_dirty()) {
                for (size_t i = 0; i < heuristics.size(); i++)
                    heuristics[i]->evaluate(node.get_state());
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
        update_jump_statistic(node);
	/*if(Current_RIDA_Phase==SOLVING_PHASE){
	  cout<<"finished SAMPLING"<<endl;
	}*/
        search_progress.inc_expanded();
        return make_pair(node, true);
    }
}

void EagerSearchKRE::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}

void EagerSearchKRE::dump_search_space() {
    search_space.dump();
}

void EagerSearchKRE::sample_frontier_now(int next_f_boundary) {
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
	    //HustSearch lsearch_space(&opts2);
	    //HustSearch lsearch_space();
	    //lsearch_space.select_best_estimated_heuristic_subset(&chosen_Hoff_Roots,orig_heuristics,heuristics);
     
	    cout<<"Memory before starting sampling:"<<get_peak_memory_in_kb()<<endl;
	    lsearch_space3.select_best_estimated_heuristic_subset(&search_space,&chosen_Hoff_Roots,orig_heuristics,heuristics,next_f_boundary, Hoff_Root_Range);
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
void EagerSearchKRE::update_jump_statistic(const SearchNode &node) {
  //vector<int> heuristics_to_drop;
    if (f_evaluator) {
      vector<pair<State,int> > chosen_Hoff_Roots;//populates with randomly selected Hoff Roots
        /*  heuristics[0]->set_evaluator_value(node.get_h());
        f_evaluator->evaluate(node.get_g(), false);
        int new_f_value = f_evaluator->get_value();
        search_progress.report_f_value(new_f_value);*/


      int new_f_value = node.get_g()+node.get_h();
      cout<<"new_f_value:"<<new_f_value<<endl;
      int last_level = search_progress.return_lastjump_f_value();
      cout<<" a que nivel pertence este nodo, nivel = "<<last_level<<endl; 
      
      /*
      bool entrance = true;
      if (first_time) {
	if (nivel == last_level) {
	    entrance = true;
            count_last_nodes_gerados = count_last_nodes_gerados + 1;
	} else {
	    entrance = false;
	}
      }
      */ 

      //cout<<"count_last_nodes_gerados: "<<count_last_nodes_gerados<<endl;

      if(search_progress.updated_lastjump_f_value(new_f_value)){
	vniveles.push_back(new_f_value);
	cout<<"______________________________________________________"<<endl;
	cout<<"Generate report_f_value passing "<<new_f_value<<endl;
	search_progress.report_f_value(new_f_value);
	cout<<"______________________________________________________"<<endl;
	cout<<"F_bound:"<<new_f_value<<",Peak memory="<<get_peak_memory_in_kb()/1024.0<<",nodes:"<<search_space.size()<<",Nodes mem_space:"<<search_space.size()*(sizeof(StateProxy)+sizeof(SearchNodeInfo))/1024.0<<",F_boundary_Range:"<<open_list->open_list_get_boundary_range()<<endl;
	///cout<<"F_bound:"<<new_f_value<<",Peak memory="<<get_peak_memory_in_kb()/1024.0<<",nodes:"<<search_space.size()<<",Nodes mem_space:"<<search_space.size()*(sizeof(: public __gnu_cxx::hash_map<StateProxy,SearchNodeInfo>))/1024.0<<endl;
    //: public __gnu_cxx::hash_map<StateProxy, SearchNodeInfo> 
    
	/*if(new_f_value%2==1){
	  heuristics.pop_back();
	  cout<<"dropped last heuristic,new size:"<<heuristics.size()<<endl;
	}
	else{
	  heuristics=orig_heuristics;
	  cout<<" heuristics set back to original,new size:"<<heuristics.size()<<endl;
	}*/
	//open_list->open_list_boundary_print(new_f_value);
	//cout<<"calling open_list_get_boundary_range"<<endl;fflush(stdout);
	if(Current_RIDA_Phase==SAMPLING_PHASE){
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
	  cout<<"F_bound:"<<new_f_value<<"F_boundary_time:"<<F_boundary_time<<",Hoff Potential Range:"<<Hoff_Root_Range<<",leaves_to_sample:"<<leaves_to_sample<<endl;

	//int max_position=Hoff_Root_Range-1;
	//choose random Hoff Roots iff potential roots bigger than minimum hoff roots to expand
	//if(new_f_value==225&&first_sample){
	 if(Hoff_Root_Range>50&&F_boundary_time>3.0&&(first_sample)){
	  //if((leaves_to_sample)<Hoff_Root_Range&&(Current_RIDA_Phase==SAMPLING_PHASE)&&F_boundary_time>5.0)
	    if(one_time_sampling==true){
	      first_sample=false;
	      cout<<"only sampling one iteration, set first_sample to false"<<endl;
	    }
    
	    Current_RIDA_Phase=SOLVING_PHASE;//needs to be changed for calcultate_heuristics function in Tree.CC
	    State s = node.get_state();
	    pair<State,int> Hoff_Root(s,node.get_g());//first state is missing from F-boundary because we already pulled it
	    chosen_Hoff_Roots.push_back(Hoff_Root);//So the first state pulled is added to the chosen Hoff roots, that way it is never missing from the available roots

	    //calculate ratio that will approximately generate  the number of random Hoff Roots in a random pass
	    if(full_sampling){
	      //leaf_selection_ratio=1.0;
	      Hoff_Root_Range=open_list->get_F_boundaries_size();
	      cout<<"All frontier range:"<<Hoff_Root_Range<<endl;
	      for(int i=0;i<Hoff_Root_Range;i++){
		Hoff_Root=open_list->get_specific_all_boundaries_state_and_depth(i);
		chosen_Hoff_Roots.push_back(Hoff_Root);
	      }
	    }
	    else{
	      leaf_selection_ratio=double(leaves_to_sample)/double(Hoff_Root_Range);
	    cout<<"SAMPLING_PHASE,F:,"<<new_f_value<<",Hoff Potential Range:"<<Hoff_Root_Range<<",leaves_to_sample:"<<leaves_to_sample<<",leaf_selection_ratio:,"<<leaf_selection_ratio<<endl;fflush(stdout);
	      for(int i=0;i<Hoff_Root_Range;i++){
		Hoff_Root=open_list->get_specific_f_boundary_states_and_depth(i);
		/*chosen_Hoff_Roots.push_back(Hoff_Root);
		if(leaves_to_sample<=i){
		  break;
		}*/

		  if(fRand(0.0,1.0)<=leaf_selection_ratio){
		  Hoff_Root=open_list->get_specific_f_boundary_states_and_depth(i);
		  //cout<<"\tadded Hoff Root#"<<i<<":;";Hoff_Root.first.dump_pddl();cout<<"g:"<<Hoff_Root.second<<endl;
		  chosen_Hoff_Roots.push_back(Hoff_Root);
		}
	      }
	    }
	    heuristics=orig_heuristics;
	    IDA_iter_sampling_timer.reset();IDA_iter_sampling_timer.resume();
	    //Options opt2;
	    //OptionParser parser;
	    //SearchEngine::add_options_to_parser(parser);
	    //HustSearch lsearch_space(&opts2);
	    //HustSearch lsearch_space();
	    //lsearch_space.select_best_estimated_heuristic_subset(&chosen_Hoff_Roots,orig_heuristics,heuristics);
     
	    cout<<"Memory before starting sampling:"<<get_peak_memory_in_kb()<<endl;
	    lsearch_space3.select_best_estimated_heuristic_subset(&search_space,&chosen_Hoff_Roots,orig_heuristics,heuristics,new_f_value,Hoff_Root_Range);
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
	cout<<"new F_bound:"<<new_f_value<<"chosen_Hoff_Roots_size:"<<chosen_Hoff_Roots.size()<<" out of "<<Hoff_Root_Range<<endl;
	cout<<"Memory after Sampling::"<<get_peak_memory_in_kb()<<endl;
	}
	chosen_Hoff_Roots.clear();
	cout<<"Memory before starting new F-boundary:"<<get_peak_memory_in_kb()<<endl;
      }
    }
}

void EagerSearchKRE::print_heuristic_values(const vector<int> &values) const {
    for (int i = 0; i < values.size(); i++) {
        cout << values[i];
        if (i != values.size() - 1)
            cout << "/";
    }
}

static SearchEngine *_parse(OptionParser &parser) {
    //open lists are currently registered with the parser on demand,
    //because for templated classes the usual method of registering
    //does not work:
    Plugin<OpenList<state_var_t *> >::register_open_lists();

    parser.add_option<OpenList<state_var_t *> *>("open");
    parser.add_option<bool>("reopen_closed", false,
                            "reopen closed nodes");
    parser.add_option<bool>("pathmax", false,
                            "use pathmax correction");
    parser.add_option<ScalarEvaluator *>("f_eval", 0,
                                         "set evaluator for jump statistics");
    parser.add_list_option<Heuristic *>
        ("preferred", vector<Heuristic *>(),
        "use preferred operators of these heuristics");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    EagerSearchKRE *engine = 0;
    if (!parser.dry_run()) {
        opts.set<bool>("mpd", false);
        engine = new EagerSearchKRE(opts);
    }

    return engine;
}

static SearchEngine *_parse_astarkre(OptionParser &parser) {
  cout<<"calling parse_astarkre"<<endl;
    parser.add_option<ScalarEvaluator *>("eval");
    parser.add_option<bool>("pathmax", false,
                            "use pathmax correction");
    parser.add_option<bool>("mpd", false,
                            "use multi-path dependence (LM-A*)");
    parser.add_option<bool>("mark_children_as_finished", false,
                            "Only usefull for incremental_lmcut at the moment. Use together with reevaluate_parent to use incremental computation only for successor generation.");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    EagerSearchKRE *engine = 0;
    if (!parser.dry_run()) {
      cout<<"parser is not dry_run"<<endl;
        GEvaluator *g = new GEvaluator();
        vector<ScalarEvaluator *> max_evals;
        max_evals.push_back(g);
        ScalarEvaluator *eval = opts.get<ScalarEvaluator *>("eval");
        max_evals.push_back(eval);
        ScalarEvaluator *f_eval = new MaxEvaluator(max_evals);

        // use eval for tiebreaking
        std::vector<ScalarEvaluator *> evals;
        evals.push_back(f_eval);
        evals.push_back(eval);
        OpenList<state_var_t *> *open = \
            new TieBreakingOpenList<state_var_t *>(evals, false, false);

        opts.set("open", open);
        opts.set("f_eval", f_eval);
        opts.set("reopen_closed", true);
        engine = new EagerSearchKRE(opts);
    }

    return engine;
}

static SearchEngine *_parse_greedy(OptionParser &parser) {
    parser.add_list_option<ScalarEvaluator *>("evals");
    parser.add_list_option<Heuristic *>("preferred", vector<Heuristic *>(), "use preferred operators of these heuristics");
    parser.add_option<int>("boost", 0, "boost value for preferred operator open lists");
    SearchEngine::add_options_to_parser(parser);


    Options opts = parser.parse();
    opts.verify_list_non_empty<ScalarEvaluator *>("evals");

    EagerSearchKRE *engine = 0;
    if (!parser.dry_run()) {
        vector<ScalarEvaluator *> evals =
            opts.get_list<ScalarEvaluator *>("evals");
        vector<Heuristic *> preferred_list =
            opts.get_list<Heuristic *>("preferred");
        OpenList<state_var_t *> *open;
        if ((evals.size() == 1) && preferred_list.empty()) {
            open = new StandardScalarOpenList<state_var_t *>(evals[0], false);
        } else {
            vector<OpenList<state_var_t *> *> inner_lists;
            for (int i = 0; i < evals.size(); i++) {
                inner_lists.push_back(
                    new StandardScalarOpenList<state_var_t *>(evals[i], false));
                if (!preferred_list.empty()) {
                    inner_lists.push_back(
                        new StandardScalarOpenList<state_var_t *>(evals[i],
                                                                  true));
                }
            }
            open = new AlternationOpenList<state_var_t *>(
                inner_lists, opts.get<int>("boost"));
        }

        opts.set("open", open);
        opts.set("reopen_closed", false);
        opts.set("pathmax", false);
        opts.set("mpd", false);
        ScalarEvaluator *sep = 0;
        opts.set("f_eval", sep);
        opts.set("bound", numeric_limits<int>::max());
        opts.set("preferred", preferred_list);
        engine = new EagerSearchKRE(opts);
    }
    return engine;
}

static Plugin<SearchEngine> _plugin("eager", _parse);
static Plugin<SearchEngine> _plugin_astarkre("astarkre", _parse_astarkre);
static Plugin<SearchEngine> _plugin_greedy("eager_greedy", _parse_greedy);
