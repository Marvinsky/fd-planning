#include "speed_progress.h"

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

#include <dirent.h>

using namespace std;


SpeedProgress::SpeedProgress(
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

void SpeedProgress::initialize() {
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    if (do_pathmax)
        cout << "Using pathmax correction" << endl;
    if (use_multi_path_dependence)
        cout << "Using multi-path dependence (LM-A*)" << endl;
    assert(open_list != NULL);

    set<Heuristic *> hset;
    open_list->get_involved_heuristics(hset);

    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); ++it) {
        estimate_heuristics.push_back(*it);
        search_progress.add_heuristic(*it); 
    }

    hset.insert(preferred_operator_heuristics.begin(),
                preferred_operator_heuristics.end());

    if (f_evaluator) {
       f_evaluator->get_involved_heuristics(hset);
    }

    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); ++it) {
        heuristics.push_back(*it);
    }

    const State &initial_state = *g_initial_state;
    for (int i = 0; i < heuristics.size(); ++i) {
         heuristics[i]->evaluate(initial_state);
    }
    open_list->evaluate(0, false);
    search_progress.inc_evaluated_states();
    search_progress.inc_evaluations(heuristics.size());

    if (open_list->is_dead_end()) {
       cout<<"Initial state is a dead end."<<endl;
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
}

int SpeedProgress::step() {
	

    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;

    cout<<"\nRaiz node h = "<<node.get_h()<<",g = "<<node.get_real_g()<<", f = "<<node.get_h() + node.get_real_g()<<endl;
 
    State s = node.get_state();
    if (check_goal_and_set_plan(s))
        return SOLVED;

    vector<const Operator *> applicable_ops;
    set<const Operator *> preferred_ops;

    g_successor_generator->generate_applicable_ops(s, applicable_ops);
    // This evaluates the expanded state (again) to get preferred ops
    for (size_t i = 0; i < preferred_operator_heuristics.size(); ++i) {
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

    for (size_t i = 0; i < applicable_ops.size(); ++i) {
        const Operator *op = applicable_ops[i];

        if ((node.get_real_g() + op->get_cost()) >= bound)
            continue;


        State succ_state(s, *op);
        search_progress.inc_generated();
        bool is_preferred = (preferred_ops.find(op) != preferred_ops.end());

        SearchNode succ_node = search_space.get_node(succ_state);

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end())
            continue;

        // update new path
        if (use_multi_path_dependence || succ_node.is_new()) {
            bool h_is_dirty = false;
            for (size_t j = 0; j < heuristics.size(); ++j) {
                /*
                  Note that we can't break out of the loop when
                  h_is_dirty is set to true or use short-circuit
                  evaluation here. We must call reach_state for each
                  heuristic for its side effects.
                */
                if (heuristics[j]->reach_state(s, *op, succ_state))
                    h_is_dirty = true;
            }
            if (h_is_dirty && use_multi_path_dependence)
                succ_node.set_h_dirty();
        }

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.
            for (size_t j = 0; j < heuristics.size(); ++j)
                heuristics[j]->evaluate(succ_state);
            succ_node.clear_h_dirty();
            search_progress.inc_evaluated_states();
            search_progress.inc_evaluations(heuristics.size());

            // Note that we cannot use succ_node.get_g() here as the
            // node is not yet open. Furthermore, we cannot open it
            // before having checked that we're not in a dead end. The
            // division of responsibilities is a bit tricky here -- we
            // may want to refactor this later.
            open_list->evaluate(node.get_g() + get_adjusted_cost(*op), is_preferred);
            bool dead_end = open_list->is_dead_end();
            if (dead_end) {
                succ_node.mark_as_dead_end();
                search_progress.inc_dead_ends();
                continue;
            }

            //TODO:CR - add an ID to each state, and then we can use a vector to save per-state information
            int succ_h = heuristics[0]->get_heuristic();
            if (do_pathmax) {
                if ((node.get_h() - get_adjusted_cost(*op)) > succ_h) {
                    //cout << "Pathmax correction: " << succ_h << " -> " << node.get_h() - get_adjusted_cost(*op) << endl;
                    succ_h = node.get_h() - get_adjusted_cost(*op);
                    heuristics[0]->set_evaluator_value(succ_h);
                    open_list->evaluate(node.get_g() + get_adjusted_cost(*op), is_preferred);
                    search_progress.inc_pathmax_corrections();
                }
            }
            succ_node.open(succ_h, node, op);

            open_list->insert(succ_node.get_state_buffer());

            cout<<"\tChild node h = "<<succ_node.get_h()<<",g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<" m&s h+g = "<<succ_node.get_h()+succ_node.get_real_g()<<endl;
                           
            Node2 node2(succ_node.get_h() + succ_node.get_real_g(), succ_node.get_real_g());
            if (collector.insert(pair<Node2, int>(node2, count_value)).second) {
                count_value = 1;
            } else {
                map<Node2, int>::iterator iter = collector.find(node2);
                int q = iter->second;
                q++;
                iter->second = q;
            }

            if (search_progress.check_h_progress(succ_node.get_g())) {
                reward_progress();
            }
        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
            // We found a new cheapest path to an open or closed state.
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
                heuristics[0]->set_evaluator_value(succ_node.get_h());
                // TODO: this appears fishy to me. Why is here only heuristic[0]
                // involved? Is this still feasible in the current version?
                open_list->evaluate(succ_node.get_g(), is_preferred);

                open_list->insert(succ_node.get_state_buffer());
            } else {
                // if we do not reopen closed nodes, we just update the parent pointers
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back
                succ_node.update_parent(node, op);
            }
        }
    }

    return IN_PROGRESS;	
	
	
	
    /*	
    pair<SearchNode, bool> n = fetch_next_node();
    cout<<"number sixty two."<<endl;           

    if (!n.second) {
      problem_was_solved=false;
      cout<<"failed to get n!"<<endl;
        return FAILED;
    }
    SearchNode node = n.first;
    
    cout<<"\nRaiz node h = "<<node.get_h()<<",g = "<<node.get_real_g()<<", f = "<<node.get_h() + node.get_real_g()<<endl;
    
    State s = node.get_state();
    

    if (check_goal_and_set_plan(s)){
        
        cout<<"overall generated nodes to last iter:,"<<search_progress.get_generated()<<",search_time:,"<<search_timer()<<",overall time:,"<<g_timer()<<endl;
        problem_was_solved=true;

        
        cout<<"\nCount the nodes in the last level."<<endl;
	
	int last_level = search_progress.return_lastjump_f_value();
	nivel = last_level;
	first_time = true;
          
	return IN_PROGRESS;
    }


    if (first_time) {
       int last_level = search_progress.return_lastjump_f_value();
       cout<<"nivel = "<<nivel<<endl;
       cout<<"last_level = "<<last_level<<endl;
       if (nivel == last_level) {
	  //Add the increment here because in the statistic the last one is added.
          count_last_nodes_gerados = count_last_nodes_gerados + 1;
           
          return IN_PROGRESS;
       } else {
	  
          cout<<"count_last_nodes_gerados: "<<count_last_nodes_gerados<<endl;
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

    for (int i = 0; i < applicable_ops.size(); i++) {
        
         const Operator *op = applicable_ops[i];
        
         if((node.get_real_g() + op->get_cost()) >= bound)
             continue;

         State succ_state(s, *op);
      
         search_progress.inc_generated();
     
         SearchNode succ_node = search_space.get_node(succ_state);
	
         if (succ_node.is_dead_end()){
	    
             continue;
 	 }

         if (succ_node.is_new()) {
           
            int succ_h=0;
	    heuristics[i]->evaluate(succ_state);
                 
	    succ_h =  max(succ_h,heuristics[i]->get_heuristic());
              
            succ_node.clear_h_dirty();
            search_progress.inc_evaluated_states();
            search_progress.inc_evaluations(heuristics.size());

            open_list->evaluate2(node.get_g() + get_adjusted_cost(*op),succ_h);
	   
            succ_node.open(succ_h, node, op);

	    open_list->insert(succ_node.get_state_buffer());
	  

	    cout<<"\tChild node h = "<<succ_node.get_h()<<",g = "<<succ_node.get_real_g()<<", f = "<<succ_node.get_h() + succ_node.get_real_g()<<" m&s h+g = "<<succ_node.get_h()+succ_node.get_real_g()<<endl;
                           
            Node2 node2(succ_node.get_h() + succ_node.get_real_g(), succ_node.get_real_g());
            if (collector.insert(pair<Node2, int>(node2, count_value)).second) {
                count_value = 1;
            } else {
                map<Node2, int>::iterator iter = collector.find(node2);
                int q = iter->second;
                q++;
                iter->second = q;
            }
 
            if (search_progress.check_h_progress(succ_node.get_g())) {
                reward_progress();
            }
         }
    }

    return IN_PROGRESS;
    */
     
}

int SpeedSearch::returnMaxF(vector<int> levels) {
      int max = levels.at(0);
      for (int i = 0; i < levels.size(); i++) {
          if (max < levels.at(i)) {
             max = levels.at(i);
          }
      }
      return max;
}

int SpeedSearch::returnMinF(vector<int> levels) {
      int min = levels.at(0);
      for (int i = 0; i < levels.size(); i++) {
          if (min > levels.at(i)) {
             min = levels.at(i);
          }
      }
      return min;
}


void EagerSearch::generateReport() {
      cout<<"collector.size() = "<<collector.size()<<endl;
      vector<int> levels;
      map<int, int> mlevels;
      int count_level = 0;
      for (map<Node2, int>::iterator iter = collector.begin(); iter !=  collector.end(); iter++) {
          Node2 n = iter->first;
          levels.push_back(n.getF());
          
          map<int, int>::iterator iter2 = mlevels.find(n.getF());
          if ((iter2 == mlevels.end()) && n.getF() <= nivel) {
             mlevels.insert(pair<int, int>(n.getF(), count_level));
             count_level++;
          }
      }

      int depth = returnMaxF(levels);
      //int minDepth = returnMinF(levels);
      cout<<"mlevels.size() = "<<mlevels.size()<<endl;
      cout<<"count_level = "<<count_level<<endl;
      map<int, int> m;

      string dominio = domain_name;
      string tarefa = problem_name2;
      string heuristica = heuristic_name2;
      cout<<"dominio = "<<dominio<<endl;
      cout<<"tarefa = "<<tarefa<<endl;
      cout<<"heuristica = "<<heuristica<<endl;

      string directoryDomain = "mkdir /home/marvin/marvin/test/"+heuristica+"/krereport/"+dominio;
      if (system(directoryDomain.c_str())) {
         cout<<"Directory created successfully."<<endl;
      }
      
      string nBL = "/home/marvin/marvin/test/"+heuristica+"/krereport/"+dominio+"/"+tarefa; 

      ofstream outputFile;
      outputFile.open(nBL.c_str(), ios::out);
      outputFile<<"\t\t"<<nBL.c_str()<<"\n";
      outputFile<<"\ttotalniveles: "<<mlevels.size()<<"\n";
       
      outputFile<<"\tf-value\t\t#nodesByLevel\t\ttime\t\t#nodesExpanded\n";

      for (int i = 0; i <= depth; i++) {
          int k = 0;
          for (map<Node2, int>::iterator iter = collector.begin(); iter !=  collector.end(); iter++) {
              Node2 n = iter->first;
              if (i == n.getF()) {
                  k = k + iter->second;
              }
          }
          map<int, int>::iterator iter = m.find(i);
          if (iter == m.end()) {
             m.insert(pair<int, int>(i, k));
          }
      }
      cout<<"print v_timer"<<endl;
      for (int i = 0; i < v_timer.size(); i++) {
          cout<<v_timer.at(i)<<endl;
      }     

      int sum = 0;
      int count_v_timer = 0;
      for (map<int, int>::iterator iter = m.begin(); iter != m.end(); iter++) {
          int f = iter->first;
          int q = iter->second;
          
          if ((f <= nivel) && (q != 0) ) {
             cout<<"f = "<<f<<"\tq = "<<q<<endl;
             sum = sum + q;
             outputFile<<"\t"<<f<<"\t\t"<<q<<"\t\t\t"<<v_timer.at(count_v_timer)<<"\t\t\t"<<sum<<"\n";
             count_v_timer++;
          }
      }

      outputFile.close();
}



pair<SearchNode, bool> SpeedProgress::fetch_next_node() {
   
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
        
        search_progress.inc_expanded();
        return make_pair(node, true);
    }
}

void SpeedProgress::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // one of the heuristics finds a state with a new best h value.
    open_list->boost_preferred();
}


void SpeedProgress::update_jump_statistic(const SearchNode &node) {
  //vector<int> heuristics_to_drop;
    if (f_evaluator) {
      vector<pair<State,int> > chosen_Hoff_Roots;//populates with randomly selected Hoff Roots
        /*  heuristics[0]->set_evaluator_value(node.get_h());
        f_evaluator->evaluate(node.get_g(), false);
        int new_f_value = f_evaluator->get_value();
        search_progress.report_f_value(new_f_value);*/
      int new_f_value = node.get_g()+node.get_h();
      //cout<<"new_f_value:"<<new_f_value<<endl; 
      
      if(search_progress.updated_lastjump_f_value(new_f_value)){
        double level_update_time = time_level();
        cout<<"level_update_time = "<<level_update_time<<endl;
        v_timer.push_back(level_update_time);

	search_progress.report_f_value(new_f_value);
      }
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

    SpeedProgress *engine = 0;
    if (!parser.dry_run()) {
        opts.set<bool>("mpd", false);
        engine = new SpeedProgress(opts);
    }

    return engine;
}

static SearchEngine *_parse_astar(OptionParser &parser) {
  cout<<"calling parse_astar"<<endl;
    parser.add_option<ScalarEvaluator *>("eval");
    parser.add_option<bool>("pathmax", false,
                            "use pathmax correction");
    parser.add_option<bool>("mpd", false,
                            "use multi-path dependence (LM-A*)");
    parser.add_option<bool>("mark_children_as_finished", false,
                            "Only usefull for incremental_lmcut at the moment. Use together with reevaluate_parent to use incremental computation only for successor generation.");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    SpeedProgress *engine = 0;
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
        engine = new SpeedProgress(opts);
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

    SpeedProgress *engine = 0;
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
        engine = new SpeedProgress(opts);
    }
    return engine;
}

static Plugin<SearchEngine> _plugin("eager", _parse);
static Plugin<SearchEngine> _plugin_astar("astar_speed", _parse_astar);
static Plugin<SearchEngine> _plugin_greedy("eager_greedy", _parse_greedy);
