#include "lazy_search.h"

#include "g_evaluator.h"
#include "heuristic.h"
#include "successor_generator.h"
#include "sum_evaluator.h"
#include "weighted_evaluator.h"
#include "plugin.h"

#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>

static const int DEFAULT_LAZY_BOOST = 1000;

LazySearch::LazySearch(const Options &opts)
    : SearchEngine(opts),
      open_list(opts.get<OpenList<OpenListEntryLazy> *>("open")),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      succ_mode(pref_first),
      current_state(*g_initial_state),
      current_predecessor_buffer(NULL),
      current_operator(NULL),
      current_g(0),
      current_real_g(0) {
}

LazySearch::~LazySearch() {
}

void LazySearch::set_pref_operator_heuristics(
    vector<Heuristic *> &heur) {
    preferred_operator_heuristics = heur;
}

void LazySearch::initialize() {
    //TODO children classes should output which kind of search
    cout << "Conducting lazy best first search, (real) bound = " << bound << endl;

    assert(open_list != NULL);
    set<Heuristic *> hset;
    open_list->get_involved_heuristics(hset);

    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
        estimate_heuristics.push_back(*it);
        search_progress.add_heuristic(*it);
    }

    // add heuristics that are used for preferred operators (in case they are
    // not also used in the open list)
    hset.insert(preferred_operator_heuristics.begin(),
                preferred_operator_heuristics.end());

    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
        heuristics.push_back(*it);
    }
    assert(!heuristics.empty());
    int total_heuristics = heuristics.size();
    if (total_heuristics == 2) {
       two_heuristics = true;
    } else {
       two_heuristics = false;
    }
    cout<<"total_heuristics = "<<total_heuristics<<endl;
}

void LazySearch::get_successor_operators(vector<const Operator *> &ops) {
    vector<const Operator *> all_operators;
    vector<const Operator *> preferred_operators;

    g_successor_generator->generate_applicable_ops(
        current_state, all_operators);

    for (int i = 0; i < preferred_operator_heuristics.size(); i++) {
        Heuristic *heur = preferred_operator_heuristics[i];
        if (!heur->is_dead_end())
            heur->get_preferred_operators(preferred_operators);
    }

    if (succ_mode == pref_first) {
        for (int i = 0; i < preferred_operators.size(); i++) {
            if (!preferred_operators[i]->is_marked()) {
                ops.push_back(preferred_operators[i]);
                preferred_operators[i]->mark();
            }
        }

        for (int i = 0; i < all_operators.size(); i++)
            if (!all_operators[i]->is_marked())
                ops.push_back(all_operators[i]);
    } else {
        for (int i = 0; i < preferred_operators.size(); i++)
            if (!preferred_operators[i]->is_marked())
                preferred_operators[i]->mark();
        ops.swap(all_operators);
        if (succ_mode == shuffled)
            random_shuffle(ops.begin(), ops.end());
    }
}

void LazySearch::generate_successors() {
    vector<const Operator *> operators;
    get_successor_operators(operators);
    search_progress.inc_generated(operators.size());

    state_var_t *current_state_buffer =
        search_space.get_node(current_state).get_state_buffer();
    cout<<"\t\t\tSuccessors: "<<endl;
    for (int i = 0; i < operators.size(); i++) {
        int new_g = current_g + get_adjusted_cost(*operators[i]);
        int new_real_g = current_real_g + operators[i]->get_cost();
        cout<<"\t\t\tnew_g = "<<new_g<<endl;
        cout<<"\t\t\tnew_real_g = "<<new_real_g<<endl;
        bool is_preferred = operators[i]->is_marked();
        cout<<"\t\t\tis_preferred = "<<is_preferred<<endl;
        if (is_preferred)
            operators[i]->unmark();
        if (new_real_g < bound) {
            cout<<"\t\t\t\tnew_g evaluated is = "<<new_g<<endl;
            open_list->evaluate(new_g, is_preferred);
            open_list->insert(
                make_pair(current_state_buffer, operators[i]));
        }
    }
}

int LazySearch::fetch_next_state() {
    if (open_list->empty()) {
        cout << "Completely explored state space -- no solution!" << endl;
        return FAILED;
    }

    OpenListEntryLazy next = open_list->remove_min();

    current_predecessor_buffer = next.first;
    current_operator = next.second;
    State current_predecessor(current_predecessor_buffer);
    assert(current_operator->is_applicable(current_predecessor));
    current_state = State(current_predecessor, *current_operator);

    SearchNode pred_node = search_space.get_node(current_predecessor);
    current_g = pred_node.get_g() + get_adjusted_cost(*current_operator);
    current_real_g = pred_node.get_real_g() + current_operator->get_cost();

    return IN_PROGRESS;
}

int LazySearch::step() {
    // Invariants:
    // - current_state is the next state for which we want to compute the heuristic.
    // - current_predecessor is a permanent pointer to the predecessor of that state.
    // - current_operator is the operator which leads to current_state from predecessor.
    // - current_g is the g value of the current state according to the cost_type
    // - current_g is the g value of the current state (using real costs)


    SearchNode node = search_space.get_node(current_state);
    cout<<"\nNode that comes from current_state: h = "<<node.get_h()<<" g = "<<node.get_real_g()<<"\n";

    bool reopen = reopen_closed_nodes && (current_g < node.get_g()) && !node.is_dead_end() && !node.is_new();
        if (node.is_new() || reopen) {
        state_var_t *dummy_address = current_predecessor_buffer;
        // HACK! HACK! we do this because SearchNode has no default/copy constructor
        if (dummy_address == NULL) {
            dummy_address = const_cast<state_var_t *>(g_initial_state->get_buffer());
        }

        SearchNode parent_node = search_space.get_node(State(dummy_address));
        cout<<"\nParent node: h "<<parent_node.get_h()<<" g = "<<parent_node.get_real_g()<<" f = "<<parent_node.get_h() + parent_node.get_real_g()<<endl;
        const State perm_state = node.get_state();

        for (int i = 0; i < heuristics.size(); i++) {
            if (current_operator != NULL) {
                heuristics[i]->reach_state(parent_node.get_state(), *current_operator, perm_state);
            }
            heuristics[i]->evaluate(current_state);
        }
        search_progress.inc_evaluated_states();
        search_progress.inc_evaluations(heuristics.size());
        open_list->evaluate(current_g, false);
        if (!open_list->is_dead_end()) {
            // We use the value of the first heuristic, because SearchSpace only
            // supported storing one heuristic value
            int h = heuristics[0]->get_value();
            if (reopen) {
                node.reopen(parent_node, current_operator);
                cout<<"\tNode Reopen h = "<<node.get_h()<<" g = "<<node.get_real_g()<<"\n";
                search_progress.inc_reopened();
            } else if (current_predecessor_buffer == NULL) {
                node.open_initial(h);
                cout<<"\tInitial State h = "<<node.get_h()<<" g = "<<node.get_real_g()<<"\n";
                /*if (two_heuristics) {
                  int h = heuristics[1]->get_value();
                  cout<<"\th = "<<h<<endl;
                  v_f.push_back(h + node.get_real_g());
                  v_g.push_back(node.get_real_g());
                  v_h.push_back(h);
                } else {
                   v_f.push_back(node.get_h() + node.get_real_g());
                   v_g.push_back(node.get_real_g());
                   v_h.push_back(node.get_h());
                }*/


                search_progress.get_initial_h_values();
            } else {
                node.open(h, parent_node, current_operator);
                cout<<"\tNode Removed From the openlist h = "<<node.get_h()<<" g = "<<node.get_real_g()<<"\n";
                /*if (two_heuristics) {
                   int h = heuristics[1]->get_value();
                   cout<<"\th = "<<h<<endl;
                   v_f.push_back(h + node.get_real_g());
                   v_g.push_back(node.get_real_g());
                   v_h.push_back(h);
                } else {
                   cout<<"\th = "<<h<<endl;
                   v_f.push_back(node.get_h() + node.get_real_g());
                   v_g.push_back(node.get_real_g());
                   v_h.push_back(node.get_h());
                }*/
            }
            cout<<"\n";
            node.close();
            if (check_goal_and_set_plan(current_state)) {
               cout<<"\tGOAL NODE h = "<<node.get_h()<<" g = "<<node.get_real_g()<<"\n";
                
               if (two_heuristics) {
                  int h = heuristics[1]->get_value();
                  cout<<"\th = "<<h<<endl;
                  v_f.push_back(h + node.get_real_g());
                  v_g.push_back(node.get_real_g());
                  v_h.push_back(h);

                  map<int, int> g;
                  map<int, vector<int> > mapv_f;
                  for (int i = 0; i < v_g.size(); i++) {
                      int a = v_g.at(i);
                      int k = 1;
                      for (int j = 0; j < v_g.size(); j++) {
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
                  cout<<"Display: "<<endl;
                  vector<int> f_exp;
                  for (map<int,int>::iterator iter = g.begin(); iter != g.end(); iter++) {
                      int level = iter->first;
                      int q = iter->second;
                
                      cout<<"level = "<<level<<endl;
                      vector<int> v;
                      for (int i = 0; i < q; i++) {
                          int f = v_h.at(r) + v_g.at(r);
                          v.push_back(f);
                          f_exp.push_back(f);
                          r++;
                      }
                      for (int i = 0; i < v.size(); i++) {
                          cout<<v.at(i)<<endl;
                      }
                      cout<<"\n\n";
                      mapv_f.insert(pair<int, vector<int> >(level, v));
                  }
                  cout<<"f_exp.size() = "<<f_exp.size()<<endl;
                  map<int, int> dist = getFDistribution(f_exp);
                  cout<<"***************************************************"<<endl;
                  cout<<"f(camada)\t#nodes expanded"<<endl;
                  for (map<int, int>::iterator iter = dist.begin(); iter != dist.end(); iter++) {
                      int f = iter->first;
                      int q = iter->second;
                      cout<<f<<"\t"<<q<<"\n";
                  }
                  cout<<"***************************************************"<<endl;
                  cout<<"\n";
                  cout<<"Dijkstra: Nodes by level:"<<endl;
                  cout<<"totalniveles: "<<mapv_f.size()<<endl;
               

                  string fDist = "/home/marvin/marvin/testss/merge_and_shrink/report/blocks/fdist/probBLOCKS-4-0.pddl";
                  ofstream outFile;
                  outFile.open(fDist.c_str(), ios::out);

                  outFile<<"\t\ttitle\n";
                  outFile<<"\ttotalniveles: "<<mapv_f.size()<<"\n";
                  outFile<<"\tthreshold: 12\n";
                

                  for (map<int, vector<int> >::iterator iter = mapv_f.begin(); iter != mapv_f.end(); iter++) {
                      
                      int g = iter->first;
                      outFile<<"\tg:"<<g<<"\n";         
                      vector<int> v = iter->second;
                      map<int, int> m = getFDistribution(v);

                     
                      outFile<<"\tsize: "<<m.size()<<"\n";
                      //Distribution of f-values print on testss to calculate te prediction.
                      for (map<int, int>:: iterator iter2 = m.begin(); iter2 != m.end(); iter2++) {
                          int f = iter2->first;
                          int q = iter2->second;
                          cout<<"f: "<<f<<" q: "<<q<<endl;
                          outFile<<"\t\tf: "<<f<<"\tq: "<<q<<"\n";

                          //cout<<"\n";
                          //cout<<"fnivel: "<<f<<"\n";
                          //cout<<"nodesGeneratedByLevel: "<<q<<"\n";
                          //cout<<"time0: 1\n";
                          //cout<<"nodesGeneratedToTheLevel: 5\n";
                          //cout<<"\n";

                      }
                      outFile<<"\n\n";
		  }
                  outFile.close();
	       } else {
                   //cout<<"\th = "<<h<<endl;
                   //v_f.push_back(node.get_h() + node.get_real_g());
                   //v_g.push_back(node.get_real_g());
                   //v_h.push_back(node.get_h());
                  
                  map<int, int> g;
                  map<int, vector<int> > mapv_f;
                  for (int i = 0; i < v_g.size(); i++) {
                      int a = v_g.at(i);
                      int k = 1;
                      for (int j = 0; j < v_g.size(); j++) {
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
                  cout<<"Display: "<<endl;
                  vector<int> f_exp;
                  for (map<int,int>::iterator iter = g.begin(); iter != g.end(); iter++) {
                      int level = iter->first;
                      int q = iter->second;
                
                      cout<<"level = "<<level<<endl;
                      vector<int> v;
                      for (int i = 0; i < q; i++) {
                          int f = v_h.at(r) + v_g.at(r);
                          v.push_back(f);
                          f_exp.push_back(f);
                          r++;
                      }
                      for (int i = 0; i < v.size(); i++) {
                          cout<<v.at(i)<<endl;
                      }
                      cout<<"\n\n";
                      mapv_f.insert(pair<int, vector<int> >(level, v));
                  }
                  cout<<"f_exp.size() = "<<f_exp.size()<<endl;
                  map<int, int> dist = getFDistribution(f_exp);
                  cout<<"***************************************************"<<endl;
                  cout<<"f-value(camada)\t#nodes expanded"<<endl;
                  for (map<int, int>::iterator iter = dist.begin(); iter != dist.end(); iter++) {
                      int f = iter->first;
                      int q = iter->second;
                      cout<<f<<"\t"<<q<<"\n";
                  }
                  cout<<"***************************************************"<<endl;
                  cout<<"\n";
                  cout<<"Dijkstra: Nodes by level:"<<endl;
                  cout<<"totalniveles: "<<mapv_f.size()<<endl;
               
                  for (map<int, vector<int> >::iterator iter = mapv_f.begin(); iter != mapv_f.end(); iter++) {
                      
                      int g = iter->first;
                      cout<<"level: "<<g<<endl;         
                      vector<int> v = iter->second;
                      map<int, int> m = getFDistribution(v);
                
                      for (map<int, int>:: iterator iter2 = m.begin(); iter2 != m.end(); iter2++) {
                          int f = iter2->first;
                          int q = iter2->second;
                          cout<<"f: "<<f<<" q: "<<q<<endl;
                          
                          //cout<<"\n";
                          //cout<<"fnivel: "<<f<<"\n";
                          //cout<<"nodesGeneratedByLevel: "<<q<<"\n";
                          //cout<<"time0: 1\n";
                          //cout<<"nodesGeneratedToTheLevel: 5\n";
                          //cout<<"\n";
                      }
		  }
               }

               return SOLVED;
            }
            cout<<"\t\t\tcurrent_g? = "<<current_g<<endl; 
            if (search_progress.check_h_progress(current_g)) {
                cout<<"\n\t\t\tNODE: h = "<<node.get_h()<<" g = "<<node.get_real_g()<<"\n\n\n";
                int h = heuristics[1]->get_value();
                cout<<"\th = "<<h<<endl;
                v_f.push_back(h + node.get_real_g());
                v_g.push_back(node.get_real_g());
                v_h.push_back(h);

                reward_progress();
            }
            generate_successors();
            search_progress.inc_expanded();
        } else {
            node.mark_as_dead_end();
            search_progress.inc_dead_ends();
        }
    }
    return fetch_next_state();
}

map<int, int> LazySearch::getFDistribution(vector<int> v_f_value) {
      map<int, int> m;
      for (int i = 0; i < v_f_value.size(); i++) {
          int a = v_f_value.at(i);
          int k = 1;
          for (int j = 0; j < v_f_value.size(); j++) {
              int b = v_f_value.at(j);
              if (i != j) {
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


void LazySearch::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    open_list->boost_preferred();
}

void LazySearch::statistics() const {
    search_progress.print_statistics();
}

static SearchEngine *_parse(OptionParser &parser) {
    Plugin<OpenList<OpenListEntryLazy > >::register_open_lists();
    parser.add_option<OpenList<OpenListEntryLazy> *>("open");
    parser.add_option<bool>("reopen_closed", false,
                            "reopen closed nodes");
    parser.add_list_option<Heuristic *>(
        "preferred", vector<Heuristic *>(),
        "use preferred operators of these heuristics");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    LazySearch *engine = 0;
    if (!parser.dry_run()) {
        engine = new LazySearch(opts);
        vector<Heuristic *> preferred_list =
            opts.get_list<Heuristic *>("preferred");
        engine->set_pref_operator_heuristics(preferred_list);
    }

    return engine;
}


static SearchEngine *_parse_greedy(OptionParser &parser) {
    parser.add_list_option<ScalarEvaluator *>("evals");
    parser.add_list_option<Heuristic *>(
        "preferred", vector<Heuristic *>(),
        "use preferred operators of these heuristics");
    parser.add_option<bool>("reopen_closed", false,
                            "reopen closed nodes");
    parser.add_option<int>("boost", DEFAULT_LAZY_BOOST,
                           "boost value for preferred operator open lists");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    LazySearch *engine = 0;
    if (!parser.dry_run()) {
        vector<ScalarEvaluator *> evals =
            opts.get_list<ScalarEvaluator *>("evals");
        vector<Heuristic *> preferred_list =
            opts.get_list<Heuristic *>("preferred");
        OpenList<OpenListEntryLazy> *open;
        if ((evals.size() == 1) && preferred_list.empty()) {
            open = new StandardScalarOpenList<OpenListEntryLazy>(evals[0],
                                                                 false);
        } else {
            vector<OpenList<OpenListEntryLazy> *> inner_lists;
            for (int i = 0; i < evals.size(); i++) {
                inner_lists.push_back(
                    new StandardScalarOpenList<OpenListEntryLazy>(evals[i],
                                                                  false));
                if (!preferred_list.empty()) {
                    inner_lists.push_back(
                        new StandardScalarOpenList<OpenListEntryLazy>(evals[i],
                                                                      true));
                }
            }
            open = new AlternationOpenList<OpenListEntryLazy>(
                inner_lists, opts.get<int>("boost"));
        }
        opts.set("open", open);
        engine = new LazySearch(opts);
        engine->set_pref_operator_heuristics(preferred_list);
    }
    return engine;
}

static SearchEngine *_parse_weighted_astar(OptionParser &parser) {
    parser.add_list_option<ScalarEvaluator *>("evals");
    parser.add_list_option<Heuristic *>(
        "preferred", vector<Heuristic *>(),
        "use preferred operators of these heuristics");
    parser.add_option<bool>("reopen_closed", true, "reopen closed nodes");
    parser.add_option<int>("boost", DEFAULT_LAZY_BOOST,
                           "boost value for preferred operator open lists");
    parser.add_option<int>("w", 1, "heuristic weight");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    opts.verify_list_non_empty<ScalarEvaluator *>("evals");

    LazySearch *engine = 0;
    if (!parser.dry_run()) {
        vector<ScalarEvaluator *> evals = opts.get_list<ScalarEvaluator *>("evals");
        vector<Heuristic *> preferred_list =
            opts.get_list<Heuristic *>("preferred");
        vector<OpenList<OpenListEntryLazy> *> inner_lists;
        for (int i = 0; i < evals.size(); i++) {
            GEvaluator *g = new GEvaluator();
            vector<ScalarEvaluator *> sum_evals;
            sum_evals.push_back(g);
            if (opts.get<int>("w") == 1) {
                sum_evals.push_back(evals[i]);
            } else {
                WeightedEvaluator *w = new WeightedEvaluator(
                    evals[i],
                    opts.get<int>("w"));
                sum_evals.push_back(w);
            }
            SumEvaluator *f_eval = new SumEvaluator(sum_evals);

            inner_lists.push_back(
                new StandardScalarOpenList<OpenListEntryLazy>(f_eval, false));

            if (!preferred_list.empty()) {
                inner_lists.push_back(
                    new StandardScalarOpenList<OpenListEntryLazy>(f_eval,
                                                                  true));
            }
        }
        OpenList<OpenListEntryLazy> *open;
        if (inner_lists.size() == 1) {
            open = inner_lists[0];
        } else {
            open = new AlternationOpenList<OpenListEntryLazy>(
                inner_lists, opts.get<int>("boost"));
        }

        opts.set("open", open);

        engine = new LazySearch(opts);
        engine->set_pref_operator_heuristics(preferred_list);
    }
    return engine;
}

static Plugin<SearchEngine> _plugin("lazy", _parse);
static Plugin<SearchEngine> _plugin_greedy("lazy_greedy", _parse_greedy);
static Plugin<SearchEngine> _plugin_weighted_astar("lazy_wastar", _parse_weighted_astar);
