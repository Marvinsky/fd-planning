#include "HUST_search.h"
#include "eager_search.h"

#include "globals.h"
#include "heuristic.h"
#include "option_parser.h"
#include "successor_generator.h"
#include "g_evaluator.h"
#include "sum_evaluator.h"
#include "plugin.h"
#include "Tree.h"
#include "utilities.h"

#include <cassert>
#include <cstdlib>
#include <set>
#include <limits.h>
#include <math.h>
using namespace std;

vector<State> path_states;//for cycle path checking

void print(vector<int> vec_int,bool newline){
  for(unsigned i=0;i<vec_int.size();i++){
      cout<<vec_int.at(i);
      if(i<(vec_int.size()-1)) cout<<",";
  }
  if(newline){
    cout<<endl;
  }
} 

HST lsearch_space(100000,1);

HustSearch::HustSearch(
    const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      do_pathmax(opts.get<bool>("pathmax")),
      use_multi_path_dependence(opts.get<bool>("mpd")),
      open_list(opts.get<OpenList<state_var_t *> *>("open")),
      f_evaluator(opts.get<ScalarEvaluator *>("f_eval")) {
    if (opts.contains("preferred")) {
        preferred_operator_heuristics =
            opts.get_list<Heuristic *>("preferred");
    }
}

void HustSearch::initialize() {
  cout<<"Doing HUST_Search"<<endl;
    //TODO children classes should output which kind of search
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    if (do_pathmax)
        cout << "Using pathmax correction" << endl;
    if (use_multi_path_dependence)
        cout << "Using multi-path dependence (LM-A*)" << endl;
    //verify initial state is not goal state!
    if (test_goal(*g_initial_state)){
    cout<<"solution_found,Total_sampling_time:"<<total_IDA_iter_sampling_timer<<endl;
      cout<<"Initial state is goal!"<<endl;
        exit(0);
    }
    if(early_termination){
      cout<<"using Early Termination"<<endl;
    }
    else{
      cout<<"No Early Termination"<<endl;
    }

    if(global_duplicate_check){
      cout<<"using global_duplicate_check"<<endl;
    }
    else{
      cout<<"global_duplicate_check is off"<<endl;
    }
    if(cycle_duplicate_check){
      cout<<"using cycle_check, so duplicate_check is off!"<<endl;
      global_duplicate_check=false;
    }

    
    total_IDA_iter_sampling_timer=0;
    total_credit_assignment_timer=0;
    total_solving_timer=0;
    nodes_gen_iter=0;
    nodes_gen_prev_iter=0;

    //assert(open_list != NULL);
    //just in case, clean the HST data
    //add one to problem instance counter
    problem_index++;
  

    set<Heuristic *> hset;
    //cout<<"h_set_size before open_list call:"<<hset.size()<<endl;
    open_list->get_involved_heuristics(hset);
    //cout<<"h_set_size after open_list call:"<<hset.size()<<endl;

    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
        estimate_heuristics.push_back(*it);
        search_progress.add_heuristic(*it);
    }
    //cout<<"h_set_size:"<<hset.size();

    // add heuristics that are used for preferred operators (in case they are
    // not also used in the open list)
    hset.insert(preferred_operator_heuristics.begin(),
                preferred_operator_heuristics.end());

    // add heuristics that are used in the f_evaluator. They are usually also
    // used in the open list and hence already be included, but we want to be
    // sure.
    if (f_evaluator) {
      //cout<<"using f_evaluator"<<endl;
        f_evaluator->get_involved_heuristics(hset);
    }

    //int counter=0;
    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
      //cout<<"pushed heurisic "<<counter++<<endl;
        heuristics.push_back(*it);
    }
    cout<<heuristics.size()<<" are active"<<endl;
  
    lsearch_space.update_hset_size(heuristics.size());
    cout<<"updated hset_size to"<<lsearch_space.get_hset_size();
    
    //reset HST just in case
    lsearch_space.reset_HST();
    //Iter index set at 0 because calling step will set it to 1, we are starting a new problem
    lsearch_space.reset_iter_index();

    cout<<"resetting h_capped to false"<<endl;
    h_capped.assign(heuristics.size(),false);//no heuristics capped if we start new problem

    assert(!heuristics.empty());

    //Now we time the node generation and expansion costs
    Timer heur_timings;
    //first time node generation time
	
    //need to know average generation time to know how many nodes is a good idea to check if out of time, so even for MAXTREE
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
      //node_time_adjusted_reval=3.0/node_gen_and_exp_cost;




    cout<<"Initial State:";g_initial_state->dump();cout<<endl;
    if(Current_RIDA_Phase==SAMPLING_PHASE){
      cout<<"INITIALIZING SAMPLING_PHASE"<<endl;
      int h_min=INT_MAX/2;
      double max_TPN=0;
      for (size_t i = 0; i < heuristics.size(); i++){
	  heuristics[i]->evaluate(*g_initial_state);
	  heur_timings.reset();heur_timings.resume();
	  double counter=0;
	  while(heur_timings()<0.1){//in situ measure heuristic evaluation costs
	    counter++;
	    heuristics[i]->evaluate(*g_initial_state);
	  }
	    
	  double TPN=heur_timings.stop()/counter;
	  max_TPN=max(TPN,max_TPN);
	  /*if(TPN<min_TPN){
	    start_heur=i;//we choose the cheapest heuristic to be the "starting one"
	    cout<<"h[,"<<i<<",] name:,";heuristics[i]->print_heur_name()<<" is start_heur"<<endl;
	    }*/
	    heuristics[i]->set_measured_TPN(TPN);
	    //cout<<"h[,"<<i<<",] is:,";heuristics[i]->print_heur_name();cout<<",measured time cost:"<<heuristics[i]->get_measured_TPN()<<endl;
	    cout<<"h[,"<<i<<",measured time cost:"<<heuristics[i]->get_measured_TPN()<<endl;
	    cout<<"\tf["<<i<<"]:"<<heuristics[i]->get_heuristic()<<",h["<<i<<"]:"<<heuristics[i]->get_heuristic()<<"g:"<<0;cout<<endl;
	    h_min=min(h_min,heuristics[i]->get_heuristic());
	}
	node_time_adjusted_reval=5.0/(node_gen_and_exp_cost+max_TPN);
	CAPPING_LIMIT=node_time_adjusted_reval;
	cout<<"node_time_adjusted_reval based on 5 seconds with most expensive heuristic:"<<node_time_adjusted_reval<<"is the new CAPPING_LIMIT"<<endl;
	/*if(node_time_adjusted_reval<10){
	  node_time_adjusted_reval=10;
	}*/
	cout<<"node_time_adjusted_reval based on half second and the most expensive heuristic:"<<node_time_adjusted_reval<<endl;
      lsearch_space.set_current_F_bound(h_min);cout<<"F_bound set to "<<lsearch_space.get_current_F_bound()<<endl;
    }
    else{
      cout<<"INITIALIZING SOLVING PHASE"<<endl;
      int h_max=0;
      for (size_t i = 0; i < heuristics.size(); i++){
	  heuristics[i]->evaluate(*g_initial_state);
	  cout<<"\tf["<<i<<"]:"<<heuristics[i]->get_heuristic()<<",h["<<i<<"]:"<<heuristics[i]->get_heuristic()<<"g:"<<0;cout<<endl;
	  h_max=max(h_max,heuristics[i]->get_heuristic());
      }
      lsearch_space.set_current_F_bound(h_max);cout<<"F_bound set to "<<lsearch_space.get_current_F_bound()<<endl;
    }
      //state_var_t *state_data=const_cast<state_var_t *>(g_initial_state->get_buffer());
    //const state_var_t *state_data=g_initial_state->get_buffer();
    //exit(0);
    search_progress.inc_evaluated_states();
    search_progress.inc_evaluations(heuristics.size());

    search_progress.get_initial_h_values();
    if (f_evaluator) {
	f_evaluator->evaluate(0, false);
	search_progress.report_f_value(f_evaluator->get_value());
    }
    search_progress.check_h_progress(0);
}
int HustSearch::evaluate_MAX_node(const State* S,int depth){
  int h_max=0;
  int H=0;
  static bool first_call=true;

  //string state_id;///FOR DEBUGGING ONLY, REMOVE ME OTHERWISE!
  //S->get_state_id_string(&state_id);
  //string interesting_state("1,1,0,1,1,1,1,0,1,0,1,10,1,10,4,6,4,2,6,0,10,");///FOR DEBUGGING ONLY, REMOVE ME OTHERWISE!
  //cout<<"calling evaluate_MAX_node"<<endl;

  if(first_call){
    cout<<"input heuristic set size:"<<heuristics.size()<<endl;
  }
  for (size_t i = 0; i < heuristics.size(); i++){
      heuristics[i]->evaluate(*S);
      //if(state_id==interesting_state){
	//cout<<",f["<<i<<"]:"<<heuristics[i]->get_heuristic()+depth<<",h["<<i<<"]:"<<heuristics[i]->get_heuristic()<<",g:"<<depth<<endl;
      //}
      H=heuristics[i]->get_heuristic();
      if(first_call){
	cout<<"\tInitial h("<<i<<"):"<<H<<endl;
      }
      if(early_termination){//return first F-value above F-bound
	if(H+depth>lsearch_space.get_current_F_bound()){
	  h_max=max(h_max,H);
	  break;
	}
      }
      h_max=max(h_max,H);
      //cout<<"\tpartial h_max for current node:"<<h_max<<endl;
  }
  //cout<<"final h_max for current node:"<<h_max<<endl;
  if(first_call){
    cout<<"returning h_max:"<<h_max<<endl;
    first_call=false;
  }
    
  //cout<<"returning h_max:"<<h_max<<endl;
  return h_max;
}

int HustSearch::evaluate_HUST_node(const State* S,int depth,int children){
  int h_min=INT_MAX/2;
  int H=0;
  boost::dynamic_bitset<> h_index(heuristics.size());
  static bool first_reported=false;
  vector<bool> first_culling_added;//needed temporary marker because no need to update culling trackers if all heuristics cull current node

  //cout<<"h_capped:"<<h_capped.count()<<endl;
  for (size_t i = 0; i < heuristics.size(); i++){
    if(h_capped.at(i)||lsearch_space.get_earliest_depth_h_culled(i)<depth){//path already culled for database
      H=INT_MAX/2;//INT_MAX on its own leads to bug as we add depth to it, resulting in the counter restarting
      if(h_capped.at(i)){
	if(!first_reported){
	  cout<<"h("<<i<<") is capped, so H="<<H<<endl;
	  first_reported=true;
	}
      }
      //cout<<"\tH["<<i<<"]:"<<H<<",depth:"<<depth<<",F:"<<depth+H<<endl;fflush(stdout);
      //
      /*if(h_capped.at(i)){
	  cout<<"H("<<i<<") is capped, so H="<<H<<endl;
      }
      else{
	  cout<<"H("<<i<<"was culled @ depth:"<<lsearch_space.get_earliest_depth_h_culled(i)<<endl;
      }*/

    }
    else{
      heuristics[i]->evaluate(*S);
      H=heuristics[i]->get_heuristic();
      //cout<<"\tH["<<i<<"]:"<<H<<",depth:"<<depth<<",F:"<<depth+H<<endl;fflush(stdout);
    }
    //Now update CC index with either culling (1) or generating (0) bit
    if((H+depth)>lsearch_space.get_current_F_bound()){//node culled is marked as 1 in binary switch list
      //cout<<"\t culling heuristic, adding one to h_index"<<endl;
      h_min=min(h_min,H);
      h_index.set(i);
      if(lsearch_space.checking_consistency()){
	if(lsearch_space.get_earliest_depth_h_culled(i)==INT_MAX/2){//first culled in path
	  //cout<<"\th("<<i<<"),first culling at:"<<depth<<endl;
	  first_culling_added.push_back(true);
	 //h_culling_index+=pow(3,database);//value is one for first culling
	}
	else{
	  first_culling_added.push_back(false);
	  //h_culling_index+=pow(3,database)*2;//value is two for previous culling
	}
      }
    }
    else if(lsearch_space.checking_consistency()){//h_min is not updated if path would have been culled by MAXTREE
      first_culling_added.push_back(false);
      if(lsearch_space.get_earliest_depth_h_culled(i)<depth){//h_min is not updated as path would have been culled by MAXTREE
	cout<<"\tcurrent_depth:"<<depth<<",inconsistency at h:"<<i<<",first culling at:"<<lsearch_space.get_earliest_depth_h_culled(i)<<endl;exit(0);
	h_index.set(i);
      }
      else{//h_min is updated as there is no inconsistency
	if(h_capped.at(i)){
	  cout<<"If the heuristic is capped, it must cull the node, check the code!"<<endl;exit(0);
	}
	  h_min=min(h_min,H);
      }
    }
    else{//update h_min
      h_min=min(h_min,H);
    }
  }
  if(children==0){//no CC counter to update as node has no children
    return h_min;
  }

  //Only update CCs if at least one heuristic is expanding, this way we reduce CC updates.  We add all children nodes for any "internal" CC
  
  if(h_index.count()!=heuristics.size()){//all heuristics culling so no child generated, so nothing to add to counters
    if(depth==0){//first addition, add root to count
      cout<<"\tdepth=0"<<endl;
      lsearch_space.add_to_counter(&h_index,children+1);
      cout<<"\tF_bound:"<<lsearch_space.get_current_F_bound()<<",h_min:"<<h_min<<"heuristic combination:"<<h_index<<",depth:"<<depth<<"adding to counter root node+children nodes:"<<children+1<<endl;fflush(stdout);
    }
    else{//second+ addition,nodes are prunned by gfth_check
      lsearch_space.add_to_counter(&h_index,children);
    }
      //cout<<"\tmin_f:"<<gcmd_line.culling<<"h_min:"<<h_min<<",heuristic combination:"<<h_index<<",depth:"<<depth<<",children nodes added to counter:"<<gnum_A-1<<endl;
  //If checking consistency, update relevant culling depth trackers

    if(lsearch_space.checking_consistency()){
      for (int i=0;i<first_culling_added.size(); i++ ){
	if(first_culling_added.at(i)){//heuristic culled for first time
	  //cout<<"heuristic "<<i<<" culled at depth"<<depth<<endl;
	  lsearch_space.set_earliest_depth_h_culled(i,depth);
	}
      }
    }
  }
  else{
    //cout<<"all heuristics culling so no update to CC for current node"<<endl;
  }
  //cout<<"final h_min for current node:"<<h_min<<endl;

  return h_min;
}



void HustSearch::statistics() const {
    search_progress.print_statistics();
    search_space.statistics();
}
int HustSearch::step() {
  //search_progress.reset();
  int status=IN_PROGRESS;
  Timer IDA_iter_sampling_timer;
  Timer IDA_iter_solving_timer;
  Timer credit_assignment_timer;
  if(Current_RIDA_Phase==SAMPLING_PHASE){
    IDA_iter_sampling_timer.reset();
    //Do the iteration
    status=step_iter_sampling();
    total_IDA_iter_sampling_timer+=IDA_iter_sampling_timer.stop();
    cout<<"Iter:"<<lsearch_space.get_iter_index()<<",F_bound:"<<lsearch_space.get_F_bound()<<",search time:"<<IDA_iter_sampling_timer<<",overall sampling time:"<<total_IDA_iter_sampling_timer<<endl;
    if(status==SOLVED){
      cout<<"problem solved while sampling, Search time:"<<total_IDA_iter_sampling_timer<<endl;
      return status;

    }
   
    //caluclate uncapped heuristics
    credit_assignment_timer.reset();
    lsearch_space.calculate_heuristics_to_degree(Degree);
    //calculate all heuristics
    lsearch_space.print_final_size_counters(false,Degree);
    total_credit_assignment_timer+=credit_assignment_timer.stop();
    total_IDA_iter_sampling_timer+=credit_assignment_timer.stop();
    cout<<"Iter:"<<lsearch_space.get_iter_index()<<",F_bound:"<<lsearch_space.get_F_bound()<<",assignment time:"<<credit_assignment_timer<<",overall search time:"<<total_credit_assignment_timer<<endl;
    cout<<"heuristics capped:";
    bool one_h_uncapped=false;
    for(unsigned i=0;i<h_capped.size();i++){
      cout<<h_capped.at(i)<<",";
	if(!h_capped.at(i)){
	  one_h_uncapped=true;
	}
    }
    cout<<endl;
    if(!one_h_uncapped){
      cout<<"No more heuristics, all capped"<<endl;
      //need to calculate latest gen_to_eval_ratio
      gen_to_eval_ratio=double(search_progress.get_generated())/double(search_progress.get_evaluated_states());
      cout<<"gen_to_eval_ratio for sampling:"<<gen_to_eval_ratio<<endl;
      lsearch_space.select_best_heuristics(Degree,heuristics);
      //dropping unnecesary heuristics
      boost::dynamic_bitset<> selec_heur(lsearch_space.get_best_h_comb());
      unsigned remaining_heuristics=0;
      unsigned offset=0;//remove() decreases the size of heuristic vector by one
      size_t original_hset_size=heuristics.size();
      for (size_t i = 0; i < original_hset_size; i++){
	if(selec_heur.test(i)){
	  cout<<"keeping heuristic "<<i<<endl;
	  remaining_heuristics++;
	}
	else{
	  cout<<"removing heuristic"<<i<<" from set"<<endl;
	  heuristics.erase(heuristics.begin()+i-offset);
	  offset++;
	}
      }
      lsearch_space.update_hset_size(remaining_heuristics);
      Current_RIDA_Phase=SOLVING_PHASE;
      nodes_gen_prev_iter=0;//updating so we calculate the solving HBF after two iterations
      return IN_PROGRESS;
    }
  }
  else{
    //boost::dynamic_bitset<> selec_heur(heuristics.size(),1);
    //reinitialize node gen counters for solving phase
    cout<<"Now run regular FD eager search(A*) with best combination,Total_sampling_time:"<<total_IDA_iter_sampling_timer<<endl;
    nodes_gen_iter=0;
    IDA_iter_solving_timer.reset();
    status=step_iter_solving(lsearch_space.get_best_h_comb());
    double iter_timer=IDA_iter_solving_timer.stop();
    total_solving_timer+=iter_timer;
    cout<<"Iter:"<<lsearch_space.get_iter_index()<<",F_bound:"<<lsearch_space.get_F_bound()<<",iteration time:"<<iter_timer<<",overall search time:"<<total_solving_timer<<endl;

    if(draw_graph){
      lsearch_space.print_Dot_Tree();
    }
  }
  return status;
}
int HustSearch::call_astar_search(boost::dynamic_bitset<> selec_heur) {
  int i;
  for (i = 0; i < argc_copy; ++i) {
    puts(argv_copy[i]);
    //cout<<"i:"<<i<<","<<argv_copy[i]<<endl;
  }
  //string str="astar(sum([gapdb(mutation_probability=0.01,pdb_max_size=50000,num_episodes=10),gapdb(mutation_probability=0.02,pdb_max_size=50000,num_episodes=10),gapdb(mutation_probability=0.03,pdb_max_size=50000,num_episodes=10),gapdb(mutation_probability=0.04,pdb_max_size=50000,num_episodes=10),lmcut(),gapdb(mutation_probability=0.05,pdb_max_size=50000,num_episodes=10)]))";
  //string str="astar(lmcut())";
  string str="astar(blind())";
  char * cstr = new char [str.length()+1];
  std::strcpy (cstr, str.c_str());
  argv_copy[2] = strdup(cstr);
  cout<<"argv_copy[2]=";puts(argv_copy[2]);
  
  str="SOLVING";
  std::strcpy (cstr, str.c_str());
  argv_copy[8] = strdup(cstr);
  cout<<"argv_copy[8]=";puts(argv_copy[8]);
  //std::string argv_str(argv_copy[1]);

  //cout<<"argv_str:"<<argv_str<<endl;fflush(stdout);
  //unsigned pos1 = argv_str.find("HUST_IDA");     
  //cout<<"pos1:"<<pos1<<endl;
  //unsigned pos2 = argv_str.find("(");     
  //cout<<"pos2:"<<pos2<<endl;

  //argv_str.replace(pos1,pos2,"astar(");
  //cout<<"new argv_str:"<<argv_str<<endl;;
  //argv_copy[1]=argv_str.c_str();
  //std::strcpy (argv_copy[1], argv_str.c_str());
  //cout<<"new argv_copy[1]:"<<argv_copy[1]<<endl;
 // std::string str3 = str.substr (pos);     // get from "live" to the end
 

  SearchEngine *engine = 0;
        
  //read_everything(cin);
  //OptionParser::parse_cmd_line(argc_copy, argv_copy, true);
  engine = OptionParser::parse_cmd_line(argc_copy, argv_copy, false);
  //selec_heur.set();
  cout<<"select_heur:"<<selec_heur<<endl;
      
  /*size_t original_hset_size=heuristics.size();
  unsigned offset=0;//remove() decreases the size of heuristic vector by one
  for (size_t i = 0; i < original_hset_size; i++){
    if(selec_heur.test(i)){
      cout<<"keeping heuristic "<<i<<endl;
    }
    else{
      cout<<"removing heuristic"<<i<<" from set"<<endl;
      heuristics.erase(heuristics.begin()+i-offset);
      offset++;
    }
  }*/
    
  Timer astar_timer;
  engine->search(heuristics);
  cout<<"Solution found, Search time:"<<astar_timer.stop()<<endl;
  delete engine;
  exit(0);
  /*
    parser.add_option<ScalarEvaluator *>("eval");
    parser.add_option<bool>("pathmax", false,
                            "use pathmax correction");
    parser.add_option<bool>("mpd", false,
                            "use multi-path dependence (LM-A*)");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    if (!parser.dry_run()) {
        GEvaluator *g = new GEvaluator();
        vector<ScalarEvaluator *> sum_evals;
        sum_evals.push_back(g);
        ScalarEvaluator *eval = opts.get<ScalarEvaluator *>("eval");
        sum_evals.push_back(eval);
        ScalarEvaluator *f_eval = new SumEvaluator(sum_evals);

        // use eval for tiebreaking
        std::vector<ScalarEvaluator *> evals;
        evals.push_back(f_eval);
        evals.push_back(eval);
        OpenList<state_var_t *> *open = \
            new TieBreakingOpenList<state_var_t *>(evals, false, false);

        opts.set("open", open);
        opts.set("f_eval", f_eval);
        opts.set("reopen_closed", true);
        engine = new EagerSearch(opts);
    }
    EagerSearch *engine = 0;*/
    exit(0);
    return 0;
}
int HustSearch::step_iter_solving(boost::dynamic_bitset<> selec_heur) {
  Current_RIDA_Phase=SOLVING_PHASE;
  //call_astar_search(selec_heur);
  //exit(0);

  int depth=0;
  const Operator* next_op;
  const State* current_state=g_initial_state;
  const State* succ_state;
  int next_F_bound=INT_MAX/2;
  int status=IN_PROGRESS;
  bool Exist_Ancestor_F_diff_one=false;
  long nodes_gen_F_diff_one=0;
  long nodes_gen_F_diff_two=0;
  int F_diff_one_depth=-1;
      
  map<int,map<int,int> >culled_nodes;

  nodes_gen_iter=0;
  
  
  lsearch_space.inc_iter_index();
  cout<<"Initial F:"<<lsearch_space.get_current_F_bound()<<endl;
  lsearch_space.reset_HST();
  search_progress.reset_for_new_iter();
  cout<<"subset size"<<selec_heur.size()<<",selected subset:"<<selec_heur<<endl;
  

  //g_successor_generator->generate_applicable_ops(*g_initial_state, applicable_ops);
  int h_max=evaluate_MAX_node(current_state,depth);
  const State* S=new State(*g_initial_state);
  if(global_duplicate_check){//need to update the initial state as visited in this iter
    lsearch_space.check_duplicate(current_state,NULL);
  }
  else if(cycle_duplicate_check){//need to update the initial state as visited in this iter
    cout<<"initial path_states size:"<<path_states.size()<<endl;
    if(path_states.size()>0){
      cout<<"check the code(culling and backtracking), cycle path is not empty at beginning of new iteration!"<<endl;
      exit(0);
    }
    //path_states.clear();
    bool visited_state=check_cyclic_path(current_state);
    if(!visited_state) cout<<"initial state was added to path_states for cyclic path check"<<endl;
  }
  lsearch_space.add_to_HST(S,NULL,h_max,0);
  cout<<"Added Root State"<<endl;lsearch_space.print_current_state();fflush(stdout);
  cout<<"With applicable operators"<<endl;
  lsearch_space.print_list_children();
  nodes_gen_iter++;
  search_progress.inc_generated();
  node_time_adjusted_reval=1000;
  
  while(!lsearch_space.empty()){//keep generating and backtracking until no more nodes
    if(search_progress.get_generated()%node_time_adjusted_reval==0){//check memory is still within bounds
      if(search_progress.get_generated()%10000000==0){
	cout<<"nodes_generated:"<<search_progress.get_generated()<<",g_timer:"<<g_timer()<<endl;
      }
	      if(memory_limit<get_peak_memory_in_kb()||g_timer()>time_limit){
		cout<<"Exiting, total hashed nodes:"<<search_space.size()<<" and the current limit is:"<<100000000<<endl;
		cout<<"Exiting, memory limit is:"<<memory_limit<<" and current memory is:"<<get_peak_memory_in_kb()<<endl;
		cout<<"Exiting, time limit is:"<<time_limit<<" and current time is:"<<g_timer()<<endl;
		exit(0);
	      }
	      if(g_timer()>200&&lsearch_space.get_iter_index()>2){
		cout<<"just running to find out Brute Force Branching Factor,Total time:,"<<g_timer()<<endl;
		exit(0);
	      }
    }

    if(lsearch_space.to_be_expanded()){//need to generate next child
      next_op=lsearch_space.get_next_op();//pops back last operator from list
      //cout<<"next op:";next_op->dump();cout<<endl;
  //get next state
      lsearch_space.get_current_state(current_state);
      //cout<<"Fetched state:"<<endl;current_state->dump();cout<<"at depth:"<<lsearch_space.get_last_depth();
      //cout<<"next state:"<<endl;
      //succ_state.gen_next_state(*current_state, *next_op);
      succ_state=new State(*current_state, *next_op);
      if(global_duplicate_check){
	if(lsearch_space.check_duplicate(current_state,next_op)){
	  delete succ_state;
	  continue;//going to next state, this one is duplicated
	}
      }
      else if(cycle_duplicate_check){//need to update the initial state as visited in this iter
	bool visited_state=check_cyclic_path(succ_state);
	if(visited_state){
	  //cout<<"state was duplicated,size of path_states:"<<path_states.size()<<endl;
	  delete succ_state;
	  continue;//state already visited in this path
	}
      }
      //succ_state->dump();
      search_progress.inc_generated();
      nodes_gen_iter++;
 
      //Check if we found solution!
      if (check_goal_and_set_plan(*succ_state)){
	if(status!=SOLVED){//fist solution found, there may be others with same depth
	  cout<<"copying solution path"<<endl;
	  lsearch_space.copy_solution_path(next_op);
	}
	cout << "\tsolution found @:"<<lsearch_space.get_lspace_end()<<endl;
	cout<<"\t(all iters) States generated until first solution was found:"<<search_progress.get_generated()<<endl;
	status=SOLVED;
	exit(0);
	//return status;
      }
      //get depth of new state, determined by previous depth and cost of the current op
      depth=lsearch_space.get_last_depth()+next_op->get_cost();
      //cout<<"Current state depth:"<<depth<<endl;
    }
    else{//node must be fully expanded so we backtrack until no more nodes can be expanded or HST is empty
      while((!lsearch_space.to_be_expanded())){
	//cout<<"backtracking,number of nodes in HST:"<<lsearch_space.get_lspace_end()<<endl;
	lsearch_space.backtrack();
	if(cycle_duplicate_check){//need to backtrack the list of states visited on this path as well
	  path_states.pop_back();
	  //cout<<"path_states backtracked, current_size:"<<path_states.size()<<endl;
	}
	if(lsearch_space.get_last_depth()<F_diff_one_depth){//Went past the ancestor with F_diff=1
	  F_diff_one_depth=-1;
	  Exist_Ancestor_F_diff_one=false;
	}
	if(lsearch_space.empty()){
	  lsearch_space.set_current_F_bound(next_F_bound);
	  //cout<<"Total inverse operator prunned"<<gfth_prunned<<endl;
	  cout<<endl<<"\tnext_F_bound:"<<next_F_bound;
	  cout<<"\tStates generated:"<<nodes_gen_iter<<endl;
	  if(nodes_gen_prev_iter!=0){
	    //cout<<"HBF["<<lsearch_space.get_iter_index()<<"]:"<<double(nodes_gen_iter)/double(nodes_gen_prev_iter)<<endl;
	    cout<<"F_boundary:,"<<lsearch_space.get_current_F_bound()<<",HBF["<<lsearch_space.get_iter_index()<<"]:,"<<double(nodes_gen_iter)/double(nodes_gen_prev_iter)<<endl;
	  }
	  nodes_gen_prev_iter=nodes_gen_iter;
	  search_progress.print_statistics();
	  if(status==SOLVED){
	    lsearch_space.print_solution_path();
	  }
	  for(map<int,map<int,int> >::iterator it = culled_nodes.begin(); it != culled_nodes.end(); it++) {
	    for(map<int,int>::iterator it2 = (it->second).begin(); it2 != (it->second).end(); it2++){
	      cout<<"culled_nodes[,"<<it->first<<",][,"<<it2->first<<",]:,"<<it2->second<<endl;
	    }
	  }
	  cout<<"current F_bound,"<<lsearch_space.get_current_F_bound()<<",nodes_gen_F_diff_one,"<<nodes_gen_F_diff_one<<",nodes_gen_F_diff_two,"<<nodes_gen_F_diff_two<<endl;
      
	  /*if(lsearch_space.get_current_F_bound()<27){
	    lsearch_space.print_Dot_Tree();
	  }
	  else{
	    exit(0);
	  }*/
	  return status;//no more nodes to expand
	}
      }
      continue;
    }
    //Now we evaluate the current state to decide if we add it to the tree
    
    //Now we evaluate and add corresponding search progress counters
    h_max=evaluate_MAX_node(succ_state,depth);
    search_progress.inc_evaluated_states();
    search_progress.inc_evaluations(heuristics.size());
    if((!Exist_Ancestor_F_diff_one)&&
	((h_max+depth)-(lsearch_space.get_last_H()+lsearch_space.get_last_depth()))==1){
      Exist_Ancestor_F_diff_one=true;
      F_diff_one_depth=depth;
    }
    //Now divide nodes between those nodes who have all ancestors with F_boundary of 2 vs those nodes who were expanded due to a F_difference of 1 unit
    if(Exist_Ancestor_F_diff_one==true){
      nodes_gen_F_diff_one++;
    }
    else{
      nodes_gen_F_diff_two++;
    }
    //Now we decide whether to add the node or not
    //cout<<"H:"<<h_max<<",depth:"<<depth<<",F:"<<h_max+depth<<",current_F_bound:"<<lsearch_space.get_current_F_bound();
    if((h_max+depth)<=lsearch_space.get_current_F_bound()){//expand nodes not culled by any of the input heuristics
      lsearch_space.add_to_HST(succ_state,next_op,h_max,depth);
      //cout<<"Added State"<<endl;lsearch_space.print_current_state();fflush(stdout);
      //cout<<"With applicable operators"<<endl;lsearch_space.print_list_children();
      //add to expansion counter because all added states will be expanded,assuming we are finishing iterations
      search_progress.inc_expanded();
    }
    else{//need to to list of culled nodes even if they were not generated
      //cout<<"Culled State"<<endl;lsearch_space.print_current_state();fflush(stdout);
      culled_nodes[lsearch_space.get_current_F_bound()][h_max+depth-lsearch_space.get_current_F_bound()]++;
      if(F_diff_one_depth==depth){//the node with the F_diff of one was culled, so need to turn off Exist_ancestor...
	Exist_Ancestor_F_diff_one=false;
	F_diff_one_depth=-1;
      }
      if(h_max==INT_MAX/2){
	search_progress.inc_dead_ends();
	lsearch_space.add_dead_end_nodes();
      }
      else{
	lsearch_space.add_nodes_culled(depth,h_max,next_op,succ_state);//adding to culled_list
      }
      //And updating F_bound if necessary,i.e. found a smaller F-bound where a solution may be found
      if((h_max+depth)<next_F_bound){
	next_F_bound=h_max+depth;
	cout<<"Leaf State with new F_bound:";succ_state->dump();
	cout<<endl<<"\tUpdated next_min_f from:"<<lsearch_space.get_current_F_bound()<<"to "<<next_F_bound<<",h_max:"<<h_max<<",depth:"<<depth<<endl;
      }
      if(cycle_duplicate_check){//need to backtrack the list of states visited on this path as well
	path_states.pop_back();
	//cout<<"path_states backtracked for culled_node, current_size:"<<path_states.size()<<endl;
      }
      delete succ_state;//state will not be added so remove it from memory
    }
  }
  cout<<"\tStates generated:"<<nodes_gen_iter<<endl;
  
  return status;
}
int HustSearch::step_iter_sampling() {
  int depth=0;
  const Operator* next_op;
  const State* current_state;
  const State* succ_state;
  int next_F_bound=INT_MAX/2;
  int status=IN_PROGRESS;
  nodes_gen_iter=0;
  //State succ_state(*g_initial_state);

  lsearch_space.inc_iter_index();
  cout<<"Initial F:"<<lsearch_space.get_current_F_bound()<<endl;
  //reset HST between iterations
  lsearch_space.reset_HST();
  //also reset the relevan search progress markers
  search_progress.reset_for_new_iter();
  
  
  //first get applicable operators
  vector<const Operator *> applicable_ops;
  //vector<const Operator> children_to_generate;
  //calculate initial heuristic value
  g_successor_generator->generate_applicable_ops(*g_initial_state, applicable_ops);
  /*for (int i = 0; i < applicable_ops.size(); i++) {
        const Operator *op = applicable_ops[i];
	children_to_generate.push_back(*op);
  }*/
  int h_min=evaluate_HUST_node(g_initial_state,depth,applicable_ops.size());
  //Now add to HST
  //lsearch_space.add_to_HST(g_initial_state,-1,-1,h_max,0,0);
  //const Operator *dummy_op;
  const State* S=new State(*g_initial_state);
  if(global_duplicate_check){
    lsearch_space.check_duplicate(S,NULL);
  }
  lsearch_space.add_to_HST(S,NULL,h_min,0);
  search_progress.inc_generated();
  nodes_gen_iter++;
  cout<<"Added Root State"<<endl;lsearch_space.print_current_state();fflush(stdout);
  cout<<"With applicable operators:,"<<applicable_ops.size()<<endl;
  lsearch_space.print_list_children();
  while(!lsearch_space.empty()){//keep generating and backtracking until no more nodes
    if(lsearch_space.to_be_expanded()){//need to generate next child
      next_op=lsearch_space.get_next_op();//pops back last operator from list
      //cout<<"next op:";next_op->dump();cout<<endl;
  //get next state
      lsearch_space.get_current_state(current_state);
      //cout<<"Fetched state:"<<endl;current_state->dump();cout<<"at depth:"<<lsearch_space.get_last_depth()<<endl;
      //succ_state.gen_next_state(*current_state, *next_op);
      succ_state=new State(*current_state, *next_op);
      //cout<<"next state:"<<endl;succ_state->dump();
      search_progress.inc_generated();
      nodes_gen_iter++;
    
      /*if(lsearch_space.checking_consistency()){//just for debugging purposes
	cout<<"remove me if running for comparison purposes, evaluating heuristic value twice for the same node!"<<endl;
	depth=lsearch_space.get_last_depth()+next_op->get_cost();
	applicable_ops.clear();
	int h=evaluate_HUST_node(succ_state,depth,applicable_ops.size());
	if(abs(h-lsearch_space.get_last_H())>1){//inconsistent heuristic,assuming operator cost=1
	  cout<<"inconsistent heuristic!"<<endl;
	  cout<<"current_state h:"<<lsearch_space.get_last_H();current_state->inline_dump();
	  h=evaluate_HUST_node(current_state,depth,applicable_ops.size());
	  cout<<"evaluated_h for current_state:"<<h<<endl;
	  cout<<"generating op:";next_op->dump();cout<<endl;
	
	  h=evaluate_HUST_node(succ_state,depth,applicable_ops.size());
	  cout<<"new_state h:"<<h;succ_state->inline_dump();cout<<endl;
	  exit(0);
	}
      }*/

      if(global_duplicate_check){
	if(lsearch_space.check_duplicate(succ_state,next_op)){
	  continue;//going to next state, this one is duplicated
	}
      }
      //succ_state->dump();
    
      //Check if we found solution!
      if (test_goal(*succ_state)){
	cout<<"solution_found,Total_sampling_time:"<<total_IDA_iter_sampling_timer<<endl;
	cout << "\tsolution found @:"<<lsearch_space.get_lspace_end()<<endl;fflush(stdout);
	cout<<"\t(all iters)States generated until first solution was found:"<<search_progress.get_generated()<<endl;fflush(stdout);
	if(status!=SOLVED){//fist solution found, there may be others with same depth
	  cout<<"copying solution path"<<endl;
	  lsearch_space.copy_solution_path(next_op);
	}
	status=SOLVED;
	return status;
      }
      //get depth of new state, determined by previous depth and cost of the current op
      depth=lsearch_space.get_last_depth()+next_op->get_cost();
      //cout<<"Current state depth:"<<depth<<endl;
    }
    else{//node must be fully expanded so we backtrack until no more nodes can be expanded or HST is empty
      while((!lsearch_space.to_be_expanded())){
	//cout<<"backtracking,number of nodes in HST:"<<lsearch_space.get_lspace_end()<<endl;fflush(stdout);
	lsearch_space.backtrack();
      
	if(lsearch_space.empty()){
	  lsearch_space.set_current_F_bound(next_F_bound);
	  //cout<<"Total inverse operator prunned"<<gfth_prunned<<endl;
	  cout<<endl<<"\tnext_F_bound:"<<next_F_bound;
	  cout<<"\tStates generated:"<<nodes_gen_iter<<endl;
	  if(nodes_gen_prev_iter!=0){
	    cout<<"F_boundary:,"<<lsearch_space.get_current_F_bound()<<",HUST_HBF["<<lsearch_space.get_iter_index()<<"]:,"<<double(nodes_gen_iter)/double(nodes_gen_prev_iter)<<endl;
	  }
	  nodes_gen_prev_iter=nodes_gen_iter;
	  search_progress.print_statistics();
	  if(status==SOLVED){
	    lsearch_space.print_solution_path();
	  }
	  return status;//no more nodes to expand
	}
	//need to populate current_state in case it is not going to be expanded
	//lsearch_space.get_current_state(current_state);
	//succ_state=new State(*current_state);
      }
      continue;
      }
    //Now we evaluate the current state to decide if we add it to the tree
    
    //Need to know how many applicable ops because we will add this amount to relevant CC
    applicable_ops.clear();
    g_successor_generator->generate_applicable_ops(*succ_state, applicable_ops);
    //cout<<"succ_state generating_op:";next_op->dump();fflush(stdout);
    //cout<<"succ_state:";succ_state->dump();fflush(stdout);
    //cout<<"succ_state has "<<applicable_ops.size()<<" operators"<<endl;fflush(stdout);
    //Now we evaluate and add corresponding search progress counters
    h_min=evaluate_HUST_node(succ_state,depth,applicable_ops.size());
    search_progress.inc_evaluated_states();
    search_progress.inc_evaluations(heuristics.size());
    //Now we decide whether to add the node or not
    //cout<<"H:"<<h_min<<",depth:"<<depth<<",F:"<<h_min+depth<<",current_F_bound:"<<lsearch_space.get_current_F_bound()<<endl;
    if((h_min+depth)<=lsearch_space.get_current_F_bound()){//expand nodes not culled by any of the input heuristics
      //cout<<"added succ_state:";fflush(stdout);succ_state->inline_dump();cout<<"h:"<<h_min<<",depth:"<<depth<<",F:"<<depth+h_min<<endl;fflush(stdout);
      lsearch_space.add_to_HST(succ_state,next_op,h_min,depth);
      //cout<<"Added State"<<endl;lsearch_space.print_current_state();fflush(stdout);
      //cout<<"With applicable operators"<<endl;lsearch_space.print_list_children();
      //add to expansion counter because all added states will be expanded,assuming we are finishing iterations
      search_progress.inc_expanded();
    }
    else{//need to to list of culled nodes even if they were not generated
      //cout<<"cull succ_state:";succ_state->inline_dump();cout<<"h:"<<h_min<<",depth:"<<depth<<",F:"<<depth+h_min<<endl;fflush(stdout);
      if(heuristics[0]->is_dead_end()){//this must be a dead end state
	search_progress.inc_dead_ends();
	lsearch_space.add_dead_end_nodes();
      }
      /*else{//we are arriving here with dead_end!! check what is going on
	lsearch_space.add_nodes_culled(depth,h_min,next_op,succ_state);//adding to culled_list
      }*/
      //And updating F_bound if necessary,i.e. found a smaller F-bound where a solution may be found
      if((h_min+depth)<next_F_bound){
	next_F_bound=h_min+depth;
	cout<<endl<<"\tUpdated next_min_f from:"<<lsearch_space.get_current_F_bound()<<" to:"<<next_F_bound<<endl;
      }
      //cout<<"before delete succ_state"<<endl;fflush(stdout);
      delete succ_state;//state will not be added so remove it from memory
      //cout<<"after delete succ_state"<<endl;fflush(stdout);
    }
  }
  cout<<"\tStates generated:"<<nodes_gen_iter<<endl;fflush(stdout);
  lsearch_space.print_solution_path();fflush(stdout);
  
  return status;
}

int HustSearch::step_old() {
      pair<SearchNode, bool> n = fetch_next_node();
      if (!n.second) {
	  return FAILED;
      }
      SearchNode node = n.first;

      State s = node.get_state();
      if (test_goal(s)){
	cout<<"solution_found,Total_sampling_time:"<<total_IDA_iter_sampling_timer<<endl;
	return SOLVED;
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
	      for (size_t i = 0; i < heuristics.size(); i++)
		  h_is_dirty = h_is_dirty || heuristics[i]->reach_state(
		      s, *op, succ_node.get_state());
	      if (h_is_dirty && use_multi_path_dependence)
		  succ_node.set_h_dirty();
	  }

	  static long evaluator_counter=0;
	  if (succ_node.is_new()) {
	      // We have not seen this state before.
	      // Evaluate and create a new node.
	      cout<<"call#:"<<evaluator_counter<<",operator:";op->dump();
	      //cout<<",state:";succ_state.dump();cout<<endl;
		  
	      evaluator_counter++;
	      int succ_h=0;
	      for (size_t i = 0; i < heuristics.size(); i++){
		  heuristics[i]->evaluate(succ_state);
		  cout<<"h("<<i<<"):";cout<<"\tf:"<<node.get_g()+1+heuristics[i]->get_heuristic()<<",h["<<i<<"]:"<<heuristics[i]->get_heuristic()<<",g:"<<node.get_g()+1;cout<<endl;fflush(stdout);
		  succ_h=max(succ_h,heuristics[i]->get_heuristic());
	      }
	      cout<<"F_max:"<<node.get_g()+1+succ_h<<endl;
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
	      //int succ_h = heuristics[0]->get_heuristic();
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
	      if (search_progress.check_h_progress(succ_node.get_g())) {
		  reward_progress();
	      }
	  } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
	    //cout<<"Found a new cheapest path to an open or closed state!"<<endl;
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
		      cout<<"reopened closed node so heuristic must be inconsistent"<<endl;
		      exit(0);
		  }
		  //cout<<"found shorter path to open node"<<endl;
		  //exit(0);
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
  }

  pair<SearchNode, bool> HustSearch::fetch_next_node() {
      /* TODO: The bulk of this code deals with multi-path dependence,
	 which is a bit unfortunate since that is a special case that
	 makes the common case look more complicated than it would need
	 to be. We could refactor this by implementing multi-path
	 dependence as a separate search algorithm that wraps the "usual"
	 search algorithm and adds the extra processing in the desired
	 places. I think this would lead to much cleaner code. */

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

  void HustSearch::reward_progress() {
      // Boost the "preferred operator" open lists somewhat whenever
      // one of the heuristics finds a state with a new best h value.
      open_list->boost_preferred();
  }

  void HustSearch::dump_search_space() {
      search_space.dump();
  }

  void HustSearch::update_jump_statistic(const SearchNode &node) {
      if (f_evaluator) {
	  heuristics[0]->set_evaluator_value(node.get_h());
	  f_evaluator->evaluate(node.get_g(), false);
	  int new_f_value = f_evaluator->get_value();
	  search_progress.report_f_value(new_f_value);
      }
  }

  void HustSearch::print_heuristic_values(const vector<int> &values) const {
      for (int i = 0; i < values.size(); i++) {
	  cout << values[i];
	  if (i != values.size() - 1)
	      cout << "/";
      }
  }
bool HustSearch::check_cyclic_path(const State *succ_state){
  bool visited_state=false;
    
  for(int i=0;i<path_states.size();i++){
    if(*succ_state==path_states[i]){
      //cout<<"\t found duplicate at depth"<<i<<endl;
      //cout<<"succ_state:";succ_state->dump();cout<<endl;
      //cout<<"visited_state:";path_states[i].dump();cout<<endl;
      visited_state=true;
      break;
    }
  }
  if(!visited_state){
    path_states.push_back(*succ_state);
  }
  return visited_state;
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

      HustSearch *engine = 0;
      if (!parser.dry_run()) {
	  opts.set<bool>("mpd", false);
	  engine = new HustSearch(opts);
      }

      return engine;
  }

  static SearchEngine *_parse_astar(OptionParser &parser) {
      parser.add_option<ScalarEvaluator *>("eval");
      parser.add_option<bool>("pathmax", false,
			      "use pathmax correction");
    parser.add_option<bool>("mpd", false,
                            "use multi-path dependence (LM-A*)");
    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    HustSearch *engine = 0;
    if (!parser.dry_run()) {
        GEvaluator *g = new GEvaluator();
        vector<ScalarEvaluator *> sum_evals;
        sum_evals.push_back(g);
        ScalarEvaluator *eval = opts.get<ScalarEvaluator *>("eval");
        sum_evals.push_back(eval);
        ScalarEvaluator *f_eval = new SumEvaluator(sum_evals);

        // use eval for tiebreaking
        std::vector<ScalarEvaluator *> evals;
        evals.push_back(f_eval);
        evals.push_back(eval);
        OpenList<state_var_t *> *open = \
            new TieBreakingOpenList<state_var_t *>(evals, false, false);

        opts.set("open", open);
        opts.set("f_eval", f_eval);
        opts.set("reopen_closed", true);
        engine = new HustSearch(opts);
    }

    return engine;
}

/*static SearchEngine *_parse_greedy(OptionParser &parser) {
    parser.add_list_option<ScalarEvaluator *>("evals");
    parser.add_list_option<Heuristic *>("preferred", vector<Heuristic *>(), "use preferred operators of these heuristics");
    parser.add_option<int>("boost", 0, "boost value for preferred operator open lists");
    SearchEngine::add_options_to_parser(parser);


    Options opts = parser.parse();
    opts.verify_list_non_empty<ScalarEvaluator *>("evals");

    HustSearch *engine = 0;
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
        engine = new HustSearch(opts);
    }
    return engine;
}*/

static Plugin<SearchEngine> _plugin("HUST", _parse);
static Plugin<SearchEngine> _plugin_astar("HUST_IDA", _parse_astar);
//static Plugin<SearchEngine> _plugin_greedy("eager_greedy", _parse_greedy);
