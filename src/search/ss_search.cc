#include "ss_search.h"

#include "globals.h"
#include "heuristic.h"
#include "utilities.h"
#include "option_parser.h"
#include "successor_generator.h"
#include "g_evaluator.h"
#include "sum_evaluator.h"
#include "max_evaluator.h"
#include "plugin.h"
#include "type.h"
#include "type_system.h"
#include "nivel.h"

//#include "../randomc/randomc.h"
//#include "../randomc/mersenne.cpp"

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <set>
#include <iostream>
#include <fstream>

#include <map>
#include <dirent.h>

using namespace std;


//HST lsearch_space2(10000, 1);
//static bool time_limit_node_adjusted=false;
SSSearch::SSSearch(
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
	this->RanGen = new CRandomMersenne((unsigned)time(NULL));
	//this->hf = hf;
	//this->typesystem = new TypeSystem(hf);
}

void SSSearch::output_problem_results() {
	ofstream outputFile;
	outputFile.open("results.txt", ios::app);
	outputFile<<g_plan_filename<<"\t"<<"0"<<"\t"<<search_progress.get_generated()<<"\t"<<search_timer();
	outputFile<<"\t"<<g_timer()<<endl;
	outputFile.close();
}



void SSSearch::initialize() {
	cout<<" __________________________________________"<<endl;
	cout<<"|  initialize() - ss_search.cc             |"<<endl;
	cout<<" __________________________________________"<<endl;
	//use basic_ios::imbue
	std::cout.imbue(std::locale(std::cout.getloc(), new punct_facet<char, ','>));

	cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
         cout<<"first_sample set to true"<<endl;
	first_sample=true;
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


void SSSearch::statistics() const {
	search_progress.print_statistics();
	search_space.statistics();
}


string SSSearch::getRealHeuristic(string heur) {
	string heuristic;
        if (heur == "lm_cut") {
	    heuristic = heur;
	}
	
	return heuristic;
}


int SSSearch::step() {
    cout<<" ____________________________"<<endl;
    cout<<"|       step process         |"<<endl;
    cout<<" ____________________________"<<endl;
  
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
	return 1;
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
 

    //Here are create the output to write the new report.
    //create the pasta
    string pastaReporteSS = "mkdir /home/marvin/marvin/testss/"+heuristica+"/report/"+dominio;
    if (system(pastaReporteSS.c_str())) {
	cout<<"the directory was not created."<<endl;
    } else {
	cout<<"the directory "<<dominio<<" was created"<<endl;
    }

    string arquivoRSS =  "/home/marvin/marvin/testss/"+heuristica+"/report/"+dominio+"/"+tarefa;

    ofstream outputfile;
    outputfile.open(arquivoRSS.c_str(), ios::out);

    //Here we are in the test directory and not the testss directory. 
    string rutaR = "/home/marvin/marvin/test/"+heuristica+"/report/"+dominio+"/"+tarefa;
    cout<<"rutaR = "<<rutaR.c_str()<<endl;
    ifstream fileR(rutaR.c_str());
    
    string titulo;
    int totalniveles;
    string str; //totalniveles title, f, total nodes, time
    fileR>>titulo;
    outputfile<<"\t\t"<<titulo<<"\n";
    cout<<"titulo = "<<titulo<<endl;
    fileR>>str;
    cout<<"totalniveles texto =  "<<str<<endl;
    fileR>>totalniveles;
    outputfile<<"\ttotalniveles: "<<totalniveles<<"\n";
    cout<<"value total niveles = "<<totalniveles<<endl;
    fileR>>str;
    cout<<"f = "<<str<<endl;
    fileR>>str;
    cout<<"#nodes = "<<str<<endl;
    fileR>>str;
    cout<<"time = "<<str<<endl;
    fileR>>str;
    cout<<"#nodes2 = "<<str<<endl;
    outputfile<<"\tf\t#nodes_by_level\ttime(s)\t#nodes_to_the_level\tsum_By_Depth\n";
     
    //Initialize niveles
    float** niveles = new float*[totalniveles];

    for (int i = 0; i < totalniveles; i++) {
	niveles[i] = new float[4];
    }
   
    vector<Nivel> v_niveles;
    vector<int> v_nivel;
   
    for (int i = 0; i < totalniveles; i++) {
	for (int j = 0; j < 4; j++) {
             fileR>>niveles[i][j];	     
	}
    }
    
    for (int i = 0; i < totalniveles; i++) {
   	v_nivel.insert(v_nivel.begin() + i, niveles[i][0]);   
        //Nivel nivel(niveles[i][0], niveles[i][1], niveles[i][2]);
        //v_niveles.insert(v_niveles.begin() + i, nivel);
    }
   
    //To collect the total of nodes by depth (Stratified Sampling)
    vector<long> sumByDepth;

    for (int i = 0; i < v_nivel.size(); i++) {
        cout<<v_nivel.at(i)<<endl;
    }

    cout<<"List of levels: "<<endl;
    vector<vector<int> > v_v_f_value;
    vector<int> v_g_dist; 
    for (std::vector<int>::size_type w = 0; w != v_nivel.size(); w++) {
	cout<<"depth = "<<v_nivel.at(w)<<endl;
    	int depth = v_nivel.at(w);

    	//Initialize the same queue.
    	//fetch next node, now just have to call the initial node.
    	SearchNode node = search_space.get_node(*g_initial_state);    //n.first; 

    	cout<<"heuristic value of te initial node based on the heuristic vector = "<<heuristics[0]->get_value()<<endl;
    	node.open_initial(heuristics[0]->get_value());

    	map<Type, SearchNode> queue;
    	//include type system
    	TypeSystem ts;
    	//long levelInicial = 1;
    	Type object = ts.getType(node, 1);//SearchNode, level initialized with 1 
  
    	cout<<"heuristic value of the initial node based on the node = "<<node.get_h()<<endl;
    	cout<<"heuristic value of the object Type  = "<<object.getH()<<endl; 
    
    	//set w = 1
    	node.setW(1);
    	//root node added to the queue
    	queue.insert(pair<Type, SearchNode>(object, node));

    	int k = 0;
    	vector<int> sumw;  //Generated Nodes
	vector<int> v_f_value;
	//Initialize map of the g value.
 	map<int, vector<int> > mapg;
    	while(!queue.empty()) {
		//tira o menor de acordo com o sistema de tipos
		Type out = queue.begin()->first;
	
		SearchNode nodecp = queue.begin()->second;
        	//remover el primer nodo o todos los que pertenecen a un mismo nivel
		sumw.insert(sumw.begin() + k, nodecp.getW());
		queue.erase(out);

        	int g = out.getLevel(); //level del type
		int w = nodecp.getW();  //level del node

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
		 	 //sample_frontier_now(nodecp.get_g()+nodecp.get_h());
      	      	      }
    	   	   }
  		}

		State newState = nodecp.get_state();

        	//obtener los op
		vector<const Operator *> applicable_ops;
    		set<const Operator *> preferred_ops;

    		g_successor_generator->generate_applicable_ops(newState, applicable_ops);
     		 
		for (int i = 0; i < applicable_ops.size(); i++) {
	     	     const Operator *op = applicable_ops[i];
	     	     State succ_state(newState, *op);
	     	     //search_progress.inc_generated();	     
	     	     SearchNode succ_node = search_space.get_node(succ_state);
	     	     //calcular el valor heuristico del succ_node
	     	     //cout<<"heuristics[0]->get_heur_name() = "<<heuristics[0]->get_heur_name()<<endl; 
	     	     heuristics[0]->evaluate(succ_state); 
	     	     int succ_h = heuristics[0]->get_heuristic();
	    	     succ_node.open(succ_h, nodecp, op);
	
	     	     int succ_h2 = succ_node.get_h();		
             	     //cout<<"succ_h2 = "<<succ_h2<<endl;
             	     //int succ_h2_value = heuristics[0]->get_value();
             	     //cout<<"succ_h2_value = "<<heuristics[0]->get_value()<<endl;
	     	     int succ_g = succ_node.get_real_g();
	     	     //cout<<"succ_g = "<<succ_g<<endl;
		
		     if (succ_h2 + succ_g <= depth) {
			int succ_f = succ_g + succ_h2;
		        v_f_value.push_back(succ_f);

			//succ_node.open(succ_h, nodecp, op);
	     		TypeSystem ts2;
			Type type =  ts2.getType(succ_node, g);
             		type.setLevel(g + 1);
	     		succ_node.setW(w);
		
        		map<Type, SearchNode>::iterator queueIt = queue.find(type);
	     		//cout<<"expression = "<<expression<<endl;
	     		if (queueIt != queue.end()) { // Check whether a key has associated value in a map.
		  	   //the type already exists in the queue
		    	   int wa = queueIt->second.getW();

		    	   queueIt->second.setW(wa + w);
                    	   double prob = (double)w/(wa +w);
		    	   //long random_number = rand();			
		    	   int  rand_100 = RanGen->IRandom(0, 99);
		    	   double a = (double)rand_100/100;
		    	   if (a < prob) {
		              succ_node.setW(wa + w);
	               	      queue.insert(pair<Type, SearchNode>(type, succ_node)); 
	            	   }
                 	}  else {
		      	    //the type does not exists in the queue, so add it to the queue
		       	    queue.insert(pair<Type, SearchNode>(type, succ_node)); 
	         	} 
	     	     } //end if pruning g + h <= depth	
		     //} //end if prunning g == depth
		} //end for from applicable_ops
		k = k + 1;
		mapg.insert(pair<int, vector<int> >(g, v_f_value));
    	} //end while
	cout<<"v_f_value.size() = "<<v_f_value.size()<<endl;
        vector<int> v_g;
        int p = 0;
        cout<<"****************************************************************"<<endl;
        cout<<"for depth = "<<v_nivel.at(w)<<" we print the levels and the number of levels."<<endl;
        for (map<int, vector<int> >::iterator it = mapg.begin(); it != mapg.end(); ++it) {
	     int g = it->first;
	     cout<<"g = "<<g<<endl;
	     v_g.insert(v_g.begin() + p, g);
             vector<int> vect = it->second;
             cout<<"f-value generated at "<<g<<" level."<<endl;
             for (int i = 0; i< vect.size(); i++) {
		cout<<vect.at(i)<<", ";
	     }
             cout<<"\n";
             p = p + 1;
	}
        cout<<"p = "<<p<<endl;	

	int max_g = getMax_gvalue(v_g);
        cout<<"The f-Distribution for max_g of this iteration "<<max_g<<endl;
	map<int, vector<int> >::iterator mapgIt = mapg.find(max_g);
       	if (mapgIt != mapg.end()) {
	    vector<int> v_gmax = mapgIt->second;
	    v_v_f_value.insert(v_v_f_value.begin() + w, v_gmax);
	    v_g_dist.insert(v_g_dist.begin() + w, max_g); 
	    for (int i = 0; i < v_gmax.size(); i++) {
		cout<<v_gmax.at(i)<<", ";
	    }
	}
        
	cout<<"\n****************************************************************"<<endl;	
    	cout<<"counter k = "<<k<<endl;

    	//prediction generated nodes
    	long total = 0;
    	for (int i = 0; i < sumw.size(); i++) {
        	total += sumw.at(i);
    	}
    	sumByDepth.insert(sumByDepth.begin() + w, total);

    	//sumw.clear();
    	cout<<" _____________________________________________________________________"<<endl;
    	cout<<"|   # of nodes expanded by ss at level "<<v_nivel.at(w)<<" is :     "<<total<<"                |"<<endl;
    	cout<<" _____________________________________________________________________"<<endl;
    }//end for levels

    for (int i = 0; i < totalniveles; i++) {
	 outputfile<<"\t"<<niveles[i][0]<<"\t"<<niveles[i][1]<<"\t\t"<<niveles[i][2]<<"\t\t"<<niveles[i][3]<<"\t\t"<<sumByDepth.at(i)<<"\n";
    }
    outputfile.close(); 

    //Print the f-value Distribution
    string fdist = "mkdir /home/marvin/marvin/testss/"+heuristica+"/report/"+dominio+"/fdist";
    if (system(fdist.c_str())) {
       cout<<"the directory was not created"<<endl;
    } else {
       cout<<"the directory "<<dominio<<" was created."<<endl;
    }

    string arqfDist =  "/home/marvin/marvin/testss/"+heuristica+"/report/"+dominio+"/fdist/"+tarefa;

    ofstream outputfile2;
    outputfile2.open(arqfDist.c_str(), ios::out);
    outputfile2<<"\t\t"<<titulo<<"\n";
    outputfile2<<"\ttotalniveles: "<<totalniveles<<"\n";
    cout<<"-----------------Print the f-Distribution of each level-----------------"<<endl;
    for (int i = 0; i < v_v_f_value.size(); i++) {
        int max_g = v_g_dist.at(i);
 
	vector<int> v_max = v_v_f_value.at(i);
	map<int, int> m = getFDistribution(v_max);
        outputfile2<<"\tlevel: "<<max_g<<endl;
	for (map<int, int>::iterator it = m.begin(); it != m.end(); ++it) {
	     int f = it->first;
             int q = it->second;
	     outputfile2<<"\t\tf: "<<f<<"\t\tq: "<<q<<endl;
	     cout<<"f = "<<f<<" q = "<<q<<endl;
	}

	for (int j = 0; j < v_max.size(); j ++) {
	    int k = v_max.at(j); 
	    cout<<k<<" ";
	}
	cout<<"\n";
    }
    outputfile2.close();

    return SOLVED;
}


map<int, int> SSSearch::getFDistribution(vector<int> v_f_value) {
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

int SSSearch::getMax_gvalue(vector<int> v_g) {
	int winner = v_g.at(0);
	for (int i = 0; i < v_g.size(); i++) {
	     if (winner < v_g.at(i)) {
		winner = v_g.at(i);
	     }
	}
	return winner;
}

pair<SearchNode, bool> SSSearch::fetch_next_node() {
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


void SSSearch::reward_progress() {
     open_list->boost_preferred();
}
/*
void SSSearch::dump_search_space() {

}


void SSSearch::sample_frontier_now(int next_f_boundary) {
	cout<<" ________________________________"<<endl;
	cout<<"|   next_f_boundary              |"<<endl;
	cout<<" ________________________________"<<endl;	
}


void SSSearch::update_jump_statistic(const SearchNode &node) {	
	cout<<" __________________________________"<<endl;
	cout<<"|   update jump statistic          |"<<endl;
	cout<<" __________________________________"<<endl;
	//cout<<"node = "<<node<<endl;
}


void SSSearch::print_heuristic_values(const vector<int> &values) const {

}
*/

static SearchEngine *_parse_ss(OptionParser &parser) {
	cout<<" ______________________________"<<endl;
	cout<<"|  parse_ss - ss_search.cc     |"<<endl;
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

	SSSearch *engine = 0;
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
		engine = new SSSearch(opts);
		cout<<"engine = "<<engine<<endl;
	}	
	return engine;
}

static Plugin<SearchEngine> _plugin_ss("ss", _parse_ss);


