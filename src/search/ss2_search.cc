#include "ss2_search.h"
#include "globals.h"
#include "option_parser.h"
#include "successor_generator.h"
#include "plugin.h"
#include "rng.h"

#include <iostream>
#include <fstream>

SS2Search::SS2Search(const Options &opts) :SearchEngine(opts), current_state(*g_initial_state){
	rg = opts.get<double>("rg");
	rl = opts.get<double>("rl");
	lookahead = opts.get<int>("lookahead");
	beamsize = opts.get<int>("beamsize");
	maxlevels = opts.get<int>("maxlevels");
	timelimit = opts.get<int>("timelimit");
	cout << "rg: " << rg << endl;
	ScalarEvaluator * evaluator = opts.get<ScalarEvaluator *>("eval");
	std::set<Heuristic *> hset;
	evaluator->get_involved_heuristics(hset);
	for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); it++) {
		heuristics.push_back(*it);
	}
	assert(heuristics.size() == 1);
	heuristic = heuristics[0];

	sampler = new TypeSystem(heuristic);
       
}

SS2Search::~SS2Search() {
}


int SS2Search::step()
{
	/*
	 * Probing is done based on the types of the children of the root state
	 */

        //queue.clear();
	// evaluating the initial state
	heuristic->evaluate(*g_initial_state);
	if (heuristic->is_dead_end())
	{
		assert(heuristic->dead_ends_are_reliable());
		cout << "Initial state is a dead end." << endl;
		exit(0);
	}
	initial_value = heuristic->get_value();

        //for the open domains the heuristic is set to six
        threshold = 2*initial_value;

	// adding the initial state to open
	//SSNode node(*g_initial_state, 1.0);
        SSNode node;
        node.setState(*g_initial_state);
        node.setWeight(1.0);
	/*
	 * Seeding the prediction with the children of the start state
	 *
	 */
	Type type = sampler->getType(node.getState(), initial_value, lookahead);
              
	type.setLevel( 0 );

	queue.insert( pair<Type, SSNode>( type, node ) );
        
        Node2 node2(initial_value + type.getLevel(), type.getLevel());
        count_value = 1;
       
	while( !queue.empty() )
	{
		Type out = queue.begin()->first;
		SSNode s = queue.begin()->second;
               	int g = (int)out.getLevel();

                printQueue();
                std::map<Type, SSNode>::iterator ret0;
                ret0 = queue.find(out);


		queue.erase( ret0 );
                       
		vweight.push_back(s);
		
	        
                //Insert each node.
                Node2 node2(out.getH() + g, g);
                collector.insert(pair<Node2, unsigned long long>(node2, s.getWeight()));
                //cout<<"Raiz: h = "<<out.getH()<<" g = "<<g<<" f = "<<out.getH() + g<<" w = "<<s.getWeight()<<"\n";
                /*
                if (collector.insert(pair<Node2, int>(node2, s.getWeight())).second) {
                   count_value = s.getWeight();
                   cout<<"Raiz: h = "<<out.getH()<<" g = "<<g<<" f = "<<out.getH() + g<<" w = "<<s.getWeight()<<"\n";
                } else {
                   map<Node2, int>::iterator iter = collector.find(node2);
                   int q = iter->second;
                   cout<<"Duplicate: h = "<<out.getH()<<" g = "<<g<<" f = "<<out.getH() + g<<"\n";
                   q++;
                   iter->second = q;
                }*/	

		int h = -1;
		double w = (double)s.getWeight();
		 
                std::vector<const Operator*> applicable_ops;
		g_successor_generator->generate_applicable_ops(s.getState(), applicable_ops);
		for (int i = 0; i < applicable_ops.size(); ++i)
		{
			State child(s.getState(), *applicable_ops[i]);

			heuristic->evaluate(child);

			//int h = -1;

			if(!heuristic->is_dead_end())
			{
				h = heuristic->get_heuristic();

			}	
                    
                        //cout<<"\tChild: h = "<<h<<" g = "<<g+1<<" f = "<<h+g+1<<" w = "<<w<<"\n";

                        if (h + g + 1 <= threshold) {
			   Type object = sampler->getType(child, h,  lookahead);
			   
                           object.setLevel( g + 1 );
 
                           SSNode child_node;
                           child_node.setState(child);
                           child_node.setWeight(w);		   

                           //cout<<"\tChild: h = "<<object.getH()<<" g = "<<object.getLevel()<<" f = "<<object.getH()+object.getLevel()<<" w = "<<child_node.getWeight()<<endl;

                           map<Type, SSNode>::iterator queueIt = queue.find( object );
			   if( queueIt != queue.end() )
			   {

                                //cout<<"\tis duplicated: h = "<<queueIt->first.getH()<<" g = "<<queueIt->first.getLevel()<<" f = "<<queueIt->first.getH()+queueIt->first.getLevel()<<" w = "<<queueIt->second.getWeight()<<"\n";


                                SSNode snode = queueIt->second;
				double wa = (double)snode.getWeight();
				
                                queueIt->second.setWeight(wa + w); 
                               
 
				double prob = ( double )w / (double)( wa + w );
				int rand_100 = (int)g_rng.next(100);
                          	 
                                double a = (( double )rand_100) / 100;
                                
				if (a < prob) 
				{
                                        cout<<"\t\tAdded even though is duplicate.\n";
                                        
				        child_node.setWeight( wa + w);
                                        
                                        //cout<<"\t\th = "<<object.getH()<<" g = "<<object.getLevel()<<" f = "<<object.getH()+object.getLevel()<<" w = "<<child_node.getWeight()<<"\n";

                                        std::pair<std::map<Type, SSNode>::iterator, bool> ret;
                                     	queue.erase(object); 

                                        ret = queue.insert( pair<Type, SSNode>( object, child_node ));      

                                        queueIt = ret.first;
                                        queueIt->second.setWeight(child_node.getWeight());
                                        
					//cout <<"\t\tnew w = "<<queue[object].getWeight() << endl;
				} else {
                                        //cout<<"\t\tNot added.\n";
                                }
			   } 
			   else
			   {
                                //cout<<"\t\tNew node added\n";
                                //cout<<"\t\th = "<<h<<" g = "<<g+1<<" f = "<<h+g+1<<" threshold = "<<threshold<<endl;
				queue.insert( pair<Type, SSNode>( object, child_node ) );
                           }
                        } else {
                                //cout<<"\tNode was pruned!"<<endl;
                                //cout<<"\th = "<<h<<" g = "<<g+1<<" f = "<<h+g+1<<" threshold = "<<threshold<<endl;
                        }
		}
	}
        
        generateReport();

	return SOLVED;
}


/*

int SS2Search::step()
{
	
        // Probing is done based on the types of the children of the root state
	 
	int last_level = 0;
	int expansions_level = 0;
	//int number_levels_without_progress = 0;
        count_value = 1;
        count_level_value = 1;
	while( !queue.empty() )
	{
		Type out = queue.begin()->first;
		SSNode s = queue.begin()->second;
		queue.erase( out );
                //Insert SSNode to count w
                mweight.insert(pair<int, SSNode>(count_level_value, s));
                count_level_value++; 
		//output.insert( pair<SemiLosslessObject, PKState> ( out, s ) );
		int g = out.getLevel();
               
                //Insert each node.
                Node2 node2(out.getH() + g, g);
                if (collector.insert(pair<Node2, int>(node2, count_value)).second) {
                   count_value = 1;
                } else {
                   map<Node2, int>::iterator iter = collector.find(node2);
                   int q = iter->second;
                   q++;
                   iter->second = q;
                }

		//negative beam sizes equal to an allowance of infinity number of nodes expansions
		if(beamsize > 0 && g == last_level && expansions_level > beamsize)
		{
			continue;
		}

		expansions_level++;

		int w = s.weight;
		std::vector<const Operator*> applicable_ops;
		g_successor_generator->generate_applicable_ops(s.state, applicable_ops);
		for (int i = 0; i < applicable_ops.size(); ++i)
		{
			State child(s.state, *applicable_ops[i]);

			heuristic->evaluate(child);

			int h = -1;

			if(!heuristic->is_dead_end())
			{
				h = heuristic->get_heuristic();

				if(h < total_min)
				{
					current_state = child;
					total_min = h;
					progress = true;
					report_progress();
				}
			}
			else
			{
				continue;
			}	

                        if (g <= threshold) {
			   Type object = sampler->getType(child, h,  lookahead);
			   object.setLevel( g + 1 );

			   SSNode child_node(child, w);
                           //cout<<"\tChild: h = "<<object.getH()<<" g = "<<object.getLevel()<<" f = "<<object.getH() + object.getLevel()<<"\n"; 
			   std::map<Type, SSNode>::iterator queueIt = queue.find( object );
			   if( queueIt != queue.end() )
			   {
				int wa = queueIt->second.weight;
				queueIt->second.weight =  wa + w;

				double prob = ( double )w / ( wa + w );
				int rand_100 = g_rng.next(100);
				double a = ( double )rand_100 / 100;

				if(a < prob)
				{
                                        //cout<<"Added even though is duplicate.\n";
					child_node.weight = wa + w;
					queue.insert( pair<Type, SSNode>( object, child_node ) );
				}
			   }  
			   else
			   {
                                //cout<<"new added\n";
				queue.insert( pair<Type, SSNode>( object, child_node ) );
			   }
                        }
		}
	}
        generateReport();

	return SOLVED;
}
*/

void SS2Search::generateReport() {
        string dominio = domain_name;
        string tarefa = problem_name2;
        string heuristica = heuristic_name2;

        cout<<"dominio = "<<dominio<<endl;
        cout<<"tarefa = "<<tarefa<<endl;
        cout<<"heuristica = "<<heuristica<<endl;

        string dirDomain = "mkdir /home/marvin/marvin/testss/"+heuristica+"/reportss/"+dominio;
        string dirfDist = "mkdir /home/marvin/marvin/testss/"+heuristica+"/reportss/"+dominio+"/fdist";
       
        string outputFile = "/home/marvin/marvin/testss/"+heuristica+"/reportss/"+dominio+"/fdist/"+tarefa;

        ofstream output;

        output.open(outputFile.c_str());
        output<<"\t"<<outputFile.c_str()<<"\n" ;
        output<<"predictionSS: "<<getProbingResult()<<"\n";
        output<<"threshold: "<<threshold<<"\n";
        

        if (system(dirDomain.c_str())) {
           cout<<"Directory: "<<heuristica<<" created."<<endl;
        }

        if (system(dirfDist.c_str())) {
           cout<<"Directory: fdist created."<<endl;
        }
        cout<<"print."<<endl;
        for (int i = 0; i <= threshold; i++) {
            int k = 0;
            vector<long> f;
            vector<unsigned long long> q;
            for (map<Node2, unsigned long long>::iterator iter = collector.begin(); iter != collector.end(); iter++) {
                 Node2 n = iter->first;
                 if (i == n.getL()) {
                    k++;
                    f.push_back(n.getF());
                    q.push_back(iter->second);
                 }
            }
            cout<<"g:"<<i<<"\n";
            output<<"g:"<<i<<"\n";

            cout<<"size: "<<k<<"\n";            
            output<<"size: "<<k<<"\n"; 
            for (int j = 0; j < f.size(); j++) {
                 cout<<"\tf: "<<f.at(j)<<"\tq: "<<q.at(j)<<"\n";
                 output<<"\tf: "<<f.at(j)<<"\tq: "<<q.at(j)<<"\n";
            }
            cout<<"\n";
            output<<"\n";
        }
        output.close();
}

double SS2Search::getProbingResult() {
        double expansions = 0;
        
        for (int i = 0; i < vweight.size(); i++) {
             SSNode n = vweight.at(i);
             expansions += n.getWeight();

		cout << n.getWeight() << " ";
        }
	cout << endl;
        cout<<"expansions = "<<expansions<<endl;
        return expansions;
}

void SS2Search::printQueue() {
        cout<<"\n\t\tPrintQueue\n";
	for (map<Type, SSNode>::iterator iter = queue.begin(); iter !=  queue.end(); iter++) {
            Type t = iter->first;
            SSNode t2  = iter->second;
            cout<<"\t\t h = "<<t.getH()<<" g = "<<t.getLevel()<<" f = "<<t.getH() + t.getLevel()<<" w = "<<t2.getWeight()<<"\n"; 
        }
        cout<<"\n";
        cout<<"\n\t\tEnd PrintQueue\n";
}


void SS2Search::report_progress()
{
	cout << "h_min: " << total_min << " depth: " << depth << " #states: " << queue.size() << " time: " << search_time << endl;
}

void SS2Search::initialize() {
	cout << "SS2Search ..." << endl;
	search_time.reset();
	level_time.reset();
        
	//queue.clear();
	// evaluating the initial state
	heuristic->evaluate(*g_initial_state);
	if (heuristic->is_dead_end())
	{
		assert(heuristic->dead_ends_are_reliable());
		cout << "Initial state is a dead end." << endl;
		exit(0);
	}
	initial_value = heuristic->get_value();
	total_min = initial_value;
        /*
        //for the open domains the heuristic is set to six
        threshold = 2*initial_value;

	// adding the initial state to open
	SSNode node(*g_initial_state, 1);


	// Seeding the prediction with the children of the start state
	 
	 
	Type type = sampler->getType(node.state, initial_value, lookahead);

	type.setLevel( 0 );
	node.weight = 1;
	queue.insert( pair<Type, SSNode>( type, node ) );

	//open.insert(make_pair(initial_value, node));
        //Initialize 
        //count_value = 1;
        //Node2 node2(initial_value + type.getLevel(), type.getLevel());
        //collector.insert(pair<Node2, int>(node2, count_value));
        */
	progress = true;
	cout << "Initial heuristic value: ";
	cout << initial_value << endl;
	depth = 0;
	report_progress();
	depth ++;
}

static SearchEngine *_parse(OptionParser &parser) {
	parser.add_option<ScalarEvaluator *>("eval");
	parser.add_option<double>("rg", DEFAULT_SS_RG, "the global restarting rate");
	parser.add_option<double>("rl", DEFAULT_SS_RL, "the local restarting rate");
	parser.add_option<int>("lookahead", DEFAULT_SS_LOOKAHEAD, "lookahead that defines the type system being used");
	parser.add_option<int>("beamsize", DEFAULT_SS_BEAMSIZE, "maximum number of nodes expanded by level");
	parser.add_option<int>("maxlevels", DEFAULT_SS_MAXLEVELS, "maximum number of nodes expanded by level");
	parser.add_option<int>("timelimit", DEFAULT_SS_TIMELIMIT, "time limit in seconds to solve the problem");
	SearchEngine::add_options_to_parser(parser);
	Options opts = parser.parse();
	SS2Search *engine = 0;
	if (!parser.dry_run()) {
		engine = new SS2Search(opts);
	}
	return engine;
}


static Plugin<SearchEngine> _plugin("ss2", _parse);
