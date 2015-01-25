#include "ss_search.h"
#include "globals.h"
#include "option_parser.h"
#include "successor_generator.h"
#include "plugin.h"
#include "rng.h"

#include <iostream>
#include <fstream>

SSSearch::SSSearch(const Options &opts) :SearchEngine(opts), current_state(*g_initial_state){
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
        this->RanGen = new CRandomMersenne((unsigned)time(NULL));
}

SSSearch::~SSSearch() {
}


int SSSearch::step() {
        predict(ss_probes);
        return SOLVED;
}

void SSSearch::predict(int probes) {
        double totalPrediction = 0;
        for (int i = 0; i < probes; i++) {
            vweight.clear();
            probe();
            double p = getProbingResult();
            totalPrediction = totalPrediction + (p - totalPrediction)/(i + 1);
            cout<<"**********"<<endl;
            cout<<"p = "<<p<<endl;
            cout<<"prePre = "<<totalPrediction<<endl;
            cout<<"**********"<<endl;
        }
        cout<<"totalPrediction = "<<totalPrediction<<endl;
}

void SSSearch::probe()
{
	/*
	 * Probing is done based on the types of the children of the root state
	 */

        queue.clear();
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
        //Type type = sampler->getType2(initial_value, 0);       
 
	type.setLevel( 0 );

	queue.insert( pair<Type, SSNode>( type, node ) );
        //S.insert( pair<Type, double>( type, 0.0 ) );	
        
        count_value = 1;
        //count_level_value = 0;


        int count1 = 1;
        int count2 = 1;
        int nraiz = 1; 
	while( !queue.empty() )
	{
		Type out = queue.begin()->first;
		SSNode s = queue.begin()->second;
               	int g = (int)out.getLevel();

                printQueue();
		queue.erase( out );
                cout<<nraiz<<": Raiz: h = "<<out.getH()<<" g = "<<out.getLevel()<<" f = "<<out.getH() + out.getLevel()<<" w  = "<<s.getWeight()<<endl;   
                nraiz++;                
                
		vweight.push_back(s);
		
	              /* 
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
*/
		int h = -1;
		double w = (double)s.getWeight();
		cout<<"w = "<<w<<endl;  
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

			cout<<"\tChild: h = "<< h <<" g = "<< g + 1 <<" f = "<< h + g + 1 <<" w = "<<w<<endl; 

                        if (h + g + 1 <= threshold) {
			   Type object = sampler->getType(child, h,  lookahead);
			   //Type object = sampler->getType2(h, g + 1);
                           object.setLevel( g + 1 );

			   //SSNode child_node(child, w);
                           SSNode child_node;
                           child_node.setState(child);
                           child_node.setWeight(w);

                           //cout<<"\tChild: h = "<<object.getH()<<" g = "<<object.getLevel()<<" f = "<<object.getH() + object.getLevel()<<"\n"; 
			   

                           map<Type, SSNode>::iterator queueIt = queue.find( object );
			   if( queueIt != queue.end() )
			   {

                                cout<<"\tis duplicate: h = "<<queueIt->first.getH()<<" g = "<<queueIt->first.getLevel()<<" f = "<< queueIt->first.getH()   +  queueIt->first.getLevel()<<"\n";
                                SSNode snode = queueIt->second;
				double wa = (double)snode.getWeight();
				snode.setWeight( wa + w);

                                std::pair<std::map<Type, SSNode>::iterator, bool> ret0;

                                ret0 = queue.insert(pair<Type, SSNode>(object, snode));
                                cout<<"\tsnode.getWeight() = "<<snode.getWeight()<<endl;
                                queueIt->second.setWeight(snode.getWeight());
 
 
				double prob = ( double )w / ( wa + w );
				int rand_100 = RanGen->IRandom(0, 99);  //(int)g_rng.next(100);
                          	 
                                double a = (( double )rand_100) / 100;

                                
				//child_node.setWeight( wa + w);

                                
				if (a < prob) 
				{
                                        //queue.erase(queueIt->first);
                                        //S.erase(queueIt->first);                                       

                                        cout<<"\t\tAdded even though is duplicate.\n";
                                        
				        child_node.setWeight( wa + w);
                                       
                                        cout<<"\t\tw = "<<child_node.getWeight()<<endl;
                                        cout<<"\t\tChild: h = "<< h <<" g = "<< g + 1 <<" f = "<< h + g + 1 <<"\n";	
				        cout<<"\t\tbefore insert."<<endl;
                                        printQueue();
                                        cout<<"\t\t190: child_node.getWeigt() = "<<child_node.getWeight()<<endl;
                                        
                                        std::pair<std::map<Type, SSNode>::iterator, bool> ret;
                                      

                                        ret = queue.insert( pair<Type, SSNode>( object, child_node ));      

                                        queueIt = ret.first;
                                        queueIt->second.setWeight(child_node.getWeight());
                                        
                                        cout<<"\t\tbegin for."<<endl;
                                        for (; queueIt !=  queue.end(); queueIt++) {
						Type t3 = queueIt->first;
						SSNode t4 = queueIt->second;
                                                cout<<"\t\th = "<<t3.getH()<<" g = "<<t3.getLevel()<<" f = "<<t3.getH() + t3.getLevel()<<" w = "<<t4.getWeight()<<endl;
					}
                                        cout<<"\t\tend for"<<endl;
                                         
                                        cout<<"\t\t192: child_node.getWeight() = "<<child_node.getWeight()<<endl;
 
                                        cout<<"\t\tafter insert."<<endl;
                                        printQueue();
                                        //queueIt->second.setWeight(wa + w);
                                        map<Type, SSNode>::iterator it2 = queue.find(object);
                                        Type t1 = it2->first;
                                        SSNode t2 = it2->second; 
                                        cout<<"\t\th = "<<t1.getH()<<" g = "<<t1.getLevel()<<" f = "<<t1.getH() + t1.getLevel()<<" w = "<<t2.getWeight()<<endl;

                                        //S.insert( make_pair<Type, double>( object, it2->second + w ) );
					count1++;
				} else {
                                        cout<<"\t\tNot added.\n";
                                }
			   } 
			   else
			   {
                                cout<<"\t\tNew node added\n";
				queue.insert( pair<Type, SSNode>( object, child_node ) );
                                cout<<"\t\tchild_node.getWeight() = "<<child_node.getWeight()<<"\n";
                                //S.insert( pair<Type, double>( object,12.44 ) );
                                count2++;
                                cout<<"\t\tChild: h = "<< h <<" g = "<< g + 1 <<" f = "<< h + g + 1 << " threshold: " << threshold <<" w = "<<child_node.getWeight()<<endl;
                           }
                        }
			else 
			{
				cout << "\tNode was pruned!" << endl;
				cout<<"\tChild: h = "<< h <<" g = "<< g + 1 <<" f = "<< h + g + 1 << " threshold: " << threshold <<"\n";
			}
		}
	}

        cout<<"count1 = "<<count1<<endl;
        cout<<"count2 = "<<count2<<endl;
        /*
        double sumS = 0.0;
        for (map<Type, SSNode>::iterator it = S.begin(); it != S.end(); ++it) {
            SSNode n = it->second;
            sumS = sumS + n.getWeight();
        }
        cout<<"sumS = "<<sumS<<endl;
        */
}


/*

int SSSearch::step()
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

void SSSearch::generateReport() {
        string dominio = domain_name;
        string tarefa = problem_name2;
        string heuristica = heuristic_name2;

        cout<<"dominio = "<<dominio<<endl;
        cout<<"tarefa = "<<tarefa<<endl;
        cout<<"heuristica = "<<heuristica<<endl;

        string dirDomain = "mkdir /home/levi/marvin/marvin/testss/"+heuristica+"/reportss/"+dominio;
        string dirfDist = "mkdir /home/levi/marvin/marvin/testss/"+heuristica+"/reportss/"+dominio+"/fdist";
       
        string outputFile = "/home/levi/marvin/marvin/testss/"+heuristica+"/reportss/"+dominio+"/fdist/"+tarefa;

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
            vector<long> q;
            for (map<Node2, int>::iterator iter = collector.begin(); iter != collector.end(); iter++) {
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

double SSSearch::getProbingResult() {
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

void SSSearch::printQueue() {
        cout<<"\nPrintQueue\n";
	for (map<Type, SSNode>::iterator iter = queue.begin(); iter !=  queue.end(); iter++) {
            Type t = iter->first;
            SSNode t2  = iter->second;
            cout<<"\t\t h = "<<t.getH()<<" g = "<<t.getLevel()<<" f = "<<t.getH() + t.getLevel()<<" w = "<<t2.getWeight()<<"\n"; 
        }
        cout<<"\n";
        cout<<"\nEnd PrintQueue\n";
}


void SSSearch::report_progress()
{
	cout << "h_min: " << total_min << " depth: " << depth << " #states: " << queue.size() << " time: " << search_time << endl;
}

void SSSearch::initialize() {
	cout << "SSSearch ..." << endl;
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
	SSSearch *engine = 0;
	if (!parser.dry_run()) {
		engine = new SSSearch(opts);
	}
	return engine;
}


static Plugin<SearchEngine> _plugin("ss", _parse);
