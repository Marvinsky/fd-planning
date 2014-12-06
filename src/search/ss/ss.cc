#include "ss.h"
#include "../globals.h"
#include "../option_parser.h"
#include "../successor_generator.h"
#include "../plugin.h"
#include "../rng.h"


SS::SS(const Options &opts) :SearchEngine(opts), current_state(*g_initial_state){
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

	sampler = new TypeSampler(heuristic);
}

SS::~SS() {
}

// determines if the algorithm should restart from the initial state or not
bool SS::global_restart() {
	if(!progress)
	{
		// only restart if no progress made in the last iteration
		double r = g_rng.next_half_open();
		if(r < rg)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

// jumps to the state with minimum heuristic value.
void SS::jump() {

	if(!progress)
	{
		// only restart if no progress made in the last iteration
		double r = g_rng.next_half_open();
		//cout << r << " " << rl << endl;
		if(r < rl)
		{
			queue.clear();
			SSNode node(current_state, 1);
			Type type = sampler->getType(node.state, total_min, lookahead);

			type.setLevel( 0 );
			node.weight = 1;
			queue.insert( pair<Type, SSNode>( type, node ) );

			//open.clear();
			//SSNode node(current_state, 1);
			//open.insert(make_pair(total_min, node));
			cout << "jump ...." << endl;
			progress = true;
		}
	}
}

int SS::step()
{
	/*
	 * Probing is done based on the types of the children of the root state
	 */
	int last_level = 0;
	int expansions_level = 0;
	int number_levels_without_progress = 0;

	while( !queue.empty() )
	{
		Type out = queue.begin()->first;
		SSNode s = queue.begin()->second;
		queue.erase( out );

		//output.insert( pair<SemiLosslessObject, PKState> ( out, s ) );
		int g = out.getLevel();

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

			if (test_goal(child))
			{
				cout << "[PROBLEM SOLVED] -- SUCCEEDED" << endl;
				return SOLVED;
			}

			Type object = sampler->getType(child, h,  lookahead);
			object.setLevel( g + 1 );

			SSNode child_node(child, w);

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
					child_node.weight = wa + w;
					queue.insert( pair<Type, SSNode>( object, child_node ) );
				}
			}
			else
			{
				queue.insert( pair<Type, SSNode>( object, child_node ) );
			}
		}

		if(last_level != g)
		{
			double time = level_time();
			double time_per_node = time / (double) expansions_level;

			if(time < 0 || time_per_node < 0)
			{
				time = 0;
				time_per_node = 0;
			}

			if(timelimit - search_time() + 100 < 0)
			{
				cout << "[TIME IS UP] -- FAILED" << endl;
				return FAILED;
			}

	//		cout << "NUMERATOR: " << (double) (timelimit - search_time()) << endl;
	//		cout << "DENOMINATOR: " << ((double) (total_min * (double) g / (double) (initial_value - total_min) ) * time_per_node) << endl;

			double temp_beamsize = -1;

			if(g > 0 && (initial_value - total_min) > 0 && ((double) (total_min * (double) g / (double) (initial_value - total_min) ) * time_per_node) > 0)
			{
				temp_beamsize = ( (double) (timelimit - search_time()) / ((double) (total_min * (double) g / (double) (initial_value - total_min) ) * time_per_node) );
			}

			if(temp_beamsize > 1)
			{
				beamsize = (long) temp_beamsize;
			}

			if(temp_beamsize < 1 && temp_beamsize > 0)
			{
				beamsize = 1;
			}

			level_time.reset();

			cout << "level: " << g << " no nodes: " << queue.size() << " beam size: " << beamsize << " time level: " << time << " time node " << time_per_node << endl;
			last_level = g;
			expansions_level = 0;

			//keeping track of the progress per level
			if(!progress)
			{
				number_levels_without_progress++;

				//reached the limit of levels without progress -- moving to a more refined type system and restarting
				if(number_levels_without_progress > this->maxlevels)
				{
					this->lookahead = this->lookahead + 1;
					cout << "Moving to type system: " << this->lookahead << " and restarting." << endl;

					restart();

					number_levels_without_progress = 0;

					return IN_PROGRESS;
				}
			}
			else
			{
				number_levels_without_progress = 0;
			}

			jump();
			if(global_restart())
			{
				restart();
				return IN_PROGRESS;
			}

			progress = false;
		}
	}

	restart();

	return IN_PROGRESS;
}

void SS::report_progress()
{
	cout << "h_min: " << total_min << " depth: " << depth << " #states: " << queue.size() << " time: " << search_time << endl;
}

void SS::initialize() {
	cout << "SS ..." << endl;
	search_time.reset();
	level_time.reset();

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
	total_min = initial_value;

	// adding the initial state to open
	SSNode node(*g_initial_state, 1);

	/*
	 * Seeding the prediction with the children of the start state
	 *
	 */
	Type type = sampler->getType(node.state, initial_value, lookahead);

	type.setLevel( 0 );
	node.weight = 1;
	queue.insert( pair<Type, SSNode>( type, node ) );

	//open.insert(make_pair(initial_value, node));
	progress = true;
	cout << "Initial heuristic value: ";
	cout << initial_value << endl;
	depth = 0;
	report_progress();
	depth ++;
}

void SS::restart(){
	cout << "restarting at depth: " << depth << endl;
	total_min = initial_value;

	queue.clear();
	SSNode node(*g_initial_state, 1);
	Type type = sampler->getType(node.state, initial_value, lookahead);

	type.setLevel( 0 );
	node.weight = 1;
	queue.insert( pair<Type, SSNode>( type, node ) );

	//open.clear();
	//open.insert(make_pair(initial_value, node));
	depth = 0;
	progress = true;
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
	SS *engine = 0;
	if (!parser.dry_run()) {
		engine = new SS(opts);
	}
	return engine;
}


static Plugin<SearchEngine> _plugin("ss", _parse);
