
#include "type_system.h"
#include "globals.h"
#include "option_parser.h"
#include "successor_generator.h"
#include "plugin.h"
#include "rng.h"

#include <vector>

TypeSystem::TypeSystem(Heuristic* heuristic)
{
	this->heuristic = heuristic;
}

TypeSystem::~TypeSystem()
{

}

void TypeSystem::sample(State state, int parent_heuristic, TypeChildren& type_children, int type, int current_level)
{
	if(current_level == type)
	{
		return;
	}

	std::vector<const Operator*> applicable_ops;
	g_successor_generator->generate_applicable_ops(state, applicable_ops);
	for (int i = 0; i < applicable_ops.size(); ++i)
	{
		State child(state, *applicable_ops[i]);

		heuristic->evaluate(child);

		int h = -1;

		if(!heuristic->is_dead_end())
		{
			h = heuristic->get_heuristic();

			if(best_h == -1 || h < best_h)
			{
				best_h = h;
			}

			TypeChild c(current_level, parent_heuristic, h);
			type_children.addTypeChild(c);
		}
		else
		{
			TypeChild c(current_level, parent_heuristic, h);
			type_children.addTypeChild(c);
		}

		sample(child, h, type_children, type, current_level + 1);
	}
}

Type TypeSystem::getType(State state, int h, int type)
{
	heuristic->evaluate(state);

	best_h = h;

	TypeChildren type_children;
	Type obj(-1, h);

	sample(state, h, type_children, type, 0);

	obj.setBestH(best_h);

	obj.setChildren(type_children);

	return obj;
}
