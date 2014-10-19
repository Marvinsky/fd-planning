#!/bin/bash
./downward-1 --Phase "SOLVING" --search "astar(max([\
    hmax(),\
    incremental_lmcut(reevaluate_parent=true),\
    ipdb(time_limit=120),\
    pdb_lp(systematic=2,time_limit=300),\
    pdb_lp(systematic=3,time_limit=60),\
    pdb_lp(systematic=4,time_limit=60),\
    pdb_lp(systematic=5,time_limit=60),\
    pdb_lp(systematic=6,time_limit=60),\
    pdb_lp(systematic=7,time_limit=60),\
    gapdb(mutation_probability=0.01,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.05,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.10,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.15,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.20,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.25,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.30,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.35,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.40,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.45,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.50,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.55,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.60,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.65,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.70,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.75,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.80,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.85,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.90,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.95,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=1.00,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.01,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.05,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.10,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.15,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.20,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.25,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.30,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.35,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.40,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.45,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.50,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.55,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.60,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.65,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.70,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.75,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.80,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.85,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.90,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=0.95,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    gapdb(mutation_probability=1.00,disjoint=true,pdb_max_size=200000,num_episodes=30,num_collections=5),\
    ]),mark_children_as_finished=true)"
