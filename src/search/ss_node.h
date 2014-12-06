#ifndef SS_NODE_H
#define SS_NODE_H

#include "state.h"

class SSNode {
private:
	State state;
	int h_value;
	int g_value;
	int level;
public:
        SSNode();
	SSNode(State s, int h, int g, int l);
        State get_state();
        void set_state(State s);
        int get_h_value();
        int get_g_value();
        int get_level();
        void set_h_value(int h);
        void set_g_value(int g);
        void set_level(int l);
};
#endif
