#include "ss_node.h"

SSNode::SSNode() {

}

SSNode::SSNode(State s, int h, int g, int l) {
	this->state = s;
	this->h_value = h;
	this->g_value = g;
	this->level = l;
}

State SSNode::get_state() {
	return this->state;
}

void SSNode::set_state(State s) {
	this->state = s;
}

int SSNode::get_h_value() {
	return this->h_value;
}

int SSNode::get_g_value() {
	return this->h_value;
}

int SSNode::get_level() {
	return level;
}

void SSNode::set_h_value(int h) {
	this->h_value = h;
}

void SSNode::set_g_value(int g) {
	this->g_value = g;
}

void SSNode::set_level(int l) {
	this->level = l;
}


