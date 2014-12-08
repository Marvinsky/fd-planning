#ifndef NODE_PROXY_H
#define NODE_PROXY_H

#include "node2.h"

#include <iostream>
#include <ext/hash_map>
#include <functional>
#include <boost/functional/hash.hpp>
#include <cassert>

using namespace std;

class NodeProxy {
public:
        
	mutable Node2 *node;
        
        NodeProxy() {
	   node = new Node2();
        }
	
	NodeProxy(Node2 *node_) {
           node = node_;
        }     
        
        explicit NodeProxy(const Node2 *node2) {
           node = node2;
        }
        /*
        const NodeProxy &operator=(const NodeProxy &other) const {
	   node = other.node;
	   return *this;
        }
        */

        bool operator==(const NodeProxy &n1) const {
	   if ((n1.node->getF() == node->getF()) && (n1.node->getL() == node->getL())) {
               return true;
           }
           return false;
        }

        /*
	void make_permanent() const {
	   Node2 *new_buffer = new Node2();
	   node = new_buffer;
	}*/
};


namespace __gnu_cxx {
template<>
struct hash<NodeProxy> {
	size_t operator()(const NodeProxy &node_proxy) const {
	    std::size_t seed = 0;
	    boost::hash_combine(seed, node_proxy.node->getF());
            boost::hash_combine(seed, node_proxy.node->getL());
	    return seed;
	}
};
}
#endif 
