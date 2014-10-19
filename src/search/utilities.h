#ifndef UTILITIES_H
#define UTILITIES_H

#include <cassert>
#include <ostream>
#include <utility>
#include <vector>
#include <tr1/functional>
#include <set>
#include <string>
#include "boost/dynamic_bitset.hpp"

extern void register_event_handlers();

extern int get_peak_memory_in_kb();
extern int get_memory_VmRSS() ;
extern int get_memory_VmHWM() ;
extern void print_peak_memory();
extern void assert_sorted_unique(const std::vector<int> &values);

namespace std {
template<class T>
ostream & operator<<(ostream &stream, const vector<T> &vec) {
    stream << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i != 0)
            stream << ", ";
        stream << vec[i];
    }
    stream << "]";
    return stream;
}
}

template<class Sequence>
size_t hash_number_sequence(Sequence data, size_t length) {
    // hash function adapted from Python's hash function for tuples.
    size_t hash_value = 0x345678;
    size_t mult = 1000003;
    for (int i = length - 1; i >= 0; --i) {
        hash_value = (hash_value ^ data[i]) * mult;
        mult += 82520 + i + i;
    }
    hash_value += 97531;
    return hash_value;
}

struct hash_int_pair {
    size_t operator()(const std::pair<int, int> &key) const {
        return size_t(key.first * 1337 + key.second);
    }
};

struct hash_pointer_pair {
    size_t operator()(const std::pair<void *, void *> &key) const {
        return size_t(size_t(key.first) * 1337 + size_t(key.second));
    }
};

class hash_pointer {
public:
    size_t operator()(const void *p) const {
        //return size_t(reinterpret_cast<int>(p));
        std::tr1::hash<const void *> my_hash_class;
        return my_hash_class(p);
    }
};

/*
  TODO: This is a mess. We're using different ways of defining hash
  functions here mostly for historical reasons. The use of hash_maps,
  hash_sets, unordered_maps and unordered_sets and the way of defining
  hash functions for them should be cleaned up globally.
*/

namespace std {
namespace tr1 {

template<>
struct hash<vector<int> > {
    std::size_t operator()(const std::vector<int> &key) const {
        return hash_number_sequence(key, key.size());
    }
};
}
}
void get_GA_patterns_from_file(std::vector<std::vector<int> > &pattern_col,bool disjoint,double mutation_rate);
double fRand(double fMin, double fMax);
bool compare_pairs_int_long (std::pair<int,long> i,std::pair<int,long> j);
bool compare_pairs_int_double (std::pair<int,double> i,std::pair<int,double> j);
bool compare_pairs_dynBitset_long (std::pair<boost::dynamic_bitset<>,long> i,std::pair<boost::dynamic_bitset<>,long> j) ;
bool compare_pairs_int_string (std::pair<int,std::string> i,std::pair<int,std::string> j) ;
void print_h_comb(boost::dynamic_bitset<> h_comb);
int grep_get_last_f_value();
int grep_chosen_heur_degree();
struct set_int_comp {
  bool operator() (const std::set<int>& a,const std::set<int>& b) const{
     return (a != b);
  }
};
void get_iPDB_patterns_from_file(std::vector<std::vector<int> > &all_pattern_col);
void print_strong_heurs(std::set<int> strong_heurs2);

#endif
