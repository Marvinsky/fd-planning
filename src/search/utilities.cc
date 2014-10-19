#include "utilities.h"
#include "globals.h"

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <unistd.h>
#include <string>
#include  <boost/lexical_cast.hpp>
using namespace std;

#ifdef __APPLE__
#include <mach/mach.h>
#endif

#ifdef __APPLE__
static void exit_handler();
#else
static void exit_handler(int exit_code, void *hint);
#endif

static void signal_handler(int signal_number);

void register_event_handlers() {
    // On exit or when receiving certain signals such as SIGINT (Ctrl-C),
    // print the peak memory usage.
#ifdef __APPLE__
    atexit(exit_handler);
#else
    on_exit(exit_handler, 0);
#endif
    signal(SIGABRT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGSEGV, signal_handler);
    signal(SIGINT, signal_handler);
}

#ifdef __APPLE__
void exit_handler() {
#else
void exit_handler(int, void *) {
#endif
    print_peak_memory();
}

void signal_handler(int signal_number) {
    // See glibc manual: "Handlers That Terminate the Process"
    static volatile sig_atomic_t handler_in_progress = 0;
    if (handler_in_progress)
        raise(signal_number);
    handler_in_progress = 1;
    print_peak_memory();
    cout << "caught signal " << signal_number << " -- exiting" << endl;
    signal(signal_number, SIG_DFL);
    raise(signal_number);
}

int get_peak_memory_in_kb() {
    // On error, produces a warning on cerr and returns -1.
    int memory_in_kb = -1;

#ifdef __APPLE__
    // Based on http://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
    task_basic_info t_info;
    mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;

    if (task_info(mach_task_self(), TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&t_info),
                  &t_info_count) == KERN_SUCCESS)
        memory_in_kb = t_info.virtual_size / 1024;
#else
    ostringstream filename_stream;
    filename_stream << "/proc/" << getpid() << "/status";
    const char *filename = filename_stream.str().c_str();

    ifstream procfile(filename);
    string word;
    while (procfile.good()) {
        procfile >> word;
        if (word == "VmPeak:") {
            procfile >> memory_in_kb;
            break;
        }
        // Skip to end of line.
        procfile.ignore(numeric_limits<streamsize>::max(), '\n');
    }
    if (procfile.fail())
        memory_in_kb = -1;
#endif

    if (memory_in_kb == -1)
        cerr << "warning: could not determine peak memory" << endl;
    return memory_in_kb;
}

void print_peak_memory() {
    cout << "Peak memory: " << get_peak_memory_in_kb() << " KB" << endl;
    cout << "VmRSS memory: " << get_memory_VmRSS() << " KB" << endl;
    cout << "VmHWM memory: " << get_memory_VmHWM()  << " KB" << endl;
}
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
bool compare_pairs_int_long (std::pair<int,long> i,std::pair<int,long> j) 
{ 
  return (i.second<j.second); 
}
bool compare_pairs_int_string (std::pair<int,string> i,std::pair<int,string> j) 
{ 
  return (i.second<j.second); 
}
bool compare_pairs_int_double (std::pair<int,double> i,std::pair<int,double> j) {
  return (i.second<j.second); 
}
bool compare_pairs_dynBitset_long (std::pair<boost::dynamic_bitset<>,long> i,std::pair<boost::dynamic_bitset<> ,long> j) 
{ 
  return (i.second<j.second); 
}
void print_h_comb(boost::dynamic_bitset<> h_comb){
  for (int i=0;i<h_comb.size();i++){
    if(h_comb.test(i)){
      cout<<i<<"-";
    }
  }
}
void print_strong_heurs(set<int> strong_heurs2){
  for (set<int>::iterator it= strong_heurs2.begin(); it != strong_heurs2.end(); it++){
      cout<<*it<<",";
  }
}
int grep_get_last_f_value()
{
    std::string line;
    ifstream in(log_file.c_str());
    int f_value=0;
    if( in.is_open())
    {
          while( getline(in,line) )
          {
              if( line.find("f =")!=string::npos ){
		cout<<"line:"<<line<<endl;
		unsigned found=line.find_first_of("123456789");
		unsigned found2 = line.find(" ",found+1);
		cout<<"found:"<<found<<",found2:"<<found2<<",f_boundary:";cout<<line.substr(found,found2-found)<<endl;
		string temp=line.substr(found,found2-found);
		f_value= atoi(temp.c_str());
	      }
          }
    }
    return f_value;
}
void get_GA_patterns_from_file(std::vector<std::vector<int> > &all_pattern_col,bool disjoint,double mutation_rate){
  all_pattern_col.clear();//just in case this was previously populated
  all_pattern_col.resize(1);//just in case this was previously populated
    std::string line;
    std::string temp;
    unsigned found2;
    unsigned next_pattern_pos;
    bool found_PDB=false;
    ifstream in(log_file.c_str());
    cout<<"log_file:"<<log_file<<",problem_name:"<<problem_name<<endl;
    std::string disjoint_pattern("disjoint_patterns:,");
    if(disjoint){
      disjoint_pattern+="1";
    }
    else{
      disjoint_pattern+="0";
    }
    
    cout<<disjoint_pattern<<endl;
    std::string mutation_rate_string("mutation_probability:,");
    //mutation_rate_string+=boost::lexical_cast<std::string>(mutation_rate);
    //mutation_rate_string+=std::to_string(mutation_rate);
    std::ostringstream strs;
    strs << mutation_rate;
    std::string str = strs.str();
    mutation_rate_string+=str;
    mutation_rate_string+=",";
    cout<<mutation_rate_string<<endl;

    if( in.is_open())
    {
          while( getline(in,line) ){
	    if( line.find(problem_name)!=string::npos&&line.find(disjoint_pattern)!=string::npos
		&&line.find(mutation_rate_string)!=string::npos){
	      cout<<"line:"<<line<<endl;
	      found_PDB=true;
		
	      unsigned current_pos=line.find("]");
	      int num_databases = std::count(line.begin(), line.end(), ']') - 1;//the first ] is for the heuristic number
	      cout<<"num_databases:"<<num_databases<<endl;
	      all_pattern_col.resize(num_databases);


	      current_pos=line.find("[",current_pos+1);
	      //unsigned found2 = line.find("[",found+1);
	      next_pattern_pos=line.find("-",current_pos+1);
	      for(int i=0;i<num_databases;i++){
		cout<<"reading database"<<i<<endl;
		while(next_pattern_pos>line.find_first_of(",]",current_pos+1)){
		//while(true)
		  current_pos=line.find_first_of("0123456789",current_pos);//so it points to the next variable
		  if(current_pos>next_pattern_pos){
		    //cout<<"skipping empty database"<<endl;
		    while(current_pos>next_pattern_pos){
		      next_pattern_pos=line.find("-",next_pattern_pos+1);
		      cout<<"skipped database "<<i<<" because it is empty"<<endl;
		      //cout<<all_pattern_col.at(i);cout<<endl;
		      i++;
		    }
		    i--;//need to decrease i by one or it will be one ahead by the for statement
		    break;
		  }
		  //cout<<"\tcurrent_pos:"<<current_pos;
		  found2 = line.find_first_of(",]",current_pos);
		  //cout<<",found2:"<<found2;
		  if(line.find_first_of(",]",current_pos)==string::npos){
		    //cout<<",finished with pattern:"<<i<<",string finished"<<endl;
		    break;
		  }
		  temp=line.substr(current_pos,found2-current_pos);
		  //cout<<",next var:"<<temp<<",";
		  int temp2=boost::lexical_cast<int>(temp);
		  all_pattern_col.at(i).push_back(temp2);
		  // cout<<",last int added:"<<all_pattern_col.at(i).back();
		  current_pos=found2;//current_pos not pointing to next , or ]
		  if(line.find_first_of(",",found2)>next_pattern_pos){
		    //cout<<",finished with pattern:"<<i<<",pattern finished"<<endl;
		    current_pos=line.find("[",next_pattern_pos);
		    next_pattern_pos=line.find("-",next_pattern_pos+1);
		    //cout<<"next_pattern_pos:"<<next_pattern_pos<<endl;
		    break;
		  }
		}
		cout<<"database:"<<i<<",read pattern:";
		cout<<all_pattern_col.at(i);cout<<endl;
	      } 
	      //Now add to our time calculations how long did iPDB originally take to generate this PDBs
	      current_pos=line.find("time:");
	      current_pos=line.find_first_of("0123456789",current_pos);
	      temp=line.substr(current_pos);
	      double temp2=boost::lexical_cast<double>(temp);
	      overall_original_pdbs_time+=temp2;
	      cout<<"Original GAPDB time:"<<temp2<<mutation_rate_string<<disjoint_pattern<<",Original_pdbs_time:"<<overall_original_pdbs_time<<endl;
	    }
	  }
    }
    if(!found_PDB){
      cout<<"Killed, no existing gaPDB to read for current problem:"<<endl;
      exit(0);
    }
}

int get_memory_VmRSS() {
    // On error, produces a warning on cerr and returns -1.
    int memory_in_kb = -1;

#ifdef __APPLE__
    // Based on http://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
    task_basic_info t_info;
    mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;

    if (task_info(mach_task_self(), TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&t_info),
                  &t_info_count) == KERN_SUCCESS)
        memory_in_kb = t_info.virtual_size / 1024;
#else
    ostringstream filename_stream;
    filename_stream << "/proc/" << getpid() << "/status";
    const char *filename = filename_stream.str().c_str();

    ifstream procfile(filename);
    string word;
    while (procfile.good()) {
        procfile >> word;
        if (word == "VmRSS:") {
            procfile >> memory_in_kb;
            break;
        }
        // Skip to end of line.
        procfile.ignore(numeric_limits<streamsize>::max(), '\n');
    }
    if (procfile.fail())
        memory_in_kb = -1;
#endif

    if (memory_in_kb == -1)
        cerr << "warning: could not determine VmRSS memory" << endl;
    return memory_in_kb;
}
int get_memory_VmHWM() {
    // On error, produces a warning on cerr and returns -1.
    int memory_in_kb = -1;

#ifdef __APPLE__
    // Based on http://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
    task_basic_info t_info;
    mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;

    if (task_info(mach_task_self(), TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&t_info),
                  &t_info_count) == KERN_SUCCESS)
        memory_in_kb = t_info.virtual_size / 1024;
#else
    ostringstream filename_stream;
    filename_stream << "/proc/" << getpid() << "/status";
    const char *filename = filename_stream.str().c_str();

    ifstream procfile(filename);
    string word;
    while (procfile.good()) {
        procfile >> word;
        if (word == "VmHWM:") {
            procfile >> memory_in_kb;
            break;
        }
        // Skip to end of line.
        procfile.ignore(numeric_limits<streamsize>::max(), '\n');
    }
    if (procfile.fail())
        memory_in_kb = -1;
#endif

    if (memory_in_kb == -1)
        cerr << "warning: could not determine VmRSS memory" << endl;
    return memory_in_kb;
}

void assert_sorted_unique(const std::vector<int> &values) {
    for (size_t i = 1; i < values.size(); ++i) {
        assert(values[i - 1] < values[i]);
    }
}

void get_iPDB_patterns_from_file(std::vector<std::vector<int> > &all_pattern_col){
  all_pattern_col.clear();//just in case this was previously populated
  all_pattern_col.resize(1);//just in case this was previously populated
    std::string line;
    std::string temp;
    unsigned found2;
    unsigned next_pattern_pos;
    ifstream in(log_file.c_str());
    cout<<"log_file:"<<log_file<<",problem_name:"<<problem_name<<endl;
    bool found_PDB=false;

    if( in.is_open())
    {
          while( getline(in,line) ){
	    if( line.find(problem_name)!=string::npos&&line.find("iPDB")!=string::npos){
	      cout<<"line:"<<line<<endl;
	      found_PDB=true;
		
	      unsigned current_pos=line.find("]");
	      int num_databases = std::count(line.begin(), line.end(), ']') - 1;//the first ] is for the heuristic number
	      cout<<"num_databases:"<<num_databases<<endl;
	      all_pattern_col.resize(num_databases);


	      current_pos=line.find("[",current_pos+1);
	      //unsigned found2 = line.find("[",found+1);
	      next_pattern_pos=line.find("-",current_pos+1);
	      for(int i=0;i<num_databases;i++){
		cout<<"reading database"<<i<<endl;
		while(next_pattern_pos>line.find_first_of(",]",current_pos+1)){
		//while(true)
		  current_pos=line.find_first_of("0123456789",current_pos);//so it points to the next variable
		  if(current_pos>next_pattern_pos){
		    //cout<<"skipping empty database"<<endl;
		    while(current_pos>next_pattern_pos){
		      next_pattern_pos=line.find("-",next_pattern_pos+1);
		      cout<<"skipped database "<<i<<" because it is empty"<<endl;
		      //cout<<all_pattern_col.at(i);cout<<endl;
		      i++;
		    }
		    i--;//need to decrease i by one or it will be one ahead by the for statement
		    break;
		  }
		  //cout<<"\tcurrent_pos:"<<current_pos;
		  found2 = line.find_first_of(",]",current_pos);
		  //cout<<",found2:"<<found2;
		  if(line.find_first_of(",]",current_pos)==string::npos){
		    //cout<<",finished with pattern:"<<i<<",string finished"<<endl;
		    break;
		  }
		  temp=line.substr(current_pos,found2-current_pos);
		  //cout<<",next var:"<<temp<<",";
		  int temp2=boost::lexical_cast<int>(temp);
		  all_pattern_col.at(i).push_back(temp2);
		  // cout<<",last int added:"<<all_pattern_col.at(i).back();
		  current_pos=found2;//current_pos not pointing to next , or ]
		  if(line.find_first_of(",",found2)>next_pattern_pos){
		    //cout<<",finished with pattern:"<<i<<",pattern finished"<<endl;
		    current_pos=line.find("[",next_pattern_pos);
		    next_pattern_pos=line.find("-",next_pattern_pos+1);
		    //cout<<"next_pattern_pos:"<<next_pattern_pos<<endl;
		    break;
		  }
		}
		cout<<"read pattern:";
		cout<<all_pattern_col.at(i);cout<<endl;
	      }
	      //Now add to our time calculations how long did iPDB originally take to generate this PDBs
	      current_pos=line.find("time:");
	      current_pos=line.find_first_of("0123456789",current_pos);
	      temp=line.substr(current_pos);
	      double temp2=boost::lexical_cast<double>(temp);
	      overall_original_pdbs_time+=temp2;
	      cout<<"Original iPDB time:"<<temp2<<",Original_pdbs_time:"<<overall_original_pdbs_time<<endl;
	    }
	  }
    } 
    if(!found_PDB){
      cout<<"Killed, no existing iPDB to read for current problem:"<<endl;
      exit(0);
    }
}
int grep_chosen_heur_degree(){
  std::string line;
  int max_degree=0;
  bool chosen_heuristic_found=false;
    ifstream in(log_file.c_str());
    if( in.is_open())
    {
          while( getline(in,line) )
          {
              if( line.find("chosen_heuristic")!=string::npos ){
		chosen_heuristic_found=true;
		cout<<"line:"<<line<<endl;
		//unsigned found=line.find("chosen_heuristic");
		unsigned found=line.find_last_of(":");
		//unsigned found2 = line.find(" ",found+1);
		cout<<"found:"<<found<<endl;cout<<line.substr(found+1)<<endl;
		string temp=line.substr(found+1);
		max_degree=max(max_degree,atoi(temp.c_str()));
	      }
          }
    }
    else{
      cout<<"cant open "<<log_file.c_str()<<endl;
      exit(0);
    }
    if(!chosen_heuristic_found){
      cout<<"chosen_heuristic not found"<<endl;
      exit(0);
    }
    cout<<"max_degree:"<<max_degree<<endl;
    return max_degree;
}
