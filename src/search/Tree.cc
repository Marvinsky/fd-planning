#include "Tree.h"
#include <iostream>
#include <fstream>
#include "rpoly.h"
//#include "output.h"
#include "relax.h"
#include "limits.h"
#include <algorithm>
#include "utilities.h"
#include <boost/lexical_cast.hpp>
#include "boost/dynamic_bitset.hpp"

using namespace std;

unsigned int databases;
/*extern void result_to_dest( state_var_t *dest, const state_var_t *source, int action );
extern void source_to_dest( state_var_t *dest, state_var_t source );
extern void generate_state_id(const state_var_t *current_state,string *state_id,int start, int end);
extern void generate_state_id_tiles_grouped(const state_var_t* current_state,string *state_id,const vector <int> * tiles_grouped);
//extern void generate_state_id_database(const state_var_t* current_state,vector<int> *state_id,const vector <int> * tiles_grouped);
//extern void blank_facts(int tile_order[MAX_TILE],vector<int> *tiles_to_blank,const state_var_t * S, state_var_t * C1);
extern int find_blank_x1_y1();
extern int do_IDA_heuristic( const state_var_t* current_state,vector<int> * tiles_grouped );*/
/*extern void result_to_dest( state_var_t *dest, const state_var_t *source, int action );
extern void source_to_dest( state_var_t *dest, state_var_t source );
extern void copy_1P_and_AH_recursive(void);
extern void restore_1P_and_AH_recursive(void);
extern int get_1P_and_AH( state_var_t S );
extern void build_fixpoint_IDA( state_var_t S );
*/
/*state_var_t ginitial_state;
state_var_t ggoal_state;*/
 

void binary(int number) {
	static int remainder;

	if(number <= 1) {
		cout << number;
		return;
	}

	remainder = number%2;
	binary(number >> 1);    
	cout << remainder;
}
void create_emergency_strong_heur_call(set<int> strong_heurs,vector<Heuristic *> heuristics){
  cout<<"strong_heur_size:"<<strong_heurs.size()<<",heuristics.size:"<<heuristics.size()<<endl;
  ofstream outputFile("RIDA_strong_heurs.sh");
  //outputFile<<"src/search/downward --plan-file $1 --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
  outputFile<<"../../downward --plan-file $1 --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
      
  bool incremental_lmcut_present=false;
  set<int>::iterator last_element; last_element=strong_heurs.end(); --last_element;
  for(set<int>::iterator iter_strong=strong_heurs.begin();iter_strong!=strong_heurs.end();iter_strong++){
    cout<<"iter_strong:"<<*iter_strong;
    string heur_name=heuristics.at(*iter_strong)->get_heur_name();
    cout<<"heuristic:"<<heur_name<<endl;
    if((heur_name.find("blind")!=string::npos)){
      cout<<"skipping blind heuristic"<<endl;
      continue;
    }
    
    if(heuristics.at(*iter_strong)->get_heur_name()=="incremental_lmcut"){
      incremental_lmcut_present=true;
    }
  
    if(iter_strong!=last_element){
      outputFile<<heuristics.at(*iter_strong)->get_heur_call_name()<<",\\"<<endl;
    }
    else{
      if(incremental_lmcut_present){
	outputFile<<heuristics.at(*iter_strong)->get_heur_call_name()<<"]),mark_children_as_finished=true)\""<<endl;
      }
      else{
	outputFile<<heuristics.at(*iter_strong)->get_heur_call_name()<<"]))\""<<endl;
      }
    }
  }
  outputFile.close();
}
void HST::annotate_sampling_data(vector<Heuristic *> orig_heuristics,vector<Heuristic *> heuristics){
  cout<<"outputing sampling time:"<<endl;fflush(stdout);	
  ofstream outputFile;
  outputFile.open("sampling_times.txt",ios::app);
  //outputFile<<"src/search/downward --plan-file $1 --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
  outputFile<<g_plan_filename<<"\t"<<g_timer()<<"\t"<<calculate_time_costs_specific(best_h_comb,orig_heuristics)<<"\t"<<node_gen_and_exp_cost<<endl;
  ofstream outputFile2;
  outputFile2.open("remaining_heurs.txt",ios::app);
  outputFile<<g_plan_filename;
  ofstream outputFile3;
  outputFile3.open("selected_heurs.txt",ios::app);
  outputFile3<<g_plan_filename;
  vector<bool> remaining_heur_index(51,false);
  vector<bool> selected_heurs(51,false);
  for (int i=0;i<heuristics.size();i++){//annotating selected heuristics
    string heur_name=heuristics.at(i)->get_heur_name();
    cout<<"selected heur:"<<heur_name<<endl;fflush(stdout);
    if((heur_name.find("blind")!=string::npos)){
	selected_heurs.at(0)=true;
    }
    //NEED TO ADD DIFFERENT NAMING IF 2 versions of INCREMENTAL are used (Loc or Full Frontier)
    else if((heur_name.find("incremental_lmcut")!=string::npos)){
	selected_heurs.at(1)=true;
    }
    else if((heur_name.find("IPDB")!=string::npos)){
	selected_heurs.at(2)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:2")!=string::npos)){
	selected_heurs.at(3)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:3")!=string::npos)){
	selected_heurs.at(4)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:4")!=string::npos)){
	selected_heurs.at(5)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:5")!=string::npos)){
	selected_heurs.at(6)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:6")!=string::npos)){
	selected_heurs.at(7)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:7")!=string::npos)){
	selected_heurs.at(8)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.05,without")!=string::npos)){
	selected_heurs.at(9)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.10,without")!=string::npos)){
	selected_heurs.at(10)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.15,without")!=string::npos)){
	selected_heurs.at(11)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.20,without")!=string::npos)){
	selected_heurs.at(12)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.25,without")!=string::npos)){
	selected_heurs.at(13)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.30,without")!=string::npos)){
	selected_heurs.at(14)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.35,without")!=string::npos)){
	selected_heurs.at(15)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.40,without")!=string::npos)){
	selected_heurs.at(16)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.45,without")!=string::npos)){
	selected_heurs.at(17)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.50,without")!=string::npos)){
	selected_heurs.at(18)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.55,without")!=string::npos)){
	selected_heurs.at(19)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.60,without")!=string::npos)){
	selected_heurs.at(20)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.65,without")!=string::npos)){
	selected_heurs.at(21)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.70,without")!=string::npos)){
	selected_heurs.at(22)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.75,without")!=string::npos)){
	selected_heurs.at(23)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.80,without")!=string::npos)){
	selected_heurs.at(24)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.85,without")!=string::npos)){
	selected_heurs.at(25)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.90,without")!=string::npos)){
	selected_heurs.at(26)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.95,without")!=string::npos)){
	selected_heurs.at(27)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:1.00,without")!=string::npos)){
	selected_heurs.at(28)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.05,with disjoint")!=string::npos)){
	selected_heurs.at(29)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.10,with disjoint")!=string::npos)){
	selected_heurs.at(30)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.15,with disjoint")!=string::npos)){
	selected_heurs.at(31)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.20,with disjoint")!=string::npos)){
	selected_heurs.at(32)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.25,with disjoint")!=string::npos)){
	selected_heurs.at(33)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.30,with disjoint")!=string::npos)){
	selected_heurs.at(34)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.35,with disjoint")!=string::npos)){
	selected_heurs.at(35)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.40,with disjoint")!=string::npos)){
	selected_heurs.at(36)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.45,with disjoint")!=string::npos)){
	selected_heurs.at(37)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.50,with disjoint")!=string::npos)){
	selected_heurs.at(38)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.55,with disjoint")!=string::npos)){
	selected_heurs.at(39)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.60,with disjoint")!=string::npos)){
	selected_heurs.at(40)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.65,with disjoint")!=string::npos)){
	selected_heurs.at(41)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.70,with disjoint")!=string::npos)){
	selected_heurs.at(42)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.75,with disjoint")!=string::npos)){
	selected_heurs.at(43)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.80,with disjoint")!=string::npos)){
	selected_heurs.at(44)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.85,with disjoint")!=string::npos)){
	selected_heurs.at(45)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.90,with disjoint")!=string::npos)){
	selected_heurs.at(46)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.95,with disjoint")!=string::npos)){
	selected_heurs.at(47)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:1.00,with disjoint")!=string::npos)){
	selected_heurs.at(48)=true;
    }
    else if((heur_name.find("hmax")!=string::npos)){
	selected_heurs.at(49)=true;
    }
  }

  for (int i=0;i<orig_heuristics.size();i++){
     if(!orig_heuristics.at(i)->is_using()){
       continue;//this heuristic was not available for choosing
     }
    string heur_name=orig_heuristics.at(i)->get_heur_name();
    cout<<"remaining heur:"<<heur_name<<endl;fflush(stdout);
    if((heur_name.find("blind")!=string::npos)){
      remaining_heur_index.at(0)=true;
    }
    //NEED TO ADD DIFFERENT NAMING IF 2 versions of INCREMENTAL are used (Loc or Full Frontier)
    else if((heur_name.find("incremental_lmcut")!=string::npos)){
      remaining_heur_index.at(1)=true;
    }
    else if((heur_name.find("IPDB")!=string::npos)){
      remaining_heur_index.at(2)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:2")!=string::npos)){
      remaining_heur_index.at(3)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:3")!=string::npos)){
      remaining_heur_index.at(4)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:4")!=string::npos)){
      remaining_heur_index.at(5)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:5")!=string::npos)){
      remaining_heur_index.at(6)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:6")!=string::npos)){
      remaining_heur_index.at(7)=true;
    }
    else if((heur_name.find("lp_pdb,systematic:7")!=string::npos)){
      remaining_heur_index.at(8)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.05,without")!=string::npos)){
      remaining_heur_index.at(9)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.10,without")!=string::npos)){
      remaining_heur_index.at(10)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.15,without")!=string::npos)){
      remaining_heur_index.at(11)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.20,without")!=string::npos)){
      remaining_heur_index.at(12)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.25,without")!=string::npos)){
      remaining_heur_index.at(13)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.30,without")!=string::npos)){
      remaining_heur_index.at(14)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.35,without")!=string::npos)){
      remaining_heur_index.at(15)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.40,without")!=string::npos)){
      remaining_heur_index.at(16)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.45,without")!=string::npos)){
      remaining_heur_index.at(17)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.50,without")!=string::npos)){
      remaining_heur_index.at(18)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.55,without")!=string::npos)){
      remaining_heur_index.at(19)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.60,without")!=string::npos)){
      remaining_heur_index.at(20)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.65,without")!=string::npos)){
      remaining_heur_index.at(21)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.70,without")!=string::npos)){
      remaining_heur_index.at(22)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.75,without")!=string::npos)){
      remaining_heur_index.at(23)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.80,without")!=string::npos)){
      remaining_heur_index.at(24)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.85,without")!=string::npos)){
      remaining_heur_index.at(25)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.90,without")!=string::npos)){
      remaining_heur_index.at(26)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.95,without")!=string::npos)){
      remaining_heur_index.at(27)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:1.00,without")!=string::npos)){
      remaining_heur_index.at(28)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.05,with disjoint")!=string::npos)){
      remaining_heur_index.at(29)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.10,with disjoint")!=string::npos)){
      remaining_heur_index.at(30)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.15,with disjoint")!=string::npos)){
      remaining_heur_index.at(31)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.20,with disjoint")!=string::npos)){
      remaining_heur_index.at(32)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.25,with disjoint")!=string::npos)){
      remaining_heur_index.at(33)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.30,with disjoint")!=string::npos)){
      remaining_heur_index.at(34)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.35,with disjoint")!=string::npos)){
      remaining_heur_index.at(35)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.40,with disjoint")!=string::npos)){
      remaining_heur_index.at(36)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.45,with disjoint")!=string::npos)){
      remaining_heur_index.at(37)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.50,with disjoint")!=string::npos)){
      remaining_heur_index.at(38)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.55,with disjoint")!=string::npos)){
      remaining_heur_index.at(39)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.60,with disjoint")!=string::npos)){
      remaining_heur_index.at(40)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.65,with disjoint")!=string::npos)){
      remaining_heur_index.at(41)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.70,with disjoint")!=string::npos)){
      remaining_heur_index.at(42)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.75,with disjoint")!=string::npos)){
      remaining_heur_index.at(43)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.80,with disjoint")!=string::npos)){
      remaining_heur_index.at(44)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.85,with disjoint")!=string::npos)){
      remaining_heur_index.at(45)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.90,with disjoint")!=string::npos)){
      remaining_heur_index.at(46)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:0.95,with disjoint")!=string::npos)){
      remaining_heur_index.at(47)=true;
    }
    else if((heur_name.find("GAPDB,mutation_rate:1.00,with disjoint")!=string::npos)){
      remaining_heur_index.at(48)=true;
    }
    else if((heur_name.find("hmax")!=string::npos)){
      remaining_heur_index.at(49)=true;
    }
  }
  for (int i=0;i<remaining_heur_index.size();i++){
    outputFile2<<"\t"<<remaining_heur_index.at(i);
  }
  for (int i=0;i<selected_heurs.size();i++){
    outputFile3<<"\t"<<selected_heurs.at(i);
  }
  outputFile2<<endl; outputFile3<<endl;
  outputFile2.close(); outputFile3.close();
  ofstream outputFile4;
  outputFile4.open("sampling_accuracy.txt",ios::ate);
  outputFile4<<endl<<g_plan_filename<<"\t"<<get_current_F_bound()<<"\t"<<F_boundary_size<<"\t"<<Pred_Asymptotic_HBF<<"\t"<<Pred_Extra_Nodes;
  outputFile4.close();
}
void HST::annotate_sampling_preds(vector<Heuristic *> orig_heuristics){
  vector<Heuristic *> heuristics;

  cout<<"outputing sampling preds:"<<endl;fflush(stdout);	
  ofstream outputFile3;
  outputFile3.open("sampling_preds.txt",ios::app);
  //outputFile<<"src/search/downward --plan-file $1 --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
  outputFile3<<g_plan_filename;
  vector<bool> remaining_heur_index(51,false);
  vector<bool> selected_heurs(51,false);
  for (unsigned level=0;level<Degree;level++){
    for(unsigned h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
      boost::dynamic_bitset<> current_comb=h_comb_map2[level][h_index];
      heuristics.clear();
      selected_heurs.assign(51,false);
      for (int i=0;i<current_comb.size();i++){
	if(current_comb.test(i)){
	  heuristics.push_back(orig_heuristics.at(i));
	}
      }

      for (int i=0;i<heuristics.size();i++){//annotating selected heuristics
	string heur_name=heuristics.at(i)->get_heur_name();
	//cout<<"selected heur:"<<heur_name<<endl;fflush(stdout);
	if((heur_name.find("blind")!=string::npos)){
	    selected_heurs.at(0)=true;
	}
	//NEED TO ADD DIFFERENT NAMING IF 2 versions of INCREMENTAL are used (Loc or Full Frontier)
	else if((heur_name.find("incremental_lmcut")!=string::npos)){
	    selected_heurs.at(1)=true;
	}
	else if((heur_name.find("IPDB")!=string::npos)){
	    selected_heurs.at(2)=true;
	}
	else if((heur_name.find("lp_pdb,systematic:2")!=string::npos)){
	    selected_heurs.at(3)=true;
	}
	else if((heur_name.find("lp_pdb,systematic:3")!=string::npos)){
	    selected_heurs.at(4)=true;
	}
	else if((heur_name.find("lp_pdb,systematic:4")!=string::npos)){
	    selected_heurs.at(5)=true;
	}
	else if((heur_name.find("lp_pdb,systematic:5")!=string::npos)){
	    selected_heurs.at(6)=true;
	}
	else if((heur_name.find("lp_pdb,systematic:6")!=string::npos)){
	    selected_heurs.at(7)=true;
	}
	else if((heur_name.find("lp_pdb,systematic:7")!=string::npos)){
	    selected_heurs.at(8)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.05,without")!=string::npos)){
	    selected_heurs.at(9)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.10,without")!=string::npos)){
	    selected_heurs.at(10)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.15,without")!=string::npos)){
	    selected_heurs.at(11)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.20,without")!=string::npos)){
	    selected_heurs.at(12)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.25,without")!=string::npos)){
	    selected_heurs.at(13)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.30,without")!=string::npos)){
	    selected_heurs.at(14)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.35,without")!=string::npos)){
	    selected_heurs.at(15)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.40,without")!=string::npos)){
	    selected_heurs.at(16)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.45,without")!=string::npos)){
	    selected_heurs.at(17)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.50,without")!=string::npos)){
	    selected_heurs.at(18)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.55,without")!=string::npos)){
	    selected_heurs.at(19)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.60,without")!=string::npos)){
	    selected_heurs.at(20)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.65,without")!=string::npos)){
	    selected_heurs.at(21)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.70,without")!=string::npos)){
	    selected_heurs.at(22)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.75,without")!=string::npos)){
	    selected_heurs.at(23)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.80,without")!=string::npos)){
	    selected_heurs.at(24)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.85,without")!=string::npos)){
	    selected_heurs.at(25)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.90,without")!=string::npos)){
	    selected_heurs.at(26)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.95,without")!=string::npos)){
	    selected_heurs.at(27)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:1.00,without")!=string::npos)){
	    selected_heurs.at(28)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.05,with disjoint")!=string::npos)){
	    selected_heurs.at(29)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.10,with disjoint")!=string::npos)){
	    selected_heurs.at(30)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.15,with disjoint")!=string::npos)){
	    selected_heurs.at(31)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.20,with disjoint")!=string::npos)){
	    selected_heurs.at(32)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.25,with disjoint")!=string::npos)){
	    selected_heurs.at(33)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.30,with disjoint")!=string::npos)){
	    selected_heurs.at(34)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.35,with disjoint")!=string::npos)){
	    selected_heurs.at(35)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.40,with disjoint")!=string::npos)){
	    selected_heurs.at(36)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.45,with disjoint")!=string::npos)){
	    selected_heurs.at(37)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.50,with disjoint")!=string::npos)){
	    selected_heurs.at(38)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.55,with disjoint")!=string::npos)){
	    selected_heurs.at(39)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.60,with disjoint")!=string::npos)){
	    selected_heurs.at(40)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.65,with disjoint")!=string::npos)){
	    selected_heurs.at(41)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.70,with disjoint")!=string::npos)){
	    selected_heurs.at(42)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.75,with disjoint")!=string::npos)){
	    selected_heurs.at(43)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.80,with disjoint")!=string::npos)){
	    selected_heurs.at(44)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.85,with disjoint")!=string::npos)){
	    selected_heurs.at(45)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.90,with disjoint")!=string::npos)){
	    selected_heurs.at(46)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:0.95,with disjoint")!=string::npos)){
	    selected_heurs.at(47)=true;
	}
	else if((heur_name.find("GAPDB,mutation_rate:1.00,with disjoint")!=string::npos)){
	    selected_heurs.at(48)=true;
	}
	else if((heur_name.find("hmax")!=string::npos)){
	    selected_heurs.at(49)=true;
	}
      }
      for (int i=0;i<selected_heurs.size();i++){
	outputFile3<<"\t"<<selected_heurs.at(i);
      }
      outputFile3<<"\t"<<h_comb_to_degree.at(level).at(h_index)<<"\t"<<calculate_time_costs_specific(current_comb,orig_heuristics)<<"\t"<<double(h_comb_to_degree.at(level).at(h_index))*calculate_time_costs_specific(current_comb,orig_heuristics)<<endl;
    }
  }
  outputFile3.close();
}
void HST::create_selected_heur_call(vector<Heuristic *> heuristics){
  cout<<"selected heur size:"<<heuristics.size()<<endl;
  ofstream outputFile("RIDA_selected_heurs.sh");
  //outputFile<<"src/search/downward --plan-file $1 --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
  if(common_sampling_F_boundary==0){
    outputFile<<"../../downward --print_F_bound_data "<<get_current_F_bound()<<" --plan-file $1 --use_saved_pdbs --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
  }
  else{
    outputFile<<"../../downward --print_F_bound_data "<<get_current_F_bound()<<" --common_sampling_F_boundary "<<common_sampling_F_boundary<<" --plan-file $1 --use_saved_pdbs --Phase \"SOLVING\" --search \"astar(max([\\"<<endl;
  }
      
  bool incremental_lmcut_present=false;
  for(int i=0;i<heuristics.size();i++){
    if(heuristics.at(i)->get_heur_name()=="incremental_lmcut"){
      incremental_lmcut_present=true;
    }
  
    if(i<(heuristics.size()-1)){
      outputFile<<heuristics.at(i)->get_heur_call_name()<<",\\"<<endl;
    }
    else{
      if(incremental_lmcut_present){
	outputFile<<heuristics.at(i)->get_heur_call_name()<<"]),mark_children_as_finished=true)\""<<endl;
      }
      else{
	outputFile<<heuristics.at(i)->get_heur_call_name()<<"]))\""<<endl;
      }
    }
  }
  outputFile.close();
}
double EXPANSION_TIME=1.9;
double HEUR_TIME=6.29;
#define MAX_LENGTH 256
void generate_state_id_database(){
  cout<<"Use FD-equivalent for generate_state_id_database!"<<endl;exit(0);
} 
void print(boost::dynamic_bitset<> subset,bool newline){
  for(unsigned i=0;i<subset.size();i++){
    if(subset.test(i)){
      cout<<i<<",";
    }
  }
  if(newline){
    cout<<endl;
  }
}


/*****************************************************************************/
/**************************** HUSTSearchNode *************************************/
/*****************************************************************************/
HUSTSearchNode::HUSTSearchNode(const State *_S,const Operator *_op, int _depth, int _H){
//  cout<<"calling HUSTSearchNode with Input State:";_S->dump();
  generating_op=_op;
  if(generating_op){
    //cout<<"\t\tgenerating op:";generating_op->dump();
    cost_type=generating_op->get_cost();
    //cout<<",cost:"<<cost_type;
  }
  else{
    cost_type=ONE;
  }
//  father=_father;
  depth=_depth;
  //cout<<",current_depth:"<<depth;
  H=_H;
  //cout<<",H:"<<H<<endl;
  //children_to_generate.assign(_children_to_generate->begin(),_children_to_generate->end());
  //children_to_generate=_children_to_generate;
  //S=new State(*_S);
  S=_S;
//  cout<<"Input State:";S->dump();
  g_successor_generator->generate_applicable_ops(*S,children_to_generate);
  //print_list_children();
}
void HUSTSearchNode::deallocate(){
  //cout<<"elminating S allocated memory"<<endl;
  delete S;
}

int HUSTSearchNode::get_depth(){
  return depth;
}
const Operator* HUSTSearchNode::get_op(){
  return generating_op;
}
/*int HUSTSearchNode::get_father(){
  return father;
}*/
int HUSTSearchNode::get_H(){
  return H;
}

int HUSTSearchNode::get_children_number(){
  return children_to_generate.size();
}

bool HUSTSearchNode::to_be_expanded(){
  //cout<<"\t\tchildren left to expand:"<<children_to_generate.size()<<endl;
  return (!children_to_generate.empty());//true if still has nodes to expand
}
void HUSTSearchNode::print_list_children(){
  for (int i = 0; i < children_to_generate.size(); i++) {
    //const Operator *op = children_to_generate[i];
    cout<<"Operator:";
    children_to_generate[i]->dump();
    cout<<"cost:"<<children_to_generate[i]->get_cost();cout<<endl;
    //cout<<"Operator cost"<<op->get_cost();
  }

  /*vector<const >::iterator iter;
  for( iter = children_to_generate.begin(); iter != children_to_generate.end(); iter++ ) {
    print_op_name(*iter);
  }*/
}

const Operator* HUSTSearchNode::get_next_op(){
  const Operator* next_op=children_to_generate.back();
//  cout<<"\t\tget_next_op:children left to expand:"<<children_to_generate.size()<<endl;
  children_to_generate.pop_back();
//  cout<<"\t\tget_next_op:children left to expand after pop back:"<<children_to_generate.size()<<endl;
  return next_op;
}
const Operator* HUSTSearchNode::get_next_random_op(){
  //static int counter=0;
  /*cout<<"children list:"<<endl;
  for(int i=0;i<children_to_generate.size();i++){
    cout<<i<<":";
    children_to_generate.at(i)->dump();
  }*/
  int chosen_op=rand()%children_to_generate.size();
  const Operator* next_op=children_to_generate.at(chosen_op);
  /*cout<<"\t\tchosen op:"<<chosen_op;next_op->dump();
  cout<<"\t\tget_next_op:children left to expand:"<<children_to_generate.size()<<endl;*/
  children_to_generate.erase(children_to_generate.begin()+chosen_op);
  /*cout<<"children list:"<<endl;
  for(int i=0;i<children_to_generate.size();i++){
    cout<<i<<":";
    children_to_generate.at(i)->dump();
  }
  counter++;
  if(counter>10){
    exit(0);
  }*/
  return next_op;
}
const Operator* HUSTSearchNode::get_next_random_op_no_erase(int &chosen_op){
  chosen_op=rand()%children_to_generate.size();
  const Operator* next_op=children_to_generate.at(chosen_op);

  return next_op;
}
const Operator* HUSTSearchNode::get_op_no_erase(int chosen_op){
  const Operator* next_op=children_to_generate.at(chosen_op);
  return next_op;
}



void HUSTSearchNode::get_state(const State* &_S){
  _S=S;
  /*for ( int i = 0; i < S.num_F; i++ ) {
    printf("\n");
    print_ft_name( S.F[i] );
  }*/
  /*for ( int i = 0; i < _S->num_F; i++ ) {
    printf("\n");
    print_ft_name( _S->F[i] );
  }
  fflush(stdout);cout<<"hola"<<endl;fflush(stdout);*/
}
void HUSTSearchNode::get_children(vector<const Operator *> children){
  children=children_to_generate;
}
void HUSTSearchNode::print_state(){
  S->dump();
}

/*****************************************************************************/
/**************************** DrawNode ***************************************/
/*****************************************************************************/
DrawNode::DrawNode(int _father, string _op, int _depth, int _H , int _hg_node, string _state_id, int _heuristic_states){
  cout<<"Starting DrawNode creation"<<endl;fflush(stdout);
  father=_father;
  op=_op;
  depth=_depth;
  H=_H;
  hg_node=_hg_node;
  gfth_node=false;//default value
  state_id=_state_id;
  heuristic_states=_heuristic_states;
  cout<<"Exiting DrawNode creation"<<endl;fflush(stdout);
}
DrawNode::DrawNode(int _father, string _op, int _depth, int _H , int _hg_node, bool _gfth_node,string _state_id, int _heuristic_states){
  father=_father;
  op=_op;
  depth=_depth;
  H=_H;
  hg_node=_hg_node;
  gfth_node=_gfth_node;
  state_id=_state_id;
  heuristic_states=_heuristic_states;
}

int DrawNode::get_father(){
  return father;
}
string DrawNode::get_op(){
  return op;
}
int DrawNode::get_depth(){
  return depth;
}
int DrawNode::get_F(){
  return depth+H;
}
int DrawNode::get_hg_node(){
  return hg_node;
}
bool DrawNode::get_gfth_status(){
  return gfth_node;
}

string DrawNode::get_state_id(){
    return state_id;
}
int DrawNode::get_heuristic_states(){
  return heuristic_states;
}



/*****************************************************************************/
/**************************** HST ********************************************/
/*****************************************************************************/
class HST::HashTable
    : public __gnu_cxx::hash_map<StateProxy, pair<int, bool> > {
    // This is more like a typedef really, but we need a proper class
    // so that we can hide the information in the header file by using
    // a forward declaration. This is also the reason why the hash
    // table is allocated dynamically in the constructor.
};

/*!This is the constructor for the SearchTree. */
HST::HST( unsigned start_capacity,unsigned start_hset_size )
{
  nodes = new HashTable;
  size=0;
  current_hg_node=0;
  current_F_bound=0;
//  lspace_end=0;
  capacity=start_capacity;
  lsearch_space.reserve(start_capacity);
  alternative_paths=0;
  hset_size=start_hset_size;
  earliest_depth_h_culled.assign(start_hset_size,INT_MAX/2);
  //cout<<"resetting h_capped to false"<<endl;
  //h_capped.assign(start_hset_size,false);
  //check_consistency=false;
  if(!check_consistency){
    cout<<"CHECKING_CONSISTENCY IS OFF, USE CONSISTENT HEURISTICS!!"<<endl;
  }
}
HST::~HST() {
  cout<<"destructor of HST eliminates all the hashed state descriptions"<<endl;
    delete nodes;
}
int HST::get_depth(int index){
  return lsearch_space.at(index).get_depth();
}
void HST::print_list_children(){
  lsearch_space.back().print_list_children();
}
int HST::get_last_depth(){
  if(lsearch_space.size()>0){
    return lsearch_space.back().get_depth();
  }
  else{
    return 0;
  }
}
const Operator* HST::get_op(int index ){
  return lsearch_space.at(index).get_op();
}
/*int HST::get_father(int index ){
  return lsearch_space.at(index).get_father();
}*/
int HST::get_H(int index ){
  return lsearch_space.at(index).get_H();
}
bool HST::to_be_expanded( ){
  return lsearch_space.back().to_be_expanded();
}
int HST::get_lspace_end( ){
  return lsearch_space.size();
}
int HST::get_nodes_fully_expanded_size(){
  return nodes_fully_expanded.size();
}
int HST::get_children_number(int index ){
  return lsearch_space.at(index).get_children_number();
}

/*int HST::get_hg_tree_size(){
  return hg_tree.size();
}

int HST::get_hg_children(int index){
  return hg_tree.at(index).get_children();
}
  
int HST::get_hg_root_depth(int index){
  return hg_tree[index].get_root_depth();
}
int HST::get_hg_max_depth(int index){
  return hg_tree[index].get_max_depth();
}

int HST::get_hg_F(int index){
  return hg_tree[index].get_F();
}

int HST::get_hg_size(int index){
  return hg_tree.at(index).get_size();
}

bool HST::get_hg_closed(int index){
  return hg_tree[index].get_node_status();
}
bool HST::get_hg_gfth_status(int index){
  return hg_tree[index].get_gfth_status();
}*/
int HST::get_alternative_paths(){
  return alternative_paths;
}
/*long HST::get_heuristic_states_per_depth(int depth ){
  if((heuristic_states_per_depth.size())==0){//we might not be doing heuristic Search Trees
    return 0;
  }
  else
    return heuristic_states_per_depth.at(depth);
}*/
void HST::add_alternative_path(){
  alternative_paths++;
}
/*
bool HST::get_hg_EBF_ED(int index,double *EBF,double *ED){

  int degree=(hg_tree[index].get_max_depth()-hg_tree[index].get_root_depth())+1;

  double coeff[degree+1];
  double real[degree+1];
  double imag[degree+1];
  
  *ED=double(degree)-1.0;
  //cout<<"ED:"<<*ED<<" N="<<hg_tree.at(index).get_size()<<endl;
  
  if(degree==hg_tree.at(index).get_size()){//EBF=1
    *EBF=1;
    return true;
  }

  if(degree==1){
    coeff[0]=(1.0-hg_tree.at(index).get_size());
    coeff[1]=(hg_tree.at(index).get_size()-1.0);
  }
  else {
    coeff[0]=1.0;
    for (int i=1;i<(degree-1);i++){
      coeff[i]=0.0;
    }
    coeff[degree-1]=-hg_tree.at(index).get_size();
    coeff[degree]=hg_tree.at(index).get_size()-1.0;
  }

   int roots=rpoly(coeff, degree, real, imag);
   if ((roots==0)){
     cout<<"roots found:"<<roots<<"degree:"<<degree<<endl;
     for (int i=0;i<=degree;i++){
       cout<<"coeff["<<i<<"]:"<<coeff[i]<<endl;
     }
     exit(1);
   }
     
   for (int j=0;j<roots;j++){
     //cout<<"Root"<<real[j]<<endl;
     if ((fabs(real[j] - 1.0) < 1E-9)||(real[j]<0.0)){//ebf cant be less or equal to 1
       //cout<<"root["<<j<<"]="<<real[j]<<" rejected"<<endl;
       continue;
     }
     *EBF=real[j];
     //cout<<"found root["<<j<<"]="<<real[j]<<endl;
     break;
   }*/
 /*    for (int i=0;i<=degree;i++){
       cout<<"coeff["<<i<<"]:"<<coeff[i]<<endl;
     }*//*
   return true;
}*/
double get_ED_from_EBF(double EBF,long N){
  double ED=0;
  /*if(gcmd_line.duplicate_check){
    ED=(log(double(N)*EBF-double(N)+1.0)/log(EBF))-1.0;
  }
  else if(gcmd_line.gfth_check){
    ED=(log((1.0-double(N)+double(N)*EBF-EBF+2.0*initial_children)/(initial_children*(EBF+1)))/log(EBF))+1.0;
  }
  else{*/
    ED=(log(double(N)*EBF-double(N)+1)/log(EBF))-1.0;
    //ED=(log((double(N)*EBF-EBF-double(N)+1+initial_children)/initial_children)/log(EBF))-1.0;//formula with children
  //}
    return ED;
}

/*void HST::get_EBF_ED_from_2iter(int step_size,double *EBF,double *ED,long N1,long N2){
cout<<"need to add gcmd_line to decide which parametric formula applies"<<endl;
exit(0);*/
  /*  int degree=step_size+1,roots;
  double real[degree+1],imag[degree+1],coeff[degree+1];
  //cout<<"EBF":<<EBF<<endl;
  if(gcmd_line.duplicate_check){
    coeff[0]=N1;
    coeff[1]=-N1+1;
    coeff[2]=-N2;
    coeff[3]=N2-1;
    roots=rpoly(coeff, degree, real, imag);
    for (int j=0;j<roots;j++){
      //cout<<"Root"<<real[j]<<endl;
      if ((fabs(real[j] - 1.0) < 1E-9)||(real[j]<1.0)){//ebf cant be less or equal to 1
       cout<<"root["<<j<<"]="<<real[j]<<" rejected"<<endl;
       continue;
      }
      *EBF=real[j];
      cout<<"found root["<<j<<"]="<<real[j]<<endl;
      break;
    }
    *ED=get_ED_from_EBF(*EBF,N2);
  }
  else if(gcmd_line.gfth_check){
    if(step_size==2){
    coeff[0]=N1-1.0;
    coeff[1]=1.0-N1+2.0*double(initial_children);
    coeff[2]=1.0-N2;
    coeff[3]=N2-1.0-2.0*initial_children;
    }
    else if (step_size==1){//step size should be one
    coeff[0]=N1-1.0;
    coeff[1]=2.0-N1+2.0*double(initial_children)-N2;
    coeff[2]=N2-1.0-2.0*initial_children;
    }
    else{
      //cout<<"wrong step_size="<<step_size<<endl<<stderr;
      //exit(0);
      *EBF=2;
      *ED=2;
      return;
    }

    for(int i=0;i<=degree;i++){
      cout<<"coeff["<<i<<"]:"<<coeff[i]<<endl;
    }
    roots=rpoly(coeff, degree, real, imag);
    for (int j=0;j<roots;j++){
      //cout<<"Root"<<real[j]<<endl;
      if ((fabs(real[j] - 1.0) < 1E-9)||(real[j]<1.0)){//ebf cant be less or equal to 1
       cout<<"root["<<j<<"]="<<real[j]<<" rejected"<<endl;
       continue;
      }
      *EBF=real[j];
      cout<<"found root["<<j<<"]="<<real[j]<<endl;
      break;
    }
    *ED=get_ED_from_EBF(*EBF,N2);
  }
  else{
    coeff[0]=N1;
    coeff[1]=-N1+1;
    coeff[2]=-N2;
    coeff[3]=N2-1;
    roots=rpoly(coeff, degree, real, imag);
    for (int j=0;j<roots;j++){
      //cout<<"Root"<<real[j]<<endl;
      if ((fabs(real[j] - 1.0) < 1E-9)||(real[j]<1.0)){//ebf cant be less or equal to 1
       cout<<"root["<<j<<"]="<<real[j]<<" rejected"<<endl;
       continue;
      }
      *EBF=real[j];
      cout<<"found root["<<j<<"]="<<real[j]<<endl;
      break;
    }
    *ED=get_ED_from_EBF(*EBF,N2);
  }*/
//}


  

  
int HST::get_TreeDraw_size( ){
  return TreeDraw.size();
}
int HST::get_TreeDraw_depth(int index ){
  return TreeDraw.at(index).get_depth();
}
int HST::get_TreeDraw_F(int index ){
  return TreeDraw.at(index).get_F();
}
int HST::get_TreeDraw_father(int index ){
  return TreeDraw.at(index).get_father();
}
/*int HST::get_TreeDraw_hg_node(int index ){
  return TreeDraw.at(index).get_hg_node();
}*/
bool HST::get_TreeDraw_gfth_status(int index ){
  return TreeDraw.at(index).get_gfth_status();
}
const char* HST::get_TreeDraw_state_id(int index ){
  return TreeDraw.at(index).get_state_id().c_str();
}
string HST::get_TreeDraw_op(int index ){
  return TreeDraw.at(index).get_op();
}
int HST::get_TreeDraw_heuristic_nodes(int index){
  return TreeDraw.at(index).get_heuristic_states();
}
/*int HST::get_hg_leaf_nodes_size(int index){
  cout<<"index:"<<index<<endl;
  return hg_tree.at(index).get_leaf_nodes_size();
}

int HST::get_hg_leaf_nodes_children(int index){
  return hg_tree[index].get_leaf_nodes_children();
}*/
/*void HST::add_to_HST(vector<int> *children_to_generate, int op, int father, int H, int depth ){
  add_to_HST(children_to_generate,op,father,H,g_initial_state->get_buffer(), depth,0);
}
void HST::add_to_HST(vector<int> *children_to_generate, int op, int father, int H ){
  add_to_HST(children_to_generate,op,father,H,g_initial_state->get_buffer(), -1,0);
}
void HST::add_to_HST(vector<int> *children_to_generate, int op, int father, int H,int depth,int heuristic_states){
  add_to_HST(children_to_generate,op,father,H,g_initial_state->get_buffer(), depth,heuristic_states);
}

void HST::add_to_HST(vector<int> *children_to_generate, int op, int father, int H, const state_var_t* initial_state ){
  add_to_HST(children_to_generate, op, father, H, initial_state, -1,0 );
}*/
  void HST::add_to_HST(const State* parent_state,const Operator *generating_op, int H, int depth)
{
  //const state_var_t* father_state;
  //assert(!children_to_generate.begin().is_axiom());
  const State *S;
  static SearchNodeInfo default_info;
  //cout<<"adding state:"<<endl;fflush(stdout);parent_state->inline_dump();cout<<"with h:"<<H<<endl;



  if((lsearch_space.size())==(lsearch_space.capacity())){
    printf("vector is too small, adding capacity");
    lsearch_space.reserve(2*lsearch_space.capacity());//double the capacity so it does not need to be updated one by one.
  }
  //cout<<"father:"<<father<<endl;fflush(stdout);
  if ( lsearch_space.size() == 0 ) {//Root state is being added
    //depth = 0;//in case we are doing random sampling we want to start at the leaf depth for check_duplicate purposes, so I commented depth=0
    //current_F_bound=depth+H;//current_F_bound=H for root node
    S=parent_state;
    //father_state=initial_state;
    //father_state=const_cast<state_var_t *>(initial_state);
    //adding root node to homogeneus list
    //Homogeneus homogeneus_node(depth,1,current_F_bound,false);
    //hg_tree.push_back(homogeneus_node);
    //adding leaf node data to father Homogeneus tree we are leaving
    //cout<<"adding to hg_tree["<<current_hg_node<<"] "<<children_to_generate->size()<<endl;
    //hg_tree.at(0).add_leaf_node(children_to_generate->size());//adding children to root node
  } else {
    if(depth==-1){//need to calculate depth
      cout<<"depth should not be -1, either check code or allow add_to_HST to calculate new depth"<<endl;exit(0);
      depth = get_depth(lsearch_space.size()) + generating_op->get_cost();
    }
    S=parent_state;
    //cout<<"Need to implement FD-friendly succ generator"<<endl;exit(0);
    //lsearch_space.at(father).get_state(father_state);
    /* adding node to current homogeneus search tree */
    /*if(((H+depth)==current_F_bound)){//adding node to current homogeneus node
      //cout<<"adding node to current homogeneus tree:"<<current_hg_node<<endl;
      //hg_tree.at(current_hg_node).add_node(depth);
      //hg_tree.at(current_hg_node).update_culling_depth(depth);
    }
    else if((H+depth)>current_F_bound){//need to add new node
      //cout<<"adding to hg_tree["<<current_hg_node<<"] a new leaf node with F val"<<H+depth<<endl;
      //hg_tree.at(current_hg_node).add_leaf_node(1);
      //hg_tree.at(current_hg_node).add_child();//adding child to current cluster because of generation of new cluster
      //int depth_homogeneus_root=get_hg_root_depth(current_hg_node);//homogeneus depth is with respect to each node
      int depth_homogeneus_root=depth;//homogeneus root depth is depth of first node.
      //now adding new Homogeneus node with new F value
      current_F_bound=H+depth;
      //Homogeneus homogeneus_node(depth_homogeneus_root,1,current_F_bound,false);
      //hg_tree.push_back(homogeneus_node);
      //current_hg_node=hg_tree.size()-1;
      //cout<<"new culling depth:"<<depth<<endl;
      //hg_tree.at(current_hg_node).update_culling_depth(depth);
    }*/
    //current_F_bound=H+depth;
  }
  if(global_duplicate_check){
    pair<HashTable::iterator, bool> result = nodes->insert(
	make_pair(StateProxy(S), pair<int,bool>(depth,true)));//initially the state is assumed to be visited for the 1st time, making state "already visited" for the current iteration
    if (result.second) {
	// This is a new entry: Must give the state permanent lifetime.
	//cout<<"Added to hash New State:";S->inline_dump();cout<<"at depth"<<depth<<endl;fflush(stdout);
	result.first->first.make_permanent();
	//cout<<"depth:"<<result.first->second.first<<",revisited:"<<result.first->second.second<<",stored_state:";fflush(stdout);//result.first->first.inline_dump();
    }
  }
  //cout<<"\n\t\tadding with depth:"<<depth<<",H:"<<H;
  //HUSTSearchNode searchNode(children_to_generate,op,father,depth,H,lsearch_space.get_state(father));
  HUSTSearchNode searchNode(S,generating_op,depth,H);
  lsearch_space.push_back(searchNode);
  /*for (int i = 0; i < children_to_generate.size(); i++) {
    //const Operator *op = children_to_generate[i];
    //cout<"sizeof";sizeof(*op);cout<<endl;
    cout<<"Operator["<<i<<"] cost:"<<children_to_generate[i]->get_cost()<<endl;
    //cout<<"Operator["<<i<<"] :";op->dump();
  }*/


  /*if(heuristic_states>0){
    if(heuristic_states_per_depth.size()<=depth){
      heuristic_states_per_depth.resize(depth+1,0);
    }
    heuristic_states_per_depth.at(depth)+=heuristic_states;
  }*/


  if (draw_graph){
    if(lsearch_space.size()==0){
      node_id_father=-1;
    }
    cout<<"draw_graph:"<<draw_graph<<",node_id_father:"<<node_id_father<<endl;
    string state_id;
    S->get_state_id_string(&state_id);
    //cout<<"op_name:";fflush(stdout);cout<<generating_op->get_name().c_str()<<endl;fflush(stdout);
    string op_name;
    if(generating_op!=NULL){
      op_name=generating_op->get_name();
    }
    DrawNode drawNode(node_id_father,op_name,depth,H,0,state_id,0);
    TreeDraw.push_back(drawNode);
    //cout<<"old_id_father:"<<node_id_father;
    node_id_father=TreeDraw.size()-1;//next time we generate a DrawNode should be with this node_father_id unless backtracking
    cout<<"new_id_father:"<<node_id_father<<endl;fflush(stdout);
  }

//  if ( gcmd_line.display_info == 5 ) { cout << "\n\tadding state:"<< lspace_end << " at depth:"<< get_depth(lspace_end); }
  
//  lspace_end++;
//  *index=lspace_end;

  /*hash_state( S );*/
  //cout<<"leaving add_to_HST"<<endl;fflush(stdout);
}

/*void HST::add_to_HST( int op, int father, int H )
{
  vector<int> tmp;
  add_to_HST(&tmp, op, father,  H, g_initial_state->get_buffer());
}*/
void HST::reset_h_counters(){
  map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> map_temp;
  h_counter.assign(hset_size+1,map_temp);
  h_counter_substract.assign(hset_size+1,map_temp);
}
void HST::reset_earliest_depth(){
  earliest_depth_h_culled.assign(hset_size,INT_MAX/2);
}
void HST::reset_duplicate_states(){
  HashTable::iterator iter;
  for( iter = nodes->begin(); iter != nodes->end(); iter++ ) {
    if(iter->second.second==true){
      iter->second.second=false;
    }
  }
}
void HST::clear_solution_path( ){
  solution_path.clear();
}
void HST::reset_HST( ){
  //lspace_end=0;
  lsearch_space.clear();//capacitiy will remain same though  
  
  for(int i=0;i<nodes_fully_expanded.size();i++){
    nodes_fully_expanded[i].clear();//capacitiy will remain same though 
  } 
  for(int i=0;i<nodes_culled.size();i++){
    nodes_culled[i].clear();//capacitiy will remain same though  
  }
  for(int i=0;i<nodes_gfth.size();i++){
    nodes_gfth[i].clear();//capacitiy will remain same though  
  }
  nodes_fully_expanded.clear();
  nodes_culled.clear();
  nodes_gfth.clear();  
  //hg_tree.clear();
  current_hg_node=0;
  //current_F_bound=0; 
  if (draw_graph){
    node_id_father=0;
    TreeDraw.clear();
  }
  alternative_paths=0;
  current_state_id.clear();
  //heuristic_states_per_depth.clear();
  map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> map_temp;
  h_counter.assign(hset_size+1,map_temp);
  h_counter_substract.assign(hset_size+1,map_temp);
  cout<<"h_counter reseted to size:"<<hset_size<<endl;
  //map<boost::dynamic_bitset<hset_size>,unsigned long,bitset_comp2> h_comb_map;
  best_heuristic.clear();
  earliest_depth_h_culled.assign(hset_size,INT_MAX/2);
  cout<<"reseted earliest_depth_h_culled to INT_MAX/2 for "<<earliest_depth_h_culled.size()<<"remaining heuristics"<<endl;
  /*cout<<"lsearch_space.capacity:"<<lsearch_space.capacity()<<endl;
  cout<<"nodes_fully_expaned.capacity::"<<nodes_fully_expanded.capacity()<<endl;
  cout<<"nodes_culled.capacity:"<<nodes_culled.capacity()<<endl;*/
  if(iter_index==1){//first_iteration
    cout<<"Resetting HBF and prev_val as we are starting a new problem"<<endl;
    prev_val.clear();
    HBF.clear();
    cout<<"resetting h_capped to false"<<endl;
    h_capped.assign(hset_size,false);
    best_h_comb.reset();
    //best_h_comb.set();
    
    HUST_total_size=0;
    HSTs_total_size=0;
    HUST_total_time=0;
    Credit_total_time=0;
    HSTs_total_time=0;
    Total_Sampling_time=0;
    best_h_comb_list.clear();
    solution_path.clear();
    dead_end_nodes=0;
  }
  //Next make all visited states unvisited for next iteration check duplicate
  cout<<"nodes.size:"<<nodes->size()<<endl;
  HashTable::iterator iter;
  long nodes_revisited=0;
  for( iter = nodes->begin(); iter != nodes->end(); iter++ ) {
    if(iter->second.second==true){
      nodes_revisited++;
      //cout<<"state:"<<endl;iter->first.dump();cout<<"was revisited"<<endl;
    }
    //cout<<"state:"<<endl;iter->first.dump();cout<<"was not revisited"<<endl;
    iter->second.second=false;
  }
  cout<<"nodes revisited="<<nodes_revisited<<endl;
}

int HST::get_nodes_culled(int depth, int H){
  if(depth>=nodes_culled.size())
    return 0;
  else if(H>=nodes_culled[depth].size())
    return 0;
  else
    return nodes_culled[depth][H];
}

int HST::get_nodes_gfth(int depth, int H){
  if(depth>=nodes_gfth.size())
    return 0;
  else if(H>=nodes_gfth[depth].size())
    return 0;
  else
    return nodes_gfth[depth][H];
}

double HST::get_ED(){
  long total_ED=0; 
  long total_nodes=0;

  
  //add culled nodes
  for( int depth=0;depth<nodes_culled.size(); depth++ ) {
    for( int h=0; h<nodes_culled[depth].size(); h++ ) {
      total_ED+=nodes_culled[depth][h]*depth;
      total_nodes+=nodes_culled[depth][h];
      //cout<<"nodes culled at depth["<<depth<<"]:"<<nodes_culled[depth][h]<<endl;
    }
  }

  //add gfth_nodes
  for( int depth=0;depth<nodes_gfth.size(); depth++ ) {
    for( int h=0;h<nodes_gfth[depth].size(); h++ ) {
      total_ED+=nodes_gfth[depth][h]*depth;
      total_nodes+=nodes_gfth[depth][h];
      //cout<<"nodes culled gfth at depth["<<depth<<"]:"<<nodes_gfth[depth][h]<<endl;
    }
  }
  double Avg_ED=double(total_ED)/double(total_nodes);
  //cout<<"Avg_ED"<<Avg_ED<<endl;
  return Avg_ED;
}
    
  


int HST::get_nodes_fully_expanded(int depth, int H){
  if(depth>=nodes_fully_expanded.size())
    return 0;
  else if(H>=nodes_fully_expanded[depth].size())
    return 0;
  else
    return nodes_fully_expanded[depth][H];
}

/*void HST::add_nodes_culled(int depth, int H){
  add_nodes_culled(depth,H,0,0);
}*/
void HST::add_nodes_culled(int depth, int H, const Operator* op,const State* S){
//  cout << "\tnodes_culled.size:" << nodes_culled.size() << endl;
//  cout<<"op:"<<op<<endl;
  if(depth>=nodes_culled.size()){//need to add depth&H to vector
    nodes_culled.resize(depth+1);
    nodes_culled[depth].resize(H+1,0);//add as many H bins as value of H
    //cout << "\tadding H: " << H <<"depth:" << depth << ",nodes_culled.depth.size:" << nodes_culled[depth].size() << endl;
    nodes_culled[depth][H]++;
  }
  else if(H>=nodes_culled[depth].size()){//need to add H to vector
//    cout << "\tadding H: " << H << endl;
//    cout << "\tadding node to vector of size:" << nodes_culled.size() << "," << nodes_culled[depth].size() << endl;
    nodes_culled[depth].resize(H+1,0);
//    cout << "\tnew H size:"  << nodes_culled[depth].size() << endl;
    nodes_culled[depth][H]++;
//    cout << "\tafter adding" << endl;
  }
  else {//bin already exists
//    cout << "\tadding node to vector of size:" << nodes_culled.size() << "," << nodes_culled[depth].size() << endl;
    nodes_culled[depth][H]++;
  }
  //adding heuristic_states overhead
/*  if(heuristic_states>0){
    if(heuristic_states_per_depth.size()<=depth){
      heuristic_states_per_depth.resize(depth+1,0);
    }
    heuristic_states_per_depth.at(depth)+=heuristic_states;
  }*/

  //else if((H+depth)>current_F_bound){//need to add new culled node
  //hg_tree.at(current_hg_node).add_child();//adding a child to current node, this is due to the new cluster
  //Homogeneus homogeneus_node(depth,1,H+depth,true);
  //hg_tree.push_back(homogeneus_node);
  //}
  if (draw_graph){
    //DrawNode drawNode(node_id_father,op,depth,H,hg_tree.size()-1,current_state_id,heuristic_states);//node added is culled child of current hg node
    string state_id;
    S->get_state_id_string(&state_id);
    DrawNode drawNode(node_id_father,op->get_name(),depth,H,1,state_id,1);//node added is culled child of current hg node
    TreeDraw.push_back(drawNode);
  }
}


/*void HST::add_nodes_gfth(int depth, int H){
  add_nodes_gfth(depth, H, 0);
}*/
void HST::add_nodes_gfth(){
  cout<<"implement add_nodes_gfth to FD"<<endl;exit(0);
}
/*
void HST::add_nodes_gfth(int depth, int H, int heuristic_states){
  cout<<"implement add_nodes_gfth to FD"<<endl;exit(0);
}*/
/*void HST::add_nodes_gfth(int depth, int H, int heuristic_states){
  if(depth>=nodes_gfth.size()){//need to add depth&H to vector
    nodes_gfth.resize(depth+1);
    nodes_gfth[depth].resize(H+1,0);//add as many H bins as value of H
    nodes_gfth[depth][H]++;
  }
  else if(H>=nodes_gfth[depth].size()){//need to add H to vector
    nodes_gfth[depth].resize(H+1,0);
    nodes_gfth[depth][H]++;
  }
  else {//bin already exists
    nodes_gfth[depth][H]++;
  }

  *//* adding gfth node to current homogeneus search node, they are considered internal nodes */
  /*if(((H+depth)==current_F_bound)){//adding gfth_check removed node to current homogeneus node
    //hg_tree.at(current_hg_node).add_node(depth);
    *//*if (draw_graph){
      DrawNode drawNode(node_id_father,0,depth,H,current_hg_node,true,current_state_id,heuristic_states);
      TreeDraw.push_back(drawNode);
    }*//*
  }
  else if((H+depth)>current_F_bound){//need to add new gfth node
    //hg_tree.at(current_hg_node).add_child();//adding a child to current node, this is due to the new cluster
    //Homogeneus homogeneus_node(depth,1,H+depth,true,true);
    //hg_tree.push_back(homogeneus_node);
    *//*if (draw_graph){
      DrawNode drawNode(node_id_father,0,depth,H,get_hg_tree_size()-1,true,current_state_id,heuristic_states);
      TreeDraw.push_back(drawNode);
    }*//*
  }
  else{//revisiting gfth check
    //hg_tree.at(current_hg_node).add_node(depth);//not sure if correct
    *//*if (draw_graph){
      DrawNode drawNode(node_id_father,0,depth,H,current_hg_node,true,current_state_id,heuristic_states);//node added is culled child of current hg node
      TreeDraw.push_back(drawNode);
    }*//*
  }
}*/

bool HST::empty(){
  return lsearch_space.empty();
}
const Operator* HST::get_next_op(){
  return lsearch_space.back().get_next_op();
}
const Operator* HST::get_next_random_op(){
  return lsearch_space.back().get_next_random_op();
}
const Operator* HST::get_next_random_op_no_erase(int &chosen_op){
  return lsearch_space.back().get_next_random_op_no_erase(chosen_op);
}
const Operator* HST::get_op_no_erase(int chosen_op){
    cout<<"\tlsearch_space.size:"<<lsearch_space.size()<<",chosen_op="<<chosen_op<<",current available children="<<get_current_children_number()<<endl;
  if(chosen_op>=get_current_children_number()){
    cout<<"chosen_op="<<chosen_op<<"but current available children="<<get_current_children_number()<<",so review code"<<endl;
    exit(0);
  }
  return lsearch_space.back().get_op_no_erase(chosen_op);
}
void HST::backtrack( ){
  int depth=lsearch_space.back().get_depth();
  int H=lsearch_space.back().get_H();

  if (draw_graph){
    if(node_id_father<0){
      cout<<"Error, node_id_father:%d"<<node_id_father<<endl;
      exit(0);
    }
    node_id_father=TreeDraw.at(node_id_father).get_father();
  }
      
  if(depth>=nodes_fully_expanded.size()){//need to add depth&H to vector
    nodes_fully_expanded.resize(depth+1);
    nodes_fully_expanded[depth].resize(H+1,0);//add as many H bins as value of H
//    cout << "\n\t\tadding H: " << H <<"depth:" << depth << ",nodes_fully_expanded.depth.size:" << nodes_fully_expanded[depth].size() << endl;
    nodes_fully_expanded[depth][H]++;
  }
  else if(H>=nodes_fully_expanded.at(depth).size()){//need to add H to vector
//    cout << "\tadding H: " << H << endl;
//    cout << "\tadding node to vector of size:" << nodes_culled.size() << "," << nodes_culled[depth].size() << endl;
    nodes_fully_expanded.at(depth).resize(H+1,0);
//    cout << "\tnew H size:"  << nodes_culled[depth].size() << endl;
    nodes_fully_expanded.at(depth).at(H)++;
//    cout << "\n\t\tadding H: " << H <<"depth:" << depth << ",nodes_fully_expanded.depth.size:" << nodes_fully_expanded[depth].size() << endl;
//    cout << "\tafter adding" << endl;
  }
  else {//bin already exists
//    cout << "\tadding node to vector of size:" << nodes_culled.size() << "," << nodes_culled[depth].size() << endl;
    nodes_fully_expanded.at(depth).at(H)++;
 //   cout << "\n\t\tadding H: " << H <<"depth:" << depth << ",nodes_fully_expanded.depth.size:" << nodes_fully_expanded[depth].size() << endl;
  }

  //cout<<"current_F_bound:"<<current_F_bound<<" backtracking with depth "<<depth<<" and culling_depth:"<< hg_tree.at(current_hg_node).get_culling_depth()<<endl;
  //adjusting Homogeneus Search Tree accordingly
  /*if(hg_tree.at(current_hg_node).get_culling_depth()==depth&&(lsearch_space.size()>1)){//going back to previous homogeneus search tree if exists
    hg_tree.at(current_hg_node).close_node();
    while(hg_tree.at(current_hg_node).get_node_status()==true){
      current_hg_node--;
    }*/
    //current_F_bound=hg_tree.at(current_hg_node).get_F();
    //culling_depth=hg_tree.at(current_hg_node).get_root_depth();
    //culling_depth=depth;
    //culling_leaf_node_depth=hg_tree.at(current_hg_node).get_culling_leaf_depth();
    //cout<<"went back to previous current_F_bound:"<<current_F_bound<<",culling_depth:"<<hg_tree.at(current_hg_node).get_culling_depth()<<",culling_leaf_node_depth:"<<hg_tree.at(current_hg_node).get_culling_leaf_depth()<<endl;
    //need to add new leaf node with one child
    //if(depth>hg_tree.at(current_hg_node).get_culling_leaf_depth()){
    //  cout<<"starting new leaf with 1 child @ homogeneus tree["<<current_hg_node<<"]"<<endl;
    //  hg_tree.at(current_hg_node).add_leaf_node(1);
    //}
    //else{
    //  cout<<"adding child to current leaf node hg_tree["<<current_hg_node<<"]"<<endl;
    //  hg_tree.at(current_hg_node).increase_leaf_node();
    //}
    //hg_tree.at(current_hg_node).update_culling_depth(depth);
  //}
  //cout<<"updating culling_leaf depth hg_tree["<<current_hg_node<<"] to "<<depth<<endl;
  //hg_tree.at(current_hg_node).update_culling_leaf_depth(depth-1);
  //everything done so removing node from current search space
  //cout<<"eliminating S,before:"<<endl;lsearch_space.back().print_state();fflush(stdout);
  lsearch_space.back().deallocate();
  //cout<<"eliminating S,after:"<<endl;lsearch_space.back().print_state();fflush(stdout);
  lsearch_space.pop_back();//remove fully expanded node from search tree
  if(check_consistency){//no need to check for consistency when doing MAXTREE
    for(unsigned long index=0;index<hset_size;index++){
      if(earliest_depth_h_culled.at(index)==depth){//backtracked beyond first culling in path
	//cout<<"backtracking at detph:"<<depth<<", earliest_depth_h:"<<index<<":"<<earliest_depth_h_culled.at(index)<<"is now INT_MAX/2"<<endl;
	earliest_depth_h_culled.at(index)=INT_MAX/2;
      }
    }
  }
}

void HST::copy_current_path(vector<HUSTSearchNode> *solution_path){
  solution_path->assign(lsearch_space.begin(),lsearch_space.end());
}

void HST::get_current_path(vector<const Operator*> &current_path,const Operator* last_op){//solution state and its generating op is not yet in HST
  current_path.clear();
  for(int i=1;i<lsearch_space.size();i++){
    current_path.push_back(lsearch_space.at(i).get_op());
  }
  if(last_op!=NULL){
    current_path.push_back(last_op);
  }
}
    //solution_path.back()->dump();
void HST::copy_solution_path(const Operator* last_op){//solution state and its generating op is not yet in HST
  for(int i=1;i<lsearch_space.size();i++){
    solution_path.push_back(lsearch_space.at(i).get_op());
    solution_path.back()->dump();
  }
  if(last_op!=NULL){
    solution_path.push_back(last_op);
    solution_path.back()->dump();
  }
}
void HST::print_solution_path(){
  cout<<"Solution steps:"<<endl;
  for(int i=0;i<solution_path.size();i++){
    solution_path.at(i)->dump();cout<<",";fflush(stdout);
  }
  cout<<endl;
}

int HST::get_max_depth(){
  //cout<<"nodes_culled:"<<nodes_culled.size()<<endl;
  //cout<<"nodes_fully expanded:"<<nodes_fully_expanded.size()<<endl;
  return (max(nodes_culled.size(),max(nodes_fully_expanded.size(),nodes_gfth.size()))-1.0);
}
int HST::get_max_size(){
  int max_size=0;
  for (int i=0;i<nodes_culled.size();i++){
    for (int j=0;j<nodes_culled[i].size();j++){
      max_size+=nodes_culled[i][j];
    }
  }
  //int nodesCulled=max_size;
  //cout<<endl<<"nodes culled:"<<nodesCulled;
  for (int i=0;i<nodes_fully_expanded.size();i++){
    for (int j=0;j<nodes_fully_expanded[i].size();j++){
      max_size+=nodes_fully_expanded[i][j];
    }
  }
  //int nodesFullyExpanded=max_size-nodesCulled;
  //cout<<endl<<"nodes fully expanded:"<<nodesFullyExpanded;
  max_size=max_size+lsearch_space.size();//in case we are doing breadth first
  //cout<<endl<<"max_size:"<<max_size;
  return max_size;
}

long HST::get_nodes_fully_expanded(){
  long total_fully_expanded=0;
  for (int i=0;i<nodes_fully_expanded.size();i++){
    for (int j=0;j<nodes_fully_expanded[i].size();j++){
      total_fully_expanded+=long(nodes_fully_expanded[i][j]);
    }
  }
  return total_fully_expanded;
}
const Operator* HST::get_last_op(){
  return lsearch_space.back().get_op();
}
int HST::get_last_H(){
  return lsearch_space.back().get_H();
}

/*bool HST::get_avg_hg_tree_size_by_F_lim(int F_limit, double *avg_size,int *number_clusters,double *avg_children,double *avg_size_growth, double *total_size_growth){
  vector<Homogeneus>::iterator iter;
  long total_size_F_lim=0,total_size=0;
  long total_children=0;
  *number_clusters=0;
  static double last_avg_size=0;
  static long last_total_size=0;

  if(first_call_statistics){//first iteration for this problem
    total_size=0;
  }

  for( iter = hg_tree.begin(); iter != hg_tree.end(); iter++ ) {
    if(iter->get_F()==F_limit){// only adding clusters with F-value
      *number_clusters+=1;
      total_size_F_lim+=iter->get_size();
      total_size+=iter->get_size();
      total_children+=iter->get_children();
    }
    else if(iter->get_F()>F_limit){
      total_size+=1;
    }
  }
  //cout<<"Number clusters"<<*number_clusters<<endl;
  //cout<<"Total_size"<<total_size_F_lim<<endl;
  *avg_size=((double)total_size_F_lim)/((double)*number_clusters);
  *avg_children=((double)total_children)/((double)*number_clusters);
  if(first_call_statistics){//first iteration for this problem
    *avg_size_growth=*avg_children;
    *total_size_growth=total_size;
  }
  else{
    *avg_size_growth=*avg_size/last_avg_size;
    *total_size_growth=(double)total_size/(double)last_total_size;
  }
  last_avg_size=*avg_size;
  last_total_size=total_size;
    
  //cout<<"avg_size"<<*avg_size<<endl;
  return true;
}*/

void HST::set_current_state_id(string *_state_id ){
  current_state_id=*_state_id;
}
void HST::set_current_F_bound(int F){
  if(current_F_bound!=F){
    cout<<"updating F_bound from"<<current_F_bound<<" to "<<F<<endl;
    current_F_bound=F;
  }
}
int HST::get_current_F_bound(){
  return current_F_bound;
}
void HST::get_current_state_id(string *_state_id ){
  *_state_id=current_state_id;
}
void HST::get_state(int index,const State* &S ){
  lsearch_space.at(index).get_state(S);
}
void HST::get_current_state(const State* &S ){
  lsearch_space.back().get_state(S);
}
void HST::extract_current_path(){
    cout<<"Use equivalent FD function"<<endl;exit(0);/*
  vector<HUSTSearchNode>::iterator iter;
  const state_var_t* S;
  //cout<<"path_size"<<lsearch_space.size()<<endl;fflush(NULL);
  
  for( iter = (lsearch_space.begin()+1); iter != lsearch_space.end(); iter++ ) {
    //cout<<"iter->get_op"<<iter->get_op()<<endl;fflush(NULL);
    print_op_name(iter->get_op());cout<<",h:"<<iter->get_H()<<",depth:"<<iter->get_depth()<<endl; 
    //iter->get_state(S);print_state(*S);
    //printf("tile: %s", gconstants[gop_conn[iter->get_op()].inst_table[0]]);cout<<endl;
  }
}
int HST::count_significant_tiles(int start, int end){
  vector<HUSTSearchNode>::iterator iter;
  char *tile_number;
  int h_dist=0,unsig_dist=0;
  state_var_t final_state;
  //int tile_counter[MAX_TILE-1]={0};
  int total_solution_length=0;
  
  //tile_moves.resize(tile_moves.size()+1);//adding new solution


  //cout<<"lsearch_space.size:"<<lsearch_space.size()<<endl;

  if(lsearch_space.size()==0){
    return(0);
  }

  for( iter = (lsearch_space.begin()+1); iter != lsearch_space.end(); iter++ ) {
    //print_op_name(iter->get_op());//cout<<endl; 
    tile_number=&(gconstants[gop_conn[iter->get_op()].inst_table[0]][1]);
    if((atoi(tile_number)>=start)&&(atoi(tile_number)<=end)){//significant tile
      h_dist++;
      //tile_counter[atoi(tile_number)-1]++;
      *//*if(tile_moves.at(tile_moves.size()-1).find(iter->get_op())==tile_moves.at(tile_moves.size()-1).end()){//move not in list so we add it
	tile_moves.at(tile_moves.size()-1).insert(iter->get_op());
	cout<<",new_move,";
      }
      else{
	cout<<",old_move,";
      }*/
      //cout<<",sig_tile,h_dist:"<<h_dist<<endl;
    /*}
    else{
      unsig_dist++;*/
      /*if(tile_moves.at(tile_moves.size()-1).find(iter->get_op())==tile_moves.at(tile_moves.size()-1).end()){//move not in list so we add it
	tile_moves.at(tile_moves.size()-1).insert(iter->get_op());
	//tile_counter[atoi(tile_number)-1]++;
	cout<<",new_move,";
      }
      else{
	cout<<",old_move,";
      }*/
      //cout<<",unsig_tile,unsig_dist:"<<unsig_dist<<endl;
    /*}
  }*/

  /*for(int i=0;i<(MAX_TILE-1);i++){ 
    cout<<"current_T"<<i+1<<":"<<tile_counter[i]<<",";
    tiles_sol_length[i]=max(tiles_sol_length[i],tile_counter[i]);
    cout<<"final_T"<<i+1<<":"<<tiles_sol_length[i]<<endl;
    total_solution_length+=tiles_sol_length[i];
    if(total_solution_length>26){
      cout<<"SOLUTION NOT ADMISSIBLE!"<<endl;
      exit(1);
    }

  }*/
      //cout<<",h_dist:"<<h_dist<<endl;
      //cout<<"unsig_dist:"<<unsig_dist<<endl;
  /*return h_dist;*/
}

/*void HST::generate_next_state(int op_id){
  state_var_t *Current_state_var_t;
  lsearch_space[lsearch_space.size()-1].get_current_state(Current_state_var_t);
  update_state(state_var_t *Current_state_var_t,int op_id);
}*/
/*void HST::create_pattern_database(){
  state_var_t Start;
  int counter;
  vector<int> number_children;
  vector<int> tiles_first_group;//used for heuristic grouping of tiles
  vector<int> tiles_second_group;//used for heuristic grouping of tiles
  static bool first_run=true;
  int tile_order[MAX_TILE];
  unsigned int database_size=1;
  clock_t clockstartDatabase,clockendDatabase;
  clock_t g_database_time;
  

  //starting timer  
  clockstartDatabase=clock();

  //clearing duplicate check structures
  visited_database.clear();

  //initial point is goal state
  source_to_dest( &Start, ggoal_state );
  //need to add blank on x1 y1 to goal_state(it wasnt necessary for goal checking purposes
  Start.num_F++;
  int blank_x1_y1_predicate=find_blank_x1_y1();
  Start.F[Start.num_F-1]=blank_x1_y1_predicate;
  
  switch(gcmd_line.start_heuristic){//choose which grouping we use
    case 0://7-7-1 division
      if(first_run){
	cout<<"Using 1 tile Division"<<endl;
      }
      tiles_second_group.push_back(1); tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(4);tiles_second_group.push_back(5);
      tiles_second_group.push_back(6); tiles_second_group.push_back(7); tiles_second_group.push_back(8); tiles_second_group.push_back(9); tiles_second_group.push_back(10);
      tiles_first_group.push_back(11); tiles_first_group.push_back(12); tiles_first_group.push_back(13); tiles_first_group.push_back(14); tiles_first_group.push_back(15);
      break;
    case 1://7-7-1 division
      if(first_run){
	cout<<"Using 7-8 Division"<<endl;
      }
      tiles_first_group.push_back(1); tiles_first_group.push_back(2); tiles_first_group.push_back(3); tiles_first_group.push_back(4);
      tiles_first_group.push_back(5); tiles_first_group.push_back(6); tiles_first_group.push_back(7); 
      tiles_second_group.push_back(8); tiles_second_group.push_back(9); tiles_second_group.push_back(10); tiles_second_group.push_back(11);
      tiles_second_group.push_back(12); tiles_second_group.push_back(13); tiles_second_group.push_back(14); tiles_second_group.push_back(15);
      break;
    case 2://8-7 division
      if(first_run){
	cout<<"Using 8-7 Division"<<endl;
      }
      tiles_first_group.push_back(8); tiles_first_group.push_back(9); tiles_first_group.push_back(10); tiles_first_group.push_back(11);
      tiles_first_group.push_back(12); tiles_first_group.push_back(13); tiles_first_group.push_back(14); tiles_first_group.push_back(15);
      tiles_second_group.push_back(1); tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(4);
      tiles_second_group.push_back(5); tiles_second_group.push_back(6); tiles_second_group.push_back(7); 
      break;
    case 3://L-R1 division
      if(first_run){
	cout<<"Using LR1 Division"<<endl;
      }
      tiles_first_group.push_back(1); tiles_first_group.push_back(4); tiles_first_group.push_back(5); tiles_first_group.push_back(8);
      tiles_first_group.push_back(9); tiles_first_group.push_back(12); tiles_first_group.push_back(13);
      tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(6);tiles_second_group.push_back(7); 
      tiles_second_group.push_back(10); tiles_second_group.push_back(11);tiles_second_group.push_back(14);tiles_second_group.push_back(15);
      break;
    case 4://L-R2 division
      if(first_run){
	cout<<"Using LR2 Division"<<endl;
      }
      tiles_first_group.push_back(1); tiles_first_group.push_back(4); tiles_first_group.push_back(5); tiles_first_group.push_back(8);
      tiles_first_group.push_back(9); tiles_first_group.push_back(12); tiles_first_group.push_back(13);tiles_first_group.push_back(14);
      tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(6);tiles_second_group.push_back(7); 
      tiles_second_group.push_back(10); tiles_second_group.push_back(11);tiles_second_group.push_back(15);
      break;
    case 5://7-7-1 division, 2nd
      if(first_run){
	cout<<"Using 7 Division,2"<<endl;
      }
      tiles_second_group.push_back(1); tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(4);
      tiles_second_group.push_back(5); tiles_second_group.push_back(6); tiles_second_group.push_back(7);tiles_second_group.push_back(11); 
      tiles_first_group.push_back(8); tiles_first_group.push_back(9); tiles_first_group.push_back(10); 
      tiles_first_group.push_back(12); tiles_first_group.push_back(13); tiles_first_group.push_back(14); tiles_first_group.push_back(15);
      break;
    case 6://7-7-1 division, 2nd
      if(first_run){
	cout<<"Using 7 Division,3"<<endl;
      }
      tiles_second_group.push_back(1); tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(4);
      tiles_second_group.push_back(5); tiles_second_group.push_back(6); tiles_second_group.push_back(7);tiles_second_group.push_back(8); 
      tiles_first_group.push_back(9); tiles_first_group.push_back(10);tiles_first_group.push_back(11);
      tiles_first_group.push_back(12); tiles_first_group.push_back(13); tiles_first_group.push_back(14); tiles_first_group.push_back(15);
      break;
    case 7://7-7-1 division, 3rd
      if(first_run){
	cout<<"Using 7 Division,4"<<endl;
      }
      tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(6); tiles_second_group.push_back(7);
      tiles_second_group.push_back(10);tiles_second_group.push_back(11); tiles_second_group.push_back(14); tiles_second_group.push_back(15);
      tiles_first_group.push_back(1); tiles_first_group.push_back(4);tiles_first_group.push_back(5);
      tiles_first_group.push_back(8); tiles_first_group.push_back(9); tiles_first_group.push_back(12); tiles_first_group.push_back(13);
      break;
    case 8://7-7-1 division, 4th
      if(first_run){
	cout<<"Using 7 Division,5"<<endl;
      }
      tiles_first_group.push_back(3); tiles_first_group.push_back(6); tiles_first_group.push_back(7); tiles_first_group.push_back(10);
      tiles_first_group.push_back(11); tiles_first_group.push_back(14); tiles_first_group.push_back(15);
      tiles_second_group.push_back(1); tiles_second_group.push_back(2); tiles_second_group.push_back(4);tiles_second_group.push_back(5);
      tiles_second_group.push_back(8); tiles_second_group.push_back(9); tiles_second_group.push_back(12); tiles_second_group.push_back(13);
      break;
    case 9://7-7-1 division, 4th
      if(first_run){
	cout<<"Creating database of whole 8 puzzle"<<endl;
      }
      tiles_second_group.push_back(1); tiles_second_group.push_back(2); tiles_second_group.push_back(3); tiles_second_group.push_back(4);
      //tiles_first_group.push_back(5); tiles_first_group.push_back(6); tiles_first_group.push_back(7);tiles_first_group.push_back(8);
      tiles_first_group.push_back(5); tiles_first_group.push_back(6); tiles_first_group.push_back(7);tiles_first_group.push_back(8);
      break;
    case 10://7-7-1 division, 5th
      if(first_run){
	cout<<"Using 7 Division,5"<<endl;
      }
      tiles_first_group.push_back(2); tiles_first_group.push_back(3); tiles_first_group.push_back(6); tiles_first_group.push_back(7);
      tiles_first_group.push_back(10); tiles_first_group.push_back(11); tiles_first_group.push_back(15);
      tiles_second_group.push_back(1); tiles_second_group.push_back(4); tiles_second_group.push_back(5);tiles_second_group.push_back(8);
      tiles_second_group.push_back(9); tiles_second_group.push_back(12); tiles_second_group.push_back(13); tiles_second_group.push_back(14);
      break;
    case 11://7-7-1 division, 6th
      if(first_run){
	cout<<"Using 7 Division,5"<<endl;
      }
      tiles_first_group.push_back(2); tiles_first_group.push_back(6); tiles_first_group.push_back(7); tiles_first_group.push_back(10);
      tiles_first_group.push_back(11); tiles_first_group.push_back(14); tiles_first_group.push_back(15);
      tiles_second_group.push_back(1); tiles_second_group.push_back(3); tiles_second_group.push_back(4);tiles_second_group.push_back(5);
      tiles_second_group.push_back(8); tiles_second_group.push_back(9); tiles_second_group.push_back(12); tiles_second_group.push_back(13);
      break;
    default:
      break;
  }
  for(int i=(MAX_TILE-tiles_first_group.size()+1);i<(MAX_TILE+1);i++){
    database_size=i*database_size;
  }
  *//*if(MAX_TILE==9){//even/odd
    database_size=database_size/2;
  }*///doesn not work with state_id to index function
/*  cout<<"database_entries:"<<database_size<<endl;fflush(stdout);
  pattern_database.assign(database_size,255);
  cout<<"database_size:"<<double(database_size*sizeof(char))/1024000.0<<endl;fflush(stdout);


  for (int i=0;i<MAX_TILE;i++){//finding which order is each tile
    //printf("%c is %d\n",gconstants[(grelevant_facts[C1.F[i]].args)[0]][1],gconstants[(grelevant_facts[C1.F[i]].args)[0]][1]);
    //printf("%c is %d\n",gconstants[(grelevant_facts[C1.F[i]].args)[0]][0],gconstants[(grelevant_facts[C1.F[i]].args)[0]][0]);
    if(gconstants[(grelevant_facts[ggoal_state.F[i]].args)[0]][0]=='T'){//dealing with tile
      tile_order[atoi(&(gconstants[(grelevant_facts[ggoal_state.F[i]].args)[0]][1]))]=i;
      //cout<<"tile_order["<<i<<"]:"<<tile_order[i];fflush(NULL);
    }
    else{//dealing with blank
      tile_order[0]=i;
    }
  }
  
  //save copy of current state
  //copy_1P_and_AH_recursive();
  blank_facts(tile_order,&tiles_second_group,&ggoal_state,&Start);
  if(first_run){
    cout<<"first grouping:"<<endl;print_state(Start);fflush(NULL);
    first_run=false;
  }
  //now initialize available actions
  get_1P_and_AH( Start );//setting actions
  cout << "\nInitial state_var_t Type:"<<gnum_A<<endl;fflush(stdout);
  number_children.assign(&gA[0],&gA[gnum_A]);
  cout<<"Initial # children to expand:"<<number_children.size()<<endl;
  for(int counter=0;counter<gnum_A;counter++){
    printf("\t\n");print_op_name(gA[counter]);
  }
  //adding intial state to tree
  add_to_HST_breadth_first(&Start,&tiles_first_group);
  
  while(!next.empty()){//keep expanding until no more nodes
    add_to_HST_breadth_first(&Start,&tiles_first_group);
  }
  cout<<"finished with database, entries:"<<visited_database.size()<<endl;
  //end timer, dont care how long it takes to write file
  //
  clockendDatabase=clock();
  g_database_time=clockendDatabase-clockstartDatabase;
  cout<<"Total time for database creation(writting file not included):"<<g_database_time;
  print_database_to_file(&tiles_first_group,gcmd_line.end_heuristic);
  //restoring connectivity graph for second search
  //restore_1P_and_AH_recursive();
}*/
  
//this function pulls a node from the beginning and adds its children at the end of the queue
//if first node it justs add to HST and returns
/*void HST::add_to_HST_breadth_first(const state_var_t* initial_state,const vector<int>* tiles_grouped)
{
  cout<<"need to make function FD-compatible"<<endl;exit(0);*//*
  state_var_t successor_state;
  const state_var_t* current_state;
  const vector<int>::iterator iter;
  vector<int> successor_children;
  int depth=0;
  vector<int> state_id;
  compressed_search_node node;

  //cout<<"father:"<<father<<endl;fflush(stdout);
  if ( next.size() == 0 ) {
    cout<<"Adding root node"<<endl;
    node.depth = 0;
    node.S=*initial_state;
    check_duplicate_database(initial_state,tiles_grouped,node.depth);//adding to database
    next.push(node);
    return;//we are finished
  } else {
    node.depth = next.front().depth + 1;
  }
  current_state=&(next.front().S);
  cout<<"get applicable operators in an FD-manner"<<endl;exit(0);
  //build_fixpoint_BFS(current_state);//getting new action list
  //reset_fixpoint();
  //cout<<"current_state:"<<endl;
  //print_state(*current_state);
  //vector<int> number_children; number_children.assign(&gA[0],&gA[gnum_A]);
  //cout<<"# children to expand:"<<number_children.size()<<endl;*/
  /*for(int counter=0;counter<gnum_A;counter++){
    printf("\t\n");print_op_name(gA[counter]);
  }*/
    
  //for( iter = current_children->begin(); iter != current_children->end(); iter++ ) 
  /*for( int iter=0;iter<gnum_A; iter++)//need to make gnum_A FD-compatible
  {
    //cout<<"\tchecking:";print_op_name(gA[iter]);cout<<endl;
    result_to_dest( &node.S,current_state,gA[iter]);
    //cout<<"resulting state:"<<endl;print_state(node.S);
    //state_id.clear();generate_state_id_database(&(node.S),&state_id,tiles_grouped);//get positions to state_id
    if(check_duplicate_database(&node.S,tiles_grouped,node.depth)){//add to database if necessary and return true if state is new
     *//* cout<<"\tadding state:";
      for(int i=0;i<state_id.size();i++){
	cout<<state_id.at(i)<<",";
      };cout<<endl;*//*
      next.push(node);*/
      /*vector<int> tiles_grouped2;tiles_grouped2.assign(tiles_grouped->begin(),tiles_grouped->end());
      copy_1P_and_AH();
      int correctness_check=do_IDA_heuristic( &node.S,&tiles_grouped2 );
      restore_1P_and_AH();
      if(correctness_check!=node.depth){
	cout<<"state:";print_state(node.S);cout<<endl<<"dist:";printf("depth:%d\n",node.depth);
	cout<<"but regular heuristic search depth:"<<correctness_check<<endl;
	exit(0);
      }*/

    //}
    /*else{
      cout<<"\tstate is duplicated:";
      for(int i=0;i<state_id.size();i++){
	cout<<state_id.at(i)<<",";
      };cout<<endl;
    }*/
  /*}
  //generate_state_id_tiles_grouped(current_state,&state_id,tiles_grouped);
  //cout<<"removing parent_state:"<<state_id<<endl;
  next.pop();//Removing expanded node from queue
  //cout<<"leaving add_to_HST_breadth_first"<<endl;fflush(stdout);
}*/
bool HST::check_duplicate_database(const State* current_state){
  if(nodes->find(StateProxy(current_state))!=nodes->end()){
    return true;
  }
  else{
    return false;
  }
  //cout<<"implement FD-check_duplicate_database"<<endl;exit(0);
}
/*bool HST::check_duplicate_database(const state_var_t *successor_state,const vector<int> *tiles_grouped,unsigned char depth){
  vector<int> state_id;
  bool state_visited;
  static int last_depth;
  int index;
  static int states_at_depth=-1;
  //cout<<"calling check_duplicate"<<endl;fflush(stdout);

  //generate_state_id_tiles_grouped(successor_state,&state_id,tiles_grouped);
  *//*if(next.size()==0){//root node
    cout<<"checking duplicate for root node:"<<endl;
    depth=0;
  }
  else{
    depth=next.front().depth+1;
  }*/

  //get_compact_database_index(successor_state,tiles_grouped,index,-1);
  /*get_compact_database_index();
  //cout<<"index:"<<index<<endl;
  if(pattern_database[index]==255){//new state
    pattern_database.at(index)=depth;
    state_visited=true;
    states_at_depth++;
    if((depth!=last_depth)||(depth==0)){
      //generate_state_id_database(successor_state,&state_id,tiles_grouped);//get positions to state_id
      generate_state_id_database();
      cout<<"state_id:";
      for(int i=0;i<state_id.size();i++){
	cout<<state_id.at(i)<<",";
      }
      printf("depth:%d",pattern_database.at(index));cout<<",index:"<<index<<" added to database"<<endl;fflush(stdout);
      if(depth>0){
	cout<<"states at previous depth:"<<states_at_depth<<endl;
      }
      states_at_depth=-1;
      cout<<"\tcompressed states stored:"<<pattern_database.size()<<endl;
      cout<<"\tstates fully stored:"<<next.size()<<endl;
      last_depth=depth;
    }
  }
  else{//state already visited
    if(depth<pattern_database[index]){//found smaller path
      cout<<"found smaller path for state:";
      for(int i=0;i<state_id.size();i++){
	cout<<state_id.at(i)<<"-";
      }
      cout<<",index:"<<index<<",old_depth:"<<depth;
      printf("new_depth:%d",pattern_database[index]);
      //print_state(*successor_state);
      pattern_database.at(index)=depth;
      state_visited=true;
    }
    else{
      //cout<<"state is duplicated"<<endl;
      state_visited=false;
    }
  }
  return state_visited;
  
  *//*if(visited_database.find(state_id)==visited_database.end()){//new state
    depth=next.front().depth;
    visited_database[state_id]=depth;
    state_visited=true;
    if(depth!=last_depth){
      cout<<"state_id:"<<state_id;printf(",depth:%d",next.front().depth);cout<<" added to database"<<endl;fflush(stdout);
      cout<<"\tcompressed states stored:"<<visited_database.size()<<endl;
      cout<<"\tstates fully stored:"<<next.size()<<endl;
      last_depth=depth;
    }
  }
  else{//state already visited
    //cout<<"state_id:"<<state_id<<" already in database"<<endl;fflush(stdout);
    if(depth<visited_database[state_id]){//found smaller path
      visited_database[state_id]=depth;
    }
    state_visited=false;
  }
  //cout<<"max states stored:"<<visited.max_size()<<endl;
  //cout<<"\treturning check_duplicate"<<endl;fflush(stdout);
  return state_visited;*/
//}
    
/*void HST::get_state_id_from_index(int index,const vector<int> *tiles_grouped,string *state_id){
  cout<<"This is a domain-specific function not applicable to FD"<<endl;exit(0);*/
  /*set<int>::iterator iter;
  long temp_pos,remainder,database_size=MAX_TILE;
  set<int> visited_positions;
  int cardinal;
  char temp_pos_char[MAX_LENGTH];

  //first calculate database size
  for( int i=1;i<tiles_grouped->size(); i++ ){
    database_size=(MAX_TILE-i)*database_size;
  }
  //cout<<"database_size:"<<database_size<<endl;fflush(stdout);
  //cout<<"input index:"<<index<<endl;fflush(stdout);
  for( int i=0;i<tiles_grouped->size(); i++ ){
    temp_pos=((index)/(database_size/(MAX_TILE-i)));
    *//*if(temp_pos>0){
      temp_pos--;
    }*//*
    //cout<<"starting temp_pos:"<<temp_pos<<endl;
    for( iter=visited_positions.begin();iter!=visited_positions.end(); iter++ ){
      if(*iter<=temp_pos){
	temp_pos++;
      }
    }
    //cout<<"final temp_pos:"<<temp_pos<<endl;
    visited_positions.insert(temp_pos);

    sprintf(temp_pos_char,"%d",temp_pos);

    *state_id+=temp_pos_char;*state_id+='-';
    database_size=database_size/(MAX_TILE-i);
    index=index%(database_size);
    //cout<<"new database_size:"<<database_size<<endl;
    //cout<<"new index:"<<index<<endl;
  }
  //cout<<"state_id:"<<*state_id<<endl;*/
//}
    


  
void HST::get_compact_database_index(){
  cout<<"get_compact_database_index is domain-specific, use the FD-equivalent";exit(0);
}
/*void HST::get_compact_database_index(const state_var_t* current_state,const vector<int> *tiles_grouped,int &index,int call_number){
  cout<<"get_compact_database_index is domain-specific, use the FD-equivalent";exit(0);
  *//*
  vector<int> state_id;
  set<int> visited_positions;
  unsigned int step;
  int cardinal;
  
  if(call_number>=0){
    step=multiple_pattern_databases.at(call_number).size();
  }
  else{
    step=pattern_database.size();
  }

  
  index=0;//start from pos 0

  generate_state_id_database(current_state,&state_id,tiles_grouped);//get positions to state_id
  *//*for(int i=0;i<state_id.size();i++){
    cout<<state_id.at(i)<<"-";
  } cout<<endl;*/
  //state_id[0]=0; state_id[1]=1; state_id[2]=2; state_id[3]=3; state_id[4]=4; state_id[5]=5; state_id[6]=6; 
  //cout<<"new state_id:"<<"0,1,2,3,4,5,6"<<endl;
  /*for( int i=0;i<state_id.size(); i++ ){
    step=step/(MAX_TILE-i);//how much index is weighted on each position
    cardinal=0;
    //cout<<"step_size:"<<step<<endl;
    for(int counter=0;counter<MAX_TILE;counter++){
      if(counter==state_id.at(i)){//finished
	//cout<<"pos:"<<state_id.at(i)<<" found at:"<<cardinal<<endl;fflush(stdout);
	visited_positions.insert(state_id.at(i));
	break;//get out of for loop, we are finished
      }
      else if(visited_positions.find(counter)!=visited_positions.end()){//this position is already used so skip it
	//cout<<"pos:"<<counter<<" already used so skipping "<<endl;fflush(stdout);
	continue;
      }
      else{
	cardinal++;//this position is not taken so add one to cardinal
      }
    }
    index+=cardinal*step;
    //cout<<"index is now:"<<index<<endl;fflush(stdout);
  }
  //cout<<"final index is: "<<index<<endl;
  //exit(0);*/
//}
  
/*void HST::print_database_to_file(const vector<int>* tiles_grouped,int number){
  cout<<"not creating databases when combining with FD"<<endl;exit(0);*//*
  FILE *fp;
  char filename[MAX_LENGTH];
  string state_id;
  
  sprintf(filename,"databases/pattern%d_%d.csv",tiles_grouped->size(),number);
  cout<<"printing "<<filename<<endl;
  fp=fopen(filename,"w");
  for(int i=0;i<(tiles_grouped->size()-1);i++){
    fprintf(fp,"%d,",tiles_grouped->at(i));
  }
  fprintf(fp,"%d\n",tiles_grouped->back());

  for(unsigned int i=0;i<pattern_database.size();i++){
    get_state_id_from_index(i,tiles_grouped,&state_id);
    fprintf(fp,"%s,",state_id);
    fprintf(fp,"%d\n",pattern_database.at(i));
    state_id.clear();
  }
  fclose(fp);*/
//}
void HST::read_databases_from_files(){
  cout<<"not reading databases when combining with FD"<<endl;exit(0);/*
  vector<string> filenames;
  vector<string>::iterator iter;
  //char tile_pattern[MAX_LENGTH];
  char tile_pattern[MAX_LENGTH];
  int input_distance;
  char ch;
  long index;
  string temp;

  //h_counter keeps track of how many nodes each heuristic culls  
  int counter_size=int(pow(2.0,float(hset_size))); 
  h_counter.assign(counter_size,0);
  cout<<"h_counter size:"<<h_counter.size()<<endl;
  switch(gcmd_line.start_heuristic){//choose which grouping we use
    case 0://5-5-5 division
      databases=1;
      cout<<"Using 5-5-5 Division"<<endl;
      temp="databases/pattern5_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern5_1.csv";
      filenames.push_back(temp);
      temp="databases/pattern5_2.csv";
      filenames.push_back(temp);
      break;
    case 1://7-7-1 division
      databases=1;
      cout<<"Using 8 Division"<<endl;
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern1_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_1.csv";
      filenames.push_back(temp);
      break;
    case 4://8 tile, 1 division
      databases=1;
      cout<<"Using 8 Division"<<endl;
      temp="databases/pattern8_0.csv";
      filenames.push_back(temp);
      break;
    case 5://8 tile,4-4 division
      databases=2;
      cout<<"Using 4-4 Division"<<endl;
      temp="databases/pattern4_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern4_1.csv";
      filenames.push_back(temp);
      break;
    case 6://7-7-1 division
      databases=5;
      //1-7,11,8-15
      cout<<"Using max of 5 7-7-1 Division"<<endl;
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern11_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_1.csv";
      filenames.push_back(temp);
      //1-7,8,9-15
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern8_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_3.csv";
      filenames.push_back(temp);
      //1-4-5-8-9-12-13,14,2-3-6-7-10-11-15
      temp="databases/pattern7_4.csv";
      filenames.push_back(temp);
      temp="databases/pattern14_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_5.csv";
      filenames.push_back(temp);
      //1-4-5-8-9-12-13,2,3-6-7-10-11-14-15
      temp="databases/pattern7_4.csv";
      filenames.push_back(temp);
      temp="databases/pattern2_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_6.csv";
      filenames.push_back(temp);
      //1-4-5-8-9-12-13,3,2-6-7-10-11-14-15
      temp="databases/pattern7_4.csv";
      filenames.push_back(temp);
      temp="databases/pattern3_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_7.csv";
      filenames.push_back(temp);
      break;
    case 7://7-7-1 division, 4 databases
      databases=4;
      //1-7,11,8-15
      cout<<"Using max of 5 7-7-1 Division"<<endl;
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern11_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_1.csv";
      filenames.push_back(temp);
      //1-7,8,9-15
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern8_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_3.csv";
      filenames.push_back(temp);
      //1-4-5-8-9-12-13,14,2-3-6-7-10-11-15
      temp="databases/pattern7_4.csv";
      filenames.push_back(temp);
      temp="databases/pattern14_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_5.csv";
      filenames.push_back(temp);
      //1-4-5-8-9-12-13,2,3-6-7-10-11-14-15
      temp="databases/pattern7_4.csv";
      filenames.push_back(temp);
      temp="databases/pattern2_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_6.csv";
      filenames.push_back(temp);
      break;
    case 8://7-7-1 division,3 databases
      databases=3;
      //1-7,11,8-15
      cout<<"Using max of 5 7-7-1 Division"<<endl;
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern11_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_1.csv";
      filenames.push_back(temp);
      //1-7,8,9-15
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern8_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_3.csv";
      filenames.push_back(temp);
      //1-4-5-8-9-12-13,14,2-3-6-7-10-11-15
      temp="databases/pattern7_4.csv";
      filenames.push_back(temp);
      temp="databases/pattern14_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_5.csv";
      filenames.push_back(temp);
      break;
    case 9://7-7-1 division,2 databases
      databases=2;
      //1-7,11,8-15
      cout<<"Using max of 5 7-7-1 Division"<<endl;
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern11_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_1.csv";
      filenames.push_back(temp);
      //1-7,8,9-15
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern8_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_3.csv";
      filenames.push_back(temp);
      break;
    case 10://7-7-1 division,1 database
      databases=1;
      //1-7,11,8-15
      cout<<"Using max of 5 7-7-1 Division"<<endl;
      temp="databases/pattern7_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern11_0.csv";
      filenames.push_back(temp);
      temp="databases/pattern7_1.csv";
      filenames.push_back(temp);
      break;
    default:
      break;
  };

    
  multiple_pattern_databases.resize(filenames.size());
  int filenumber=0;
  for (iter=filenames.begin();iter!=filenames.end();iter++){
    FILE *fp;
    fp=fopen(iter->c_str(),"r");
    //fscanf(fp,"%s",tile_pattern);
    fgets(tile_pattern,MAX_LENGTH,fp);
    string tile_patterns(tile_pattern);
    cout<<"file:"<<*iter<<" working with file pattern:"<<tile_pattern<<endl;
    int dimensions=1;int pos=0;long database_size=MAX_TILE;
    while(true){
      pos=tile_patterns.find(',',pos);
      if (pos== string::npos){
	break;
      }
      database_size=(MAX_TILE-dimensions)*database_size;
      pos++;
      dimensions++;
    }
    //database_size=(MAX_TILE-dimensions)*database_size;
    cout<<"number of dimensions:"<<dimensions<<" in "<<tile_pattern<<endl;
    cout<<"database size:"<<database_size<<endl;
    multiple_pattern_databases.at(filenumber).reserve(database_size);
    //now read patterns into vector until EOF  
     while( fscanf(fp,"%d",&input_distance)!=EOF ){
       sprintf(&ch,"%c",input_distance);
       multiple_pattern_databases.at(filenumber).push_back(ch);
       //printf("added %d\n",multiple_pattern_databases.at(filenumber).back());
     }
     cout<<"finshed loading database "<<filenumber<<",total patterns loaded:"<<multiple_pattern_databases.at(filenumber).size()<<endl;
     fclose(fp);
     filenumber++;
  }*/
}  
  
int HST::get_database_value(int index,int call_number){
  return multiple_pattern_databases.at(call_number).at(index);
}
  
void HST::add_to_counter(boost::dynamic_bitset<> *h_bitset, unsigned long nodes){
  //cout<<"h_counter["<<h_bitset;fflush(stdout);
  h_counter[hset_size-h_bitset->count()][*h_bitset]+=nodes;
  //cout<<"]:"<<h_counter[hset_size-h_bitset->count()][*h_bitset]<<endl;
}
void HST::add_to_substract_counter(boost::dynamic_bitset<> *h_bitset, unsigned long nodes){
  //cout<<"h_counter_substract["<<hset_size-h_bitset->count()<<"]["<<*h_bitset;fflush(stdout);
  h_counter_substract[hset_size-h_bitset->count()][*h_bitset]+=nodes;
  //h_counter_substract[hset_size-h_bitset->count()][*h_bitset]+=nodes-nodes;
  //cout<<"]:"<<h_counter_substract[hset_size-h_bitset->count()][*h_bitset]<<endl;
}
void HST::substract_from_counter(boost::dynamic_bitset<> *h_bitset, unsigned nodes){//inc case dup_check stops evaluation from hapenning
  h_counter[hset_size-h_bitset->count()][*h_bitset]-=nodes;
}

void HST::print_h_counters(){
 for ( unsigned level=0 ;level<h_counter.size(); level++ ){
   cout<<"level:"<<level<<",counters in the lattice:"<<h_counter[level].size()<<endl;
   for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
     cout<<"it->first"<<","<<it_h_values->first<<","<<it_h_values->second<<endl;
     if(h_counter_substract[level].find(it_h_values->first)!=h_counter_substract[level].end()){
       cout<<"h_counter_substract"<<","<<it_h_values->first<<","<<h_counter_substract[level][it_h_values->first];
     }
   }
   cout<<"level:"<<level<<",substract_counters in the lattice:"<<h_counter[level].size()<<endl;
   for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter_substract[level].begin(); it_h_values != h_counter_substract[level].end(); it_h_values++){
     cout<<"substract_it->first"<<","<<it_h_values->first<<","<<it_h_values->second<<endl;
   }
 }
}


  
void HST::fprint_h_counters(){
  /*char filename[MAX_LENGTH];
  static bool first_call=true;
  //static int last_F=gcmd_line.culling;
  static int last_F=current_F_bound;
  static int problem_index=0;

  problem_index++;
  sprintf(filename,"output/h_counter.csv");
  ofstream myfile ( filename, ios::app );
  cout<<"printing "<<filename<<endl;
  if(first_call){  
    myfile<<"N\tF\titer_index\t";
    for(int i=0;i<databases;i++){
      myfile<<"h"<<databases-i-1<<"\t";
    }
    myfile<<"specific_count"<<"\ttotal_count"<<"\tpredicted_time_reduction"<<endl;
  }
  int iteration_index=0;
  for( int i =0; i<h_counter.size(); i++ ) {
    if(last_F!=current_F_bound){//new iteration
      iteration_index=1;
      last_F=current_F_bound;
    }
    else{
      iteration_index++;
    }

    bitset<HEURISTICS> bs( i );
    myfile<<problem_index<<","<<current_F_bound<<","<<iteration_index<<",";
 // display that bitset, too
    for( int j = (databases-1); j >= 0; j-- ) {
      myfile << bs[j] << ",";
    }
    myfile<<h_counter.at(i)<<",";
    //long int nodes_generated_h_selection=total_generated_by_dropped_heuristics(bs);
    //myfile<<nodes_generated_h_selection-h_counter.at(0)<<",";
    //double predicted_reduction=double(nodes_generated_h_selection-h_counter.at(0))*double(EXPANSION_TIME+HEUR_TIME*bs.count())-double(h_counter.at(0))*HEUR_TIME*(hset_size-bs.count());
    //myfile<<predicted_reduction<<endl;
    myfile<<endl;

    if((bs.count()==0)||(bs.count()==databases)){
      continue;//skipping 00000&11111
    }
    if(best_heuristic.find(bs.count())==best_heuristic.end()){
      cout<<"best heuristic for counter"<<bs.count()<<" is empty"<<endl;
      //best_heuristic[bs.count()]=pair<bitset<hset_size>,long>(bs,INT_MAX/2);
    }

    *//*if(best_heuristic[bs.count()].second>(nodes_generated_h_selection-h_counter.at(0))){
      best_heuristic[bs.count()].first=bs;
      best_heuristic[bs.count()].second=nodes_generated_h_selection-h_counter.at(0);
      }*//*
  }
  myfile.close();
  
  sprintf(filename,"output/best_h.csv");
  ofstream myfile2 ( filename, ios::app );
  if(first_call){  
    cout<<"printing "<<filename<<endl;
    myfile2<<"N\tF\t";
    for(int i=0;i<databases;i++){
      myfile2<<"h"<<databases-i-1;
    }
    myfile2<<"\tnodes_added"<<endl;
    first_call=false;
  }

  map<int,pair<bitset<HEURISTICS>,long> >::iterator best_heuristic_iter;
	
  for( best_heuristic_iter = best_heuristic.begin(); best_heuristic_iter != best_heuristic.end(); best_heuristic_iter++ ) {
    myfile2<<problem_index<<","<<current_F_bound<<",";
    for( int j = (databases-1); j >= 0; j-- ) {
      myfile2 << (best_heuristic_iter->second).first[j];
    }
    myfile2<<","<<(best_heuristic_iter->second).second<<endl;
  }
  myfile2.close();*/
}
/*long unsigned HST::total_generated_by_dropped_heuristics(bitset<HEURISTICS> bs){
  long unsigned total=0;//minimum number
  set<int> heuristics_active;
  bool add;
  //set<int>::iterator iter;
    
  //cout<<"generated by all heuristics:"<<h_counter.at(0);
  if(bs.none()){
    return INT_MAX/2;
  }
  for( int i = (databases-1); i >= 0; i-- ) {
    if(bs[i]==1){
      heuristics_active.insert(i);//we will skip this positions
      //cout<<"skipping"<<i;
    }
  }
  for( int i=0;i<h_counter.size();i++){
    bitset<hset_size> bs2( i );
    add=true;
    //cout<<"checking heuristics:"<<bs2<<endl;
    for( int j = (databases-1); j >= 0; j-- ) {
      if(heuristics_active.find(j)!=heuristics_active.end()){//we verify this position is 0
	if(bs2[j]==1){//not adding
	  add=false;
	  break;
	}
      }
    }
    if(add){
      //cout<<"adding pos:"<<i<<endl;
      total+=h_counter.at(i);
    }
  }
  return total;
}*/
    
unsigned HST::get_iter_index(){
  return iter_index;
}
void HST::inc_iter_index(){
  iter_index++;
  /*if(iter_index>12){
    cout<<"exiting for debuggin, iter>"<<iter_index-1<<endl;
    exit(0);
  }*/
  cout<<"Starting Iteration:"<<iter_index<<endl;
}
void HST::reset_iter_index(){
  iter_index=0;
  cout<<"starting problem"<<endl;
  problem_index++;
}
void HST::print_current_state(){
  lsearch_space.back().print_state();
}
void HST::set_F_bound(unsigned F){
  current_F_bound=F;
}
unsigned HST::get_F_bound(){
  return current_F_bound;
}
void HST::set_earliest_depth_h_culled(int database,int depth){
  earliest_depth_h_culled.at(database)=depth;
}
int HST::get_earliest_depth_h_culled(int database){
  //cout<<"earliest_depth_h_culled.size:"<<earliest_depth_h_culled.size()<<endl;
  return earliest_depth_h_culled.at(database);
}
bool HST::checking_consistency(){
  return check_consistency;
}
  
int HST::get_current_children_number(){
  return lsearch_space.back().get_children_number();
}

void HST::print_final_size_counters(bool actual_print,int degree){
  vector<unsigned> sig_bits;
  unsigned pos_to_move=0;
  boost::dynamic_bitset<> current_h_comb(hset_size);
  //static bool first_call=true;
  bool calculate_h_comb_maps=false;
  unsigned long partial_HSTs_count=0;

  if(h_comb_map.size()==0){
    calculate_h_comb_maps=true;
  }
  cout<<"calling print_final_size_counters"<<endl;
  //For calculating HUST compression factor
  //double current_time=0;
  calculate_time_costs();
  for(int level=0;level<degree;level++){
    cout<<"\t working on level"<<level<<endl;
    partial_HSTs_count=0;
    for(int index=0;index<h_comb_to_degree.at(level).size();index++){
     HSTs_total_size+=h_comb_to_degree.at(level).at(index);
     partial_HSTs_count+=h_comb_to_degree.at(level).at(index);
    }
    HSTs_total_time+=partial_HSTs_count*time_cost.at(level)*pow(10,7);
  }
  if(actual_print) 
    cout<<"Printing final size counters"<<endl;
  else 
    cout<<"Calculate final size counters"<<endl;
#ifdef CROSSOVER_PREDICTION
  //initialize variables for crossover prediction
     double best_iter_time=std::numeric_limits<double>::max();
     bool estimated_time,best_estimated;
     boost::dynamic_bitset<> temp_h_comb;
     calculate_time_costs();
     cout<<"starting best time:"<<best_iter_time<<endl;fflush(stdout);
#endif
     //cout<<"hcomb_map2[0][0]"<<h_comb_map2[0][0]<<endl;fflush(stdout);
  for (unsigned level=0;level<degree;level++){
    sig_bits.resize(level+1);
    current_h_comb.reset();
    //initialize sig_bits to starting position
    for(unsigned i=0;i<sig_bits.size();i++){
      sig_bits.at(i)=i;
      current_h_comb.set(i);
    }
    if(actual_print) cout<<"Working on level:"<<level<<endl;

    for (unsigned long h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
      if(h_index==0){
	if(actual_print) cout<<"\th_index:"<<h_index<<",current_h_comb "<<current_h_comb<<":"<<h_comb_to_degree.at(level).at(h_index)<<endl;
      }
      else if(sig_bits.size()==1){//only one sig bit so bitset is obvious
	//cout<<"\t setting sig_bits["<<sig_bits[0]+1<<endl;
	current_h_comb.reset(sig_bits[0]);
	current_h_comb.set(sig_bits[0]+1);
	sig_bits[0]++;
	if(actual_print) cout<<"\th_index:"<<h_index<<",current_h_comb "<<current_h_comb<<":"<<h_comb_to_degree.at(level).at(h_index)<<endl;
      }
      else{

      //find new pos of lowest bit
      pos_to_move=1;
      current_h_comb.reset(sig_bits[0]);//Will be set again 
      for(unsigned i=1;i<sig_bits.size();i++){
	if(sig_bits[i]==sig_bits[0]+i){
	  pos_to_move++;
	  current_h_comb.reset(sig_bits[i]);//Will be set again 
	  //cout<<"\t\tResetting bit:"<<sig_bits[i]<<endl;fflush(stdout);
	}
	else{
	  //current_h_comb.reset(sig_bits[i]);//Will be set again 
	  //cout<<"\t\tResetting bit:"<<sig_bits[i]<<endl;fflush(stdout);
	  break;//get out as no more bits need to be moved
	}
      }
      //set new pos of lowest bit
      //cout<<"\tPos to move:"<<pos_to_move<<",Setting bit:"<<sig_bits[0]+pos_to_move<<endl;fflush(stdout);
      current_h_comb.set(sig_bits[0]+pos_to_move);
      
      //cout<<"\t\tSetting highest bit "<<pos_to_move-1<<" to:";fflush(stdout);
      sig_bits[pos_to_move-1]++;
      //cout<<sig_bits[pos_to_move-1]<<endl;fflush(stdout);

      //set the lowest bit positions up to the new bit
      for(unsigned i=1;i<pos_to_move;i++){
	current_h_comb.set(i-1);
	sig_bits[i-1]=i-1;
      }
      //cout<<"\th_index:"<<h_index<<",current_h_comb "<<current_h_comb<<":"<<h_comb_to_degree.at(level).at(h_index)<<endl;
      /*for(unsigned i=0;i<sig_bits.size();i++){
	cout<<"\t\tsig_bit["<<i<<"]:"<<sig_bits.at(i)<<endl;
      }*/
      }
      if((level==0)&&(h_comb_to_degree.at(level).at(h_index)>CAPPING_LIMIT)&&\
	 (Current_RIDA_Phase==SAMPLING_PHASE)){

	culling_iteration[current_h_comb]=iter_index;
	//cout<<"h_index:"<<h_index<<",capping "<<current_h_comb<<", current value:"<<h_comb_to_degree.at(level).at(h_index)<<",capping limit:"<<CAPPING_LIMIT<<",culling_iteration:"<<culling_iteration[current_h_comb]<<endl;
	h_capped.at(h_index)=true;
	//Now calculate all the HBF who are affected by this heuristic
	boost::dynamic_bitset<> h_comb(0);
	//unsigned temp_index=0;
	for (unsigned level2=0;level2<degree;level2++){
	  for (unsigned long h_index2=0;h_index2<h_comb_to_degree.at(level2).size();h_index2++){
	    h_comb=h_comb_map2[level2][h_index2];
	  //cout<<"\tchecking h_comb:"<<h_comb<<endl;
	    if(HBF.find(h_comb)==HBF.end()&&h_comb.test(h_index)){//update h_comb if not previously capped and this heuristic applies to it
	      Culling_size[h_comb]=double(h_comb_to_degree.at(h_comb.count()-1).at(h_comb_map[h_comb]));
	      if(Culling_size[h_comb]<1000){
		//cout<<"Warning, very few nodes for h_comb:"<<h_comb<<",size:"<<Culling_size[h_comb]<<endl;
	      }
	      HBF[h_comb]=double(h_comb_to_degree.at(h_comb.count()-1).at(h_comb_map[h_comb]))/double(prev_val[h_comb]);
	      if(actual_print) cout<<"\t\tlevel:"<<h_comb.count()-1<<",HBF["<<h_comb<<"]="<<h_comb_to_degree.at(h_comb.count()-1).at(h_comb_map[h_comb])\
		<<"/"<<prev_val[h_comb]<<"="<<HBF[h_comb]<<"Culling_size["<<h_comb<<"]"<<Culling_size[h_comb]<<endl;
	      culling_iteration[h_comb]=iter_index;
	      if(actual_print) cout<<"culling_iteration["<<h_comb<<"]:"<<culling_iteration[h_comb]<<endl;
	    }
	  }
	}
	if(actual_print) cout<<"now checking MAXTREE"<<endl;
	//also need to upadate MAXTREE, as we might not reach it if degree!=hset_size
	h_comb.set();
	if(HBF[h_comb]==0&&h_comb.test(h_index)){//update h_comb if not previously capped and this heuristic applies to it
	  //Culling_size[h_comb]=double((h_counter.at(hset_size-1)[h_comb]).second+1+initial_children);
	  Culling_size[h_comb]=double(h_counter[hset_size].begin()->second+1);
	  HBF[h_comb]=Culling_size[h_comb]/double(prev_val[h_comb]);
	  //if(actual_print) cout<<"\t\tlevel:"<<h_comb.count()-1<<",HBF["<<h_comb<<"]="<<h_comb_to_degree.at(h_comb.count()-1).at(h_comb_map[h_comb])\ <<"/"<<prev_val[h_comb]<<"="<<HBF[h_comb]<<"Culling_size["<<h_comb<<"]"<<Culling_size[h_comb]<<endl;
	  culling_iteration[h_comb]=iter_index;
	  if(actual_print) cout<<"culling_iteration["<<h_comb<<"]:"<<culling_iteration[h_comb]<<endl;
	}

      }
      if(calculate_h_comb_maps==true){
	//cout<<"Populating h_comb_map"<<endl;fflush(stdout);
	//cout<<"h_comb_map["<<current_h_comb<<"]:"<<h_index<<endl;
	h_comb_map[current_h_comb]=h_index;
	//cout<<"h_comb_map["<<current_h_comb<<"]:"<<h_comb_map[current_h_comb]<<endl;fflush(stdout);
	if(h_comb_map2.size()==0){
	  map<unsigned long,boost::dynamic_bitset<> > map_temp;
	  h_comb_map2.assign(degree,map_temp);
	}
	h_comb_map2[level][h_index]=current_h_comb;
	//cout<<"\th_comb_map2["<<level<<","<<h_index<<"]:"<<current_h_comb<<endl;fflush(stdout);
      }
      prev_val[current_h_comb]=double(h_comb_to_degree.at(level).at(h_index));
      //cout<<"prev_val["<<current_h_comb<<"]:"<<prev_val[current_h_comb]<<endl;fflush(stdout);

#ifdef CROSSOVER_PREDICTION
      //Now I calculate the best h_comb for the iteration as it is of interest to study crossovers
      estimated_time=false;
      if(culling_iteration.find(h_comb_map2[level][h_index])==culling_iteration.end()){
	current_time=h_comb_to_degree.at(level).at(h_index)*time_cost.at(level);
      }
      else{//the h_comb is culled so we need to estimate new time
	estimated_time=true;
	current_time=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],iter_index-culling_iteration[h_comb_map2[level][h_index]]-1)*time_cost[level];
	//cout<<"Problem:"<<problem_index<<",h_comb:"<<h_comb_map2[level][h_index];
	//cout<<",estimated_time:"<<estimated_time<<",iteration_index:"<<iteration_index<<",culling_iteration:"<<culling_iteration[h_comb_map2[level][h_index]]<<",current_time:"<<current_time<<endl;
      }
      if(best_iter_time>current_time){
	//cout<<"\th_comb_to_degree["<<level<<"]["<<h_index<<"]:"<<h_comb_to_degree.at(level).at(h_index)<<",";
	//cout<<"current_time:"<<current_time<<endl;fflush(stdout);
	best_iter_time=current_time;
	temp_h_comb=h_comb_map2[level][h_index];
	best_estimated=estimated_time;
	//if(h_comb_map2[level][h_index]!=best_prev_iter_heurstic.back()){
	//}
      }
#endif
    }
  }
  //need to store previous values for MAXTREE if degree<hset_size
  if(h_counter.at(hset_size).size()>0&&degree!=hset_size){
    boost::dynamic_bitset<> temp_comb(hset_size);
    temp_comb.set();	
    if(iter_index>1){
      cout<<"Prev_val for MAXTREE h_comb calculated from h_counter["<<h_counter[hset_size].begin()->first<<"]:"<<prev_val[temp_comb]<<",MAXTREE_HBF"<<double(h_counter[hset_size].begin()->second)/double(prev_val[temp_comb])<<endl;
    }
    prev_val[temp_comb]=double(h_counter[hset_size].begin()->second+1);
    cout<<"MAXTREE h_comb(no root+children) calculated from h_counter["<<h_counter[hset_size].begin()->first<<"]:"<<h_counter[hset_size].begin()->second<<endl;
  }
  
  /*if(first_call){
    first_call=false;
  }*/
#ifdef CROSSOVER_PREDICTION
  best_prev_iter_heuristic.push_back(temp_h_comb);
  cout<<"Problem:"<<problem_index<<"best_estimated time:"<<best_estimated<<",Best heuristic for prev_iter:"<<best_prev_iter_heuristic.size()<<" is "<<best_prev_iter_heuristic.back();fflush(stdout);
  cout<<" ,best time:"<<best_iter_time<<endl;fflush(stdout);
#endif
}

void HST::print_counters(){
  cout<<"Here is the list of h_counters:"<<endl;
  for(int level=0;level<h_counter.size();level++){
    cout<<"\tLevel:"<<level<<endl;
      for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
	cout<<"h_counter["<<level<<"]"<<"["<<it_h_values->first<<"]:"<<it_h_values->second<<endl;fflush(stdout);
      }
  }
}

void HST::calculate_heuristics_to_degree(unsigned degree){
  cout<<"Starting calculate_heuristics_to_degree "<<degree<<endl;
  /*  cout<<"Here is the list of h_counters:"<<endl;
  for(int level=0;level<h_counter.size();level++){
    cout<<"\tLevel:"<<level<<endl;
      for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
	cout<<"h_counter["<<level<<"]"<<"["<<it_h_values->first<<"]:"<<it_h_values->second<<endl;fflush(stdout);
     if(h_counter_substract[level].find(it_h_values->first)!=h_counter_substract[level].end()){
       cout<<"h_counter_substract"<<","<<it_h_values->first<<","<<h_counter_substract[level][it_h_values->first];
     }
      }
  }*/
  unsigned H_Comb_size=0;
  boost::dynamic_bitset<> first(hset_size);first.set();
  vector<int> indx;
  //bitset<hset_size> H_comb;
  //string alpha;
  //char temp[2];
  vector<int> aval_pos;
  long total=0;
  long total_CCs=0;
 
  //unsigned n=hset_size;
  //unsigned k=0,j=1;
  size_t pos=0;
  unsigned sig_bit=1;

  h_comb_to_degree.resize(degree);
  if(h_counter.size()<1){
    cout<<"Initializing h_counter C11...1 to dummy 0 value"<<endl;
    h_counter.resize(1);
  }

  //calculating number of initial children
  //vector<const Operator *> applicable_ops;
  //g_successor_generator->generate_applicable_ops(*g_initial_state, applicable_ops);
  for (unsigned i=0;i<degree;i++){
    cout<<hset_size<<"\theuristics, degree "<<i+1<<", Combinations:"<<comb(hset_size,i+1)<<endl;fflush(stdout);
    //cout<<"adding initial children and root node"<<",value:"<<1+applicable_ops.size()<<endl;fflush(stdout);
    //h_comb_to_degree.at(i).assign(comb(hset_size,i+1),1+applicable_ops.size());//adding intial counter 0000
    h_comb_to_degree.at(i).assign(comb(hset_size,i+1),0);//root is included in the counters
    H_Comb_size+=comb(hset_size,i+1);
  }
  //cout<<hset_size<<"\theuristics, degree "<<degree<<", total_combinations:"<<H_Comb_size<<endl;

  for (unsigned level=1;level<h_counter.size();level++){
      //cout<<"applies to levels "<< it_h_values->first.count()<<"to 1"<<endl;
      //cout<<"\tlevel:"<<level<<",Processing "<<h_counter[level].size()<<" Culprit Counters"<<endl;
      total_CCs+=h_counter[level].size();
      if(h_counter.at(level).size()==0){
	continue;
      }
    for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
      cout<<"\tCC counter"<<","<<it_h_values->first;cout<<",size:"<<it_h_values->second<<endl;
      cout<<"applies to levels "<< hset_size-it_h_values->first.count()<<"to 1"<<endl;
      long adding=it_h_values->second;
      cout<<"\t initial size:"<<adding<<endl;

      /*if(h_counter_substract[level].find(it_h_values->first)!=h_counter_substract[level].end()){
	adding-=h_counter_substract[level][it_h_values->first];
	//cout<<"\t after substract(,"<<h_counter_substract[level][it_h_values->first]<<",)final size:"<<adding<<endl;
      }*/
      
//Set which bits are available      
      aval_pos.clear();
      for(unsigned i=0;i<(it_h_values->first).size();i++){
	if(!((it_h_values->first).test(i))){
	  aval_pos.push_back(i);
	  //cout<<"\tpos "<<i<<"is available"<<endl;
	}
      }

      //cout<<"aval_pos size:"<<aval_pos.size()<<endl;
      int n=aval_pos.size();
      int j=1;
      //int k=aval_pos.size();
      //int sig_bits_remaining=k;

      for(int twk=j;twk<=min(degree,unsigned(aval_pos.size()));twk++){
	cout<<"\tWorking on level:"<<twk<<endl;
	//H_comb.reset();
	int r=twk;
	//cout<<"n:"<<n<<"r"<<r;
	total+=comb(n,r);
	//cout<<",total combinations:"<<total<<endl;fflush(stdout);
	//int ccount=1;
	bool done=true;

	for(int iwk=0;iwk<r;iwk++)indx.push_back(iwk);
	while(done){
	  done=false;
	  //cout <<"\t"<< ccount++ << " of " << total << " ";fflush(stdout);
	  //H_comb.reset();
	  pos=0;
	  sig_bit=1;
	  for(int owk=0;owk<r;owk++){
		  //H_comb.set(aval_pos.at(indx[owk]));
		  //cout<<"owk:"<<owk<<",H_comb:"<<H_comb<<endl;
	    //cout <<"index:"<< aval_pos.at(indx[owk]) <<",sig_bit: "<<sig_bit;fflush(stdout);
	    pos+=comb(aval_pos.at(indx[owk]),sig_bit);
	    //cout<<",pos:"<<comb(aval_pos.at(indx[owk]),sig_bit);
	    sig_bit++;
	  }
	  //cout<<"final_pos:"<<pos<<endl;
	  //cout<<"out of for loop"<<endl;fflush(stdout);
	  //cout<<"->"<<H_comb;fflush(stdout);
	  /*sig_bits_remaining=r;
	  for(int i=(H_comb.size()-1);i>=0;i--){
	    //cout<<"\t\ti:"<<i;fflush(stdout);
	    if(H_comb.test(i)){
	      if(sig_bits_remaining>1){
		pos+=comb(i,sig_bits_remaining);
	      }
	      else{
		pos+=i;
	      }
	      sig_bits_remaining--;
	    }
	      
	    //cout<<",sig_bits_remaining:"<<sig_bits_remaining<<endl;fflush(stdout);
	    if(sig_bits_remaining==0||i==1){
	      //cout<<"exiting, no more significant bits"<<endl;fflush(stdout);
	      break;//no more bits to count
	    }
	  }*/
	  //cout<<",at pos:"<<pos<<",level:"<<twk<<",max_pos:";fflush(stdout);cout<<h_comb_to_degree.at(twk-1).size()<<endl;fflush(stdout);
	  h_comb_to_degree.at(twk-1).at(pos)+=adding;
	  //cout<<"\tAdded CC"<<it_h_values->first<<",value:"<<it_h_values->second<<" to level:"<<twk<<" to pos:"<<pos<<endl;fflush(stdout);

	
	  for(int iwk=r-1;iwk>=0;iwk--){
	    //cout<<"\t\t\tiwk:"<<iwk<<"ind[iwk]:"<<indx[iwk]<<"(n-1)-(r-iwk)"<<(n-1)-(r-iwk)<<endl;fflush(stdout);
	    if(indx[iwk]<=(n-1)-(r-iwk)){
		    indx[iwk]++;
		    for(int swk=iwk+1;swk<r;swk++){
			    indx[swk]=indx[swk-1]+1;
		    }
		    iwk=-1;
		    done=true;
		    //cout<<"\tFinished with subset"<<endl;
	    }
	  }
	}
	cout << "\t--------------------------- " << endl;
	indx.clear();
	}

    }
  }
  //Now print totals to sql database
  for (unsigned level=0;level<degree;level++){
    for(unsigned h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
      //outputFile<<g_plan_filename<<"\t"<<h_comb_to_degree.at(level).at(h_index)<<"\t"<<calculate_time_costs_specific(best_h_comb,orig_heuristics)<<"\t"<<node_gen_and_exp_cost<<"\t"<<endl;
      //cout<<"\th_comb_to_degree["<<level<<"]["<<h_index<<"]:"<<h_comb_to_degree.at(level).at(h_index)<<endl;
    }
  }
  cout<<"Total Additions:"<< scientific <<total<<",Total CCs:"<<total_CCs<<endl;
}
/*void HST::calculate_heuristics_to_degree(unsigned degree){
  cout<<"need to add heuristics vector to calculate_heuristics_to_degree input so we know the specific time costs per heuristic"<<",degree:"<<degree<<endl;
  exit(0);
}*/

long HST::calculate_specific_heuristic(boost::dynamic_bitset<> h_comb){
  bool add_counter=true;
  double h_comb_gen_nodes=0;
  long adding=0;long substracting=0;
  for(int level=0;level<h_counter.size();level++){
      if(h_counter.at(level).size()==0){
	continue;
      }
      for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
	for(int heur=0;heur<h_comb.size();heur++){
	  if(it_h_values->first.test(heur)&&h_comb.test(heur)){//if indiv heuristic is active and culling the counter then this counter does not apply to the h_comb
	    add_counter=false;
	    break;
	  }
	}
	if(add_counter){
	  h_comb_gen_nodes+=it_h_values->second;
	  adding+=it_h_values->second;
	}
      }
      //also need to account for substract counters
      for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter_substract[level].begin(); it_h_values != h_counter_substract[level].end(); it_h_values++){
	for(int heur=0;heur<h_comb.size();heur++){
	  if(it_h_values->first.test(heur)&&h_comb.test(heur)){//if indiv heuristic is active and culling the counter then this counter does not apply to the h_comb
	    add_counter=false;
	    break;
	  }
	}
	if(add_counter){//we substract because it is the substract counters!
	  h_comb_gen_nodes-=it_h_values->second;
	  substracting+=it_h_values->second;
	}
      }
  }
 //we substract because it is the substract counters! }
  cout<<"h_comb:"<<h_comb<<" generates "<<h_comb_gen_nodes<<" nodes"<<",adding:"<<adding<<",substracting:"<<substracting<<endl;
  return h_comb_gen_nodes;
}
void HST::calculate_heuristics_to_degree(unsigned degree,const vector<Heuristic *> orig_heuristics){
  cout<<"Starting calculate_heuristics_to_degree "<<degree<<endl;
  cout<<"Here is the list of h_counters:"<<endl;
  for(int level=0;level<h_counter.size();level++){
    cout<<"\tLevel:"<<level<<endl;
      for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
	cout<<"h_counter["<<level<<"]"<<"["<<it_h_values->first<<"]:"<<it_h_values->second<<endl;fflush(stdout);
       if(h_counter_substract[level].find(it_h_values->first)!=h_counter_substract[level].end()){
	 cout<<"h_counter_substract"<<","<<it_h_values->first<<","<<h_counter_substract[level][it_h_values->first];
       }
      }
  }
  boost::dynamic_bitset<> prev_best_h_comb=best_h_comb;
  cout<<"prev_best_h_comb:"<<prev_best_h_comb;
  cout<<"F_bound:,"<<get_current_F_bound()<<",strong_heurs(real_pos):,";
  for (set<int>::iterator it= strong_heur.begin(); it != strong_heur.end(); it++){
    cout<<real_heur_pos[*it]<<",";
  }
  cout<<endl;
  unsigned H_Comb_size=0;
  //boost::dynamic_bitset<> first(hset_size);first.set();
  vector<int> indx;
  //bitset<hset_size> H_comb;
  //string alpha;
  //char temp[2];
  vector<int> aval_pos;
  long total=0;
  long total_CCs=0;
 
  //unsigned n=hset_size;
  //unsigned k=0,j=1;
  size_t pos=0;
  unsigned sig_bit=1;

  //h_comb_to_degree.resize(degree);
  h_comb_to_degree.clear();
  if(h_counter.size()<1){
    cout<<"Initializing h_counter C11...1 to dummy 0 value"<<endl;
    h_counter.resize(1);
  }

  //calculating number of initial children
  //vector<const Operator *> applicable_ops;
  //g_successor_generator->generate_applicable_ops(*g_initial_state, applicable_ops);
  for (unsigned i=0;i<degree;i++){
    cout<<hset_size<<"\theuristics, degree "<<i+1<<", Combinations:"<<comb(hset_size,i+1)<<endl;fflush(stdout);
    //cout<<"adding initial children and root node"<<",value:"<<1+applicable_ops.size()<<endl;fflush(stdout);
    //h_comb_to_degree.at(i).assign(comb(hset_size,i+1),1+applicable_ops.size());//adding intial counter 0000
    if(comb(hset_size,i+1)>maximum_combination_limit){
      cout<<"too many combinations, actual:"<<comb(hset_size,i+1)<<",but maximum allowed:"<<maximum_combination_limit<<",so setting degree to "<<i<<endl;
      degree=i; 
      break;
    }
    else if((i+1)>Degree){
      cout<<"Max degree allowed:"<<Degree<<endl;
      break;
    }
    h_comb_to_degree.resize(i+1);
    h_comb_to_degree.at(i).assign(comb(hset_size,i+1),0);//root is included in the counters
    H_Comb_size+=comb(hset_size,i+1);
  }
  cout<<hset_size<<"\theuristics, degree "<<degree<<", total_combinations:"<<H_Comb_size<<endl;

  for (unsigned level=1;level<h_counter.size();level++){
      //cout<<"applies to levels "<< it_h_values->first.count()<<"to 1"<<endl;
      //cout<<"\tlevel:"<<level<<",Processing "<<h_counter[level].size()<<" Culprit Counters"<<endl;
      total_CCs+=h_counter[level].size();
      if(h_counter.at(level).size()==0){
	continue;
      }
    for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
      //cout<<"\tCC counter"<<","<<it_h_values->first;cout<<",size:"<<it_h_values->second<<endl;
      //cout<<"applies to levels "<< hset_size-it_h_values->first.count()<<"to 1"<<endl;
      
      long adding=it_h_values->second;
/*        if(h_counter_substract[level].find(it_h_values->first)!=h_counter_substract[level].end()){
	adding-=h_counter_substract[level][it_h_values->first];
	//cout<<"\t after substract(,"<<h_counter_substract[level][it_h_values->first]<<",)final size:"<<adding<<endl;
      }
      //skip counter if now its modified value is 0
      if(adding==0){
	continue;
      }*/
          
//Set which bits are available      
      aval_pos.clear();
      for(unsigned i=0;i<(it_h_values->first).size();i++){
	if(!((it_h_values->first).test(i))){
	  aval_pos.push_back(i);
	  //cout<<"\tpos "<<i<<"is available"<<endl;
	}
      }

      //cout<<"aval_pos size:"<<aval_pos.size()<<endl;
      int n=aval_pos.size();
      int j=1;
      //int k=aval_pos.size();
      //int sig_bits_remaining=k;

      for(int twk=j;twk<=min(degree,unsigned(aval_pos.size()));twk++){
	//cout<<"\tWorking on level:"<<twk<<endl;
	//H_comb.reset();
	int r=twk;
	//cout<<"n:"<<n<<"r"<<r;
	total+=comb(n,r);
	//cout<<",total combinations:"<<total<<endl;fflush(stdout);
	//int ccount=1;
	bool done=true;

	for(int iwk=0;iwk<r;iwk++)indx.push_back(iwk);
	while(done){
	  done=false;
	  //cout <<"\t"<< ccount++ << " of " << total << " ";fflush(stdout);
	  //H_comb.reset();
	  pos=0;
	  sig_bit=1;
	  for(int owk=0;owk<r;owk++){
		  //H_comb.set(aval_pos.at(indx[owk]));
		  //cout<<"owk:"<<owk<<",H_comb:"<<H_comb<<endl;
	    //cout <<"index:"<< aval_pos.at(indx[owk]) <<",sig_bit: "<<sig_bit;fflush(stdout);
	    pos+=comb(aval_pos.at(indx[owk]),sig_bit);
	    //cout<<",pos:"<<comb(aval_pos.at(indx[owk]),sig_bit);
	    sig_bit++;
	  }
	  //cout<<"final_pos:"<<pos<<endl;
	  //cout<<"out of for loop"<<endl;fflush(stdout);
	  //cout<<"->"<<H_comb;fflush(stdout);
	  /*sig_bits_remaining=r;
	  for(int i=(H_comb.size()-1);i>=0;i--){
	    //cout<<"\t\ti:"<<i;fflush(stdout);
	    if(H_comb.test(i)){
	      if(sig_bits_remaining>1){
		pos+=comb(i,sig_bits_remaining);
	      }
	      else{
		pos+=i;
	      }
	      sig_bits_remaining--;
	    }
	      
	    //cout<<",sig_bits_remaining:"<<sig_bits_remaining<<endl;fflush(stdout);
	    if(sig_bits_remaining==0||i==1){
	      //cout<<"exiting, no more significant bits"<<endl;fflush(stdout);
	      break;//no more bits to count
	    }
	  }*/
	  //cout<<",at pos:"<<pos<<",level:"<<twk<<",max_pos:";fflush(stdout);cout<<h_comb_to_degree.at(twk-1).size()<<endl;fflush(stdout);
	  h_comb_to_degree.at(twk-1).at(pos)+=adding;
	  //cout<<"\tAdded CC"<<it_h_values->first<<",value:"<<it_h_values->second<<" to level:"<<twk<<" to pos:"<<pos<<endl;fflush(stdout);

	
	  for(int iwk=r-1;iwk>=0;iwk--){
	    //cout<<"\t\t\tiwk:"<<iwk<<"ind[iwk]:"<<indx[iwk]<<"(n-1)-(r-iwk)"<<(n-1)-(r-iwk)<<endl;fflush(stdout);
	    if(indx[iwk]<=(n-1)-(r-iwk)){
		    indx[iwk]++;
		    for(int swk=iwk+1;swk<r;swk++){
			    indx[swk]=indx[swk-1]+1;
		    }
		    iwk=-1;
		    done=true;
		    //cout<<"\tFinished with subset"<<endl;
	    }
	  }
	}
	//cout << "\t--------------------------- " << endl;
	indx.clear();
	}

    }
//NOW we Substract the corresponding substract counters
    total_CCs+=h_counter_substract[level].size();
      if(h_counter_substract.at(level).size()==0){
	continue;
      }
    for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter_substract[level].begin(); it_h_values != h_counter_substract[level].end(); it_h_values++){
      //cout<<"\tCC counter"<<","<<it_h_values->first;cout<<",size:"<<it_h_values->second<<endl;
      //cout<<"applies to levels "<< hset_size-it_h_values->first.count()<<"to 1"<<endl;
      
      long substracting=it_h_values->second;
      //long substracting=0;//just to see the effects if we do not substract
          
//Set which bits are available      
      aval_pos.clear();
      for(unsigned i=0;i<(it_h_values->first).size();i++){
	if(!((it_h_values->first).test(i))){
	  aval_pos.push_back(i);
	  //cout<<"\tpos "<<i<<"is available"<<endl;
	}
      }

      //cout<<"aval_pos size:"<<aval_pos.size()<<endl;
      int n=aval_pos.size();
      int j=1;
      //int k=aval_pos.size();
      //int sig_bits_remaining=k;

      for(int twk=j;twk<=min(degree,unsigned(aval_pos.size()));twk++){
	//cout<<"\tWorking on level:"<<twk<<endl;
	//H_comb.reset();
	int r=twk;
	//cout<<"n:"<<n<<"r"<<r;
	total+=comb(n,r);
	//cout<<",total combinations:"<<total<<endl;fflush(stdout);
	//int ccount=1;
	bool done=true;

	for(int iwk=0;iwk<r;iwk++)indx.push_back(iwk);
	while(done){
	  done=false;
	  //cout <<"\t"<< ccount++ << " of " << total << " ";fflush(stdout);
	  //H_comb.reset();
	  pos=0;
	  sig_bit=1;
	  for(int owk=0;owk<r;owk++){
		  //H_comb.set(aval_pos.at(indx[owk]));
		  //cout<<"owk:"<<owk<<",H_comb:"<<H_comb<<endl;
	    //cout <<"index:"<< aval_pos.at(indx[owk]) <<",sig_bit: "<<sig_bit;fflush(stdout);
	    pos+=comb(aval_pos.at(indx[owk]),sig_bit);
	    //cout<<",pos:"<<comb(aval_pos.at(indx[owk]),sig_bit);
	    sig_bit++;
	  }
	  //cout<<"final_pos:"<<pos<<endl;
	  //cout<<"out of for loop"<<endl;fflush(stdout);
	  //cout<<"->"<<H_comb;fflush(stdout);
	  /*sig_bits_remaining=r;
	  for(int i=(H_comb.size()-1);i>=0;i--){
	    //cout<<"\t\ti:"<<i;fflush(stdout);
	    if(H_comb.test(i)){
	      if(sig_bits_remaining>1){
		pos+=comb(i,sig_bits_remaining);
	      }
	      else{
		pos+=i;
	      }
	      sig_bits_remaining--;
	    }
	      
	    //cout<<",sig_bits_remaining:"<<sig_bits_remaining<<endl;fflush(stdout);
	    if(sig_bits_remaining==0||i==1){
	      //cout<<"exiting, no more significant bits"<<endl;fflush(stdout);
	      break;//no more bits to count
	    }
	  }*/
	  //cout<<",at pos:"<<pos<<",level:"<<twk<<",max_pos:";fflush(stdout);cout<<h_comb_to_degree.at(twk-1).size()<<endl;fflush(stdout);
	  h_comb_to_degree.at(twk-1).at(pos)-=substracting;
	  //cout<<"\tAdded CC"<<it_h_values->first<<",value:"<<it_h_values->second<<" to level:"<<twk<<" to pos:"<<pos<<endl;fflush(stdout);

	
	  for(int iwk=r-1;iwk>=0;iwk--){
	    //cout<<"\t\t\tiwk:"<<iwk<<"ind[iwk]:"<<indx[iwk]<<"(n-1)-(r-iwk)"<<(n-1)-(r-iwk)<<endl;fflush(stdout);
	    if(indx[iwk]<=(n-1)-(r-iwk)){
		    indx[iwk]++;
		    for(int swk=iwk+1;swk<r;swk++){
			    indx[swk]=indx[swk-1]+1;
		    }
		    iwk=-1;
		    done=true;
		    //cout<<"\tFinished with subset"<<endl;
	    }
	  }
	}
	//cout << "\t--------------------------- " << endl;
	indx.clear();
	}

    }
  }
  //Now print totals
  cout.setf(ios::fixed);
  vector<pair<boost::dynamic_bitset<> ,long> > h_ranking;
  //bool skip_heuristic=false;
      
  if(Current_RIDA_Phase==SOLVING_PHASE){//so if we are calling from solving phase is because we are reevaluating our choice
    //if any heuristic has been removed, then we need to calculate h_comb_map2 again for the new set of heuristics!
    if(hset_size!=orig_hset_size){//need to recalculate h_comb_map because we have eliminated at least one heuristic
      cout<<"Resetting h_comb_map&h_comb_map2!"<<endl;
      h_comb_map.clear();
      h_comb_map2.clear();
    }
    if(h_comb_map2.size()==0){//call print_final_size to populate h_comb_map2
      print_final_size_counters(false,degree);
    }
    cout<<"finished Resetting h_comb_map and h_comb map2"<<endl;
    cout<<"h_comb_map1&2 sizes:"<<h_comb_map.size()<<","<<h_comb_map2.size()<<endl;
  }
  for (unsigned level=0;level<degree;level++){
    for(unsigned h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
      //Now revaluate to reselect best heuristic if necessary
      if(Current_RIDA_Phase==SOLVING_PHASE){//so if we are calling from solving phase is because we are reevaluating our choice
	//First get rid of capped heuristics because the were never the best heuristic for any of the Hoff roots
	/*for(int heur=0;heur<h_comb_to_degree.at(0).size();heur++){
	  if(h_capped.at(heur)&&h_comb_map2[level][h_index].test(heur)){//heuristic is active in this subset and capped
	    skip_heuristic=true;
	    break;
	  }
	}
	if(skip_heuristic){
	  skip_heuristic=false;
	  continue;
	}*/
	boost::dynamic_bitset<> translated_h_comb(orig_hset_size);
	boost::dynamic_bitset<> untranslated_h_comb(h_comb_map2[level][h_index]);
	bool at_least_one_strong_heur=false;//we skip combinations which do not have at least one strong heur
	for(int i=0;i<untranslated_h_comb.size();i++){
	  if(untranslated_h_comb.test(i)){//activate the real heuristic
	    translated_h_comb.set(real_heur_pos[i]);
	    if((!at_least_one_strong_heur)&&strong_heur.find(i)!=strong_heur.end()){
	      //cout<<"\tstrong heur at:"<<i<<",strong_heur.size:"<<strong_heur.size()<<endl;
	      at_least_one_strong_heur=true;
	    }
	  }
	}
	if(!at_least_one_strong_heur){//we do not care about this heuristic combination so we skip it
	  //cout<<"F_bound:"<<get_current_F_bound()<<",no strong_heur in the combination so skipping (translated),"<<translated_h_comb<<",with,"<<h_comb_to_degree.at(level).at(h_index)<<" nodes"<<endl;
	  continue;
	}
	if(hset_size!=orig_hset_size){//heur comb was translated
	  //cout<<"translated h_comb from:"<<untranslated_h_comb<<" to ";print_h_comb(translated_h_comb);cout<<endl;
	}
	h_ranking.push_back(pair<boost::dynamic_bitset<>,long>(translated_h_comb,h_comb_to_degree.at(level).at(h_index)));
	/*for(unsigned h_index2=h_index+1;h_index2<h_comb_to_degree.at(level).size();h_index2++){
	  cout<<"h("<<h_comb_map2[level][h_index]<<")/h("<<h_comb_map2[level][h_index2]<<")="<<double(h_comb_to_degree.at(level).at(h_index))/double(h_comb_to_degree.at(level).at(h_index2))<<endl;
	}*/
      }
      //cout<<"F_bound:"<<get_current_F_bound()<<",h_comb_to_degree["<<level<<"]["<<h_index<<"]:"<<h_comb_to_degree.at(level).at(h_index)<<endl;
    }
  }
  double best_overall_time=INT_MAX;
  double best_overall_nodes=INT_MAX;
  boost::dynamic_bitset<> h_comb_temp;
  vector<pair<boost::dynamic_bitset<>,pair<long,double> > > best_time_in_degree(degree,make_pair(h_comb_temp,make_pair(INT_MAX,INT_MAX)));
  double current_time=0;

  if(Current_RIDA_Phase==SOLVING_PHASE){//so if we are calling from solving phase is because we are reevaluating our choice
    //calculate_time_costs();//need to know the TimePerNode(heuristic_size) to determine better heuristic
    //assumes TPN to be a function of number of heuristics in subset
    sort (h_ranking.begin(), h_ranking.end(),compare_pairs_dynBitset_long);
    vector<bool> degree_visited(degree+1,false);
    cout<<"size of degree_visited:"<<degree_visited.size();
    bool skip_heur=false;
    double lmcut_current_time=0;
    double ipdb_current_time=0;
    int lmcut_current_nodes=0;
    int ipdb_current_nodes=0;
    
    for(unsigned i=0;i<h_ranking.size();i++){
      //if(degree_visited.at(h_ranking.at(i).first.count())==false){
	//double current_time=double(h_ranking.at(i).second)*double(time_cost[h_ranking.at(i).first.count()-1]);
	if(h_ranking.at(i).first.count()>1){//skip adding blind or stopped heurs if degree>1
	  for (size_t j = 0; j < h_ranking.at(i).first.size(); j++){
	    if(h_ranking.at(i).first.test(j)){
	      string heur_name=orig_heuristics.at(j)->get_heur_name();
	      if((!orig_heuristics[j]->is_using())||(heur_name.find("blind")!=string::npos)){
		  skip_heur=true;
		  //cout<<"skipping "<<heur_name<<endl;
		  break;
	      }
	    }
	  }
	}
	if(skip_heur){
	  current_time=INT_MAX/2;
	  skip_heur=false;
	}
	else{
	  current_time=double(h_ranking.at(i).second)*calculate_time_costs_specific(h_ranking.at(i).first,orig_heuristics);
	  if(h_ranking.at(i).first.count()==1){
	    for (size_t j = 0; j < h_ranking.at(i).first.size(); j++){
	      if(h_ranking.at(i).first.test(j)){
		if(orig_heuristics.at(j)->get_heur_name().find(",regular_lm_cut")!=string::npos){
		  lmcut_current_time=current_time;
		  lmcut_current_nodes=h_ranking.at(i).second;
		}
		else if(orig_heuristics.at(j)->get_heur_name().find("heur is IPDB")!=string::npos){
		  ipdb_current_time=current_time;
		  ipdb_current_nodes=h_ranking.at(i).second;
		}
	      }
	    }
	  }
	}
	if(best_time_in_degree.at(h_ranking.at(i).first.count()-1).second.second>current_time){
	  best_time_in_degree.at(h_ranking.at(i).first.count()-1).first=h_ranking.at(i).first;
	  best_time_in_degree.at(h_ranking.at(i).first.count()-1).second.first=h_ranking.at(i).second;
	  best_time_in_degree.at(h_ranking.at(i).first.count()-1).second.second=current_time;
	}
	if(current_time<best_overall_time){
	  best_h_comb=h_ranking.at(i).first;
	  best_overall_time=current_time;
	  best_overall_nodes=h_ranking.at(i).second;
	}
      //}
      /*else{
      cout<<"F_bound:,"<<get_current_F_bound()<<",degree,"<<h_ranking.at(i).first.count()<<",ranking,h(,";print_h_comb(h_ranking.at(i).first);cout<<",) is in pos,"<<i<<",nodes:,"<<h_ranking.at(i).second<<endl;
      }*/
    }
    //Now predicting the actual amount of nodes for the next F-boundary by the selected heursitic
    //This is done for debugging purposes at the moment, but will be needed if we want to use HBF to predict future performance up to solution cost
    Pred_Asymptotic_HBF=double(best_overall_nodes)/double(last_sampled_HoF_Root);
    Pred_Extra_Nodes=double(F_boundary_size)*Pred_Asymptotic_HBF;
    cout<<"F-bound:,"<<get_current_F_bound()<<",Pred_Asymptotic_HBF:,"<<Pred_Asymptotic_HBF<<",nodes:,"<<best_overall_nodes<<",sampled roots:,"<<last_sampled_HoF_Root<<",next_iter_additional_nodes:"<<double(F_boundary_size)*Pred_Asymptotic_HBF<<endl;
    for(unsigned i=0;i<best_time_in_degree.size();i++){
      if(i==0){

	cout<<"lmcut_current_time:,"<<lmcut_current_time<<",lmcut_current_nodes:,"<<lmcut_current_nodes<<",time_ratio:,"<<best_time_in_degree.at(0).second.second/lmcut_current_time<<",node_ratio:,"<<double(best_time_in_degree.at(h_ranking.at(i).first.count()-1).second.first)/lmcut_current_nodes<<endl;
	cout<<"ipdb_current_time:,"<<ipdb_current_time<<",ipdb_current_nodes:,"<<ipdb_current_nodes<<",time_ratio:"<<best_time_in_degree.at(0).second.second/ipdb_current_time<<",node_ratio:"<<double(best_time_in_degree.at(h_ranking.at(i).first.count()-1).second.first)/ipdb_current_nodes<<endl;
      }
	cout<<"Best_heuristic_in_degree;F_bound:,"<<get_current_F_bound()<<",degree,"<<best_time_in_degree.at(i).first.count()<<",h(,";print_h_comb(best_time_in_degree.at(i).first);cout<<",) nodes:,"<<best_time_in_degree.at(i).second.first<<",time:"<<best_time_in_degree.at(i).second.second<<",time_cost:"<<calculate_time_costs_specific(best_time_in_degree.at(i).first,orig_heuristics)<<endl;
	cout<<"Best_heuristics_in_degree names"<<endl;
	for (int j=0;j<orig_heuristics.size();j++){
	  if(best_time_in_degree.at(i).first.test(j)){
	      cout<<"\tHeur[:"<<j<<"]:";cout<<orig_heuristics.at(j)->get_heur_name()<<endl;
	  }
	}
	/*  if(i>0){
	  if((best_time_in_degree.at(i).second.second*0.80)<=best_overall_time){
	    cout<<"additional heuristic is quite cheap, and it either decreases the overall time or increases it less than 20%, we will add it just in case, it could be beneficial for memory or even time reasons, we do not do a complete sample afterall"<<endl;
	    best_h_comb=best_time_in_degree.at(i).first;
	  }
	}*/
    }
	//cout<<"Best_heuristic_in_degree;F_bound:,"<<get_current_F_bound()<<",degree,"<<best_time_in_degree.at(i).first.count()<<",h(,";print_h_comb(best_time_in_degree.at(i).first);cout<<",) is in pos,"<<i<<",nodes:,"<<h_ranking.at(i).second<<",time:"<<best_time_in_degree<<",time_cost:"<<calculate_time_costs_specific(h_ranking.at(i).first,orig_heuristics)<<endl;
    //best_h_comb=h_ranking.at(0).first;

    //For Experiment only purposed
    /*boost::dynamic_bitset<> not_disj_comb;
    for (size_t i=0;i<heuristics.size();i++){
      if((heur_name.find("without disjoint")!=string::npos)){
	not_disj_comb.set(i);
      }
    }
    long not_disj_comb_nodes=calculate_specific_heuristic(not_disj_comb);
    double not_disj_comb_time=double(h_ranking.at(i).second)*calculate_time_costs_specific(h_ranking.at(i).first,orig_heuristics);
    cout<<"not_disj_comb:"<<not_disj_comb<<",nodes:"<<not_disj_comb_nodes<<",time:"<<not_disj_comb_time<<endl;
    */

    cout<<"Setting best heuristic subset for F_bound,"<<get_current_F_bound()<<",is h_comb:,";print_h_comb(best_h_comb);cout<<endl;
  }
  //annotate_sampling_preds(orig_heuristics);
  cout<<"NEED TO FIX annotate_sampling_preds, NEED TO ADD TO FUNCTION SELECTED HEURS TRANSLATED POSITIONS, I THINK!, for now stopping calling it"<<endl;
  cout<<"Total Additions:"<< scientific <<total<<",Total CCs:"<<total_CCs<<endl;
  return;
}
void HST::update_hset_size(unsigned set_size){
  cout<<"Updating hset_size from"<<hset_size<<"to hset_size:"<<set_size<<endl;
  hset_size=set_size;
}
void HST::update_original_hset_size(unsigned orig_set_size){
  cout<<"Updating original hset_size from to orig_hset_size:"<<orig_set_size<<endl;
  orig_hset_size=orig_set_size;
}
unsigned HST::get_hset_size(){
  return hset_size;
}
void HST::select_best_heuristics(unsigned degree,const vector<Heuristic *> orig_heuristics){
  vector<unsigned> best_next_iteration;
  vector<unsigned> best_MAX;//want to look at whether Korf,Ariel or comb is best
  boost::dynamic_bitset<> h_comb;
  vector<double> Korf_size;
  vector<double> Korf_time;
  vector<double> Ariel_size;
  vector<double> Ariel_time;
  vector<double> MAX_size;
  vector<double> MAX_time;
  char filename[MAX_LENGTH];
  //map<boost::dynamic_bitset<>,double,bitset_comp2>::iterator culling_size_index;
  //static bool first_call=true;
  sprintf(filename,"output/h_comparison_local_HBF.csv");
  ofstream myfile ( filename, ios::app );
  map<boost::dynamic_bitset<>,bool,bitset_comp2>Best_h_combs;

  cout<<"Now predicting future performance with for "<<hset_size<<" databases, degree:"<<degree<<endl;
  cout<<"Using local HBF"<<endl;
  cout<<"Current iteration:"<<iter_index<<",F_bound:"<<current_F_bound<<endl;
  int latest_iteration=iter_index;
   
  boost::dynamic_bitset<> MAX_9_COMB(hset_size); //(string("111111111"));
  boost::dynamic_bitset<> KORF_H_COMB(hset_size); //(string("000000001"));
  boost::dynamic_bitset<> ARIEL_H_COMB(hset_size);// (string("111111110"));
  MAX_9_COMB.set();
  KORF_H_COMB.reset();KORF_H_COMB.set(0);
  ARIEL_H_COMB.set();ARIEL_H_COMB.flip(0);

  //unsigned best_heuristic=0;
  //unsigned last_best_heuristic=0;
  vector<long double> best_time;
  best_time.resize(10,0);

  long double current_time=0;
  long double current_nodes=0;
  /*long double current_time_RAND=0;
  long double current_time_MAX=0;
  long double best_time_RAND=INT_MAX/2;
  long double best_time_MAX=INT_MAX/2;
  boost::dynamic_bitset<> best_h_comb_RAND;
  boost::dynamic_bitset<> best_h_comb_MAX;
  bool current_Rand_is_faster=false;*/
  calculate_time_costs();

/*  for(culling_size_index=Culling_size.begin();culling_size_index!=Culling_size.end();culling_size_index++){
    cout<<"\tCulling_size["<<culling_size_index->first<<"]:"<<culling_size_index->second<<endl;
    cout<<"\tCulling_iteration["<<culling_size_index->first<<"]:"<<culling_iteration[culling_size_index->first]<<endl;
  }*/
  cout<<"latest HUST iteration:"<<latest_iteration<<endl;

  best_time.at(0)=INT_MAX/2;
  best_h_comb=h_comb_map2[0][0];
    for(unsigned level=0;level<degree;level++){
//      for(unsigned long  h_index=0;h_index<h_comb_to_degree_wRand.at(level).size();h_index++){}
     for(unsigned long h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
	//cout<<"\t examining Culling_size["<<level<<"]["<<h_index<<"]";cout<<Culling_size[h_comb_map2[level][h_index]];
	//current_time_RAND=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],1+latest_iteration-culling_iteration[h_comb_map2[level][h_index]])*time_cost[0];
	//current_time_MAX=Culling_size_MAX[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],1+latest_iteration-culling_iteration[h_comb_map2[level][h_index]])*time_cost[level];
	current_nodes=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],1+latest_iteration-culling_iteration[h_comb_map2[level][h_index]]);
	//current_time=current_nodes*time_cost[level];
	current_time=current_nodes*calculate_time_costs_specific(h_comb_map2[level][h_index],orig_heuristics);
	/*if(current_time_RAND<current_time_MAX){
	  current_Rand_is_faster=true;
	  if(current_time_RAND<best_time_RAND){
	    best_h_comb_RAND=h_comb_map2[level][h_index];
	    best_time_RAND=current_time_RAND;
	  }
	}
	else{
	  current_Rand_is_faster=false;
	  if(current_time_MAX<best_time_MAX){
	    best_h_comb_MAX=h_comb_map2[level][h_index];
	    best_time_MAX=current_time_MAX;
	  }
	}*/
	//current_time=min(current_time_RAND,current_time_MAX);
	//current_time=current_time_RAND;
	//current_time=min(current_time_MAX,current_time_RAND);
	//cout<<",culling_iteration["<<h_comb_map2[level][h_index]<<"]:"<<culling_iteration[h_comb_map2[level][h_index]]<<"current_nodes:"<<current_nodes<<",HBF:"<<HBF[h_comb_map2[level][h_index]]<<",current_time:"<<current_time<<endl;
	if(current_time<best_time.at(0)){
	  /*if(current_Rand_is_faster){
	    //cout<<"Rand is faster,"<<"current_time_RAND"<<current_time_RAND<<",current_time_MAX"<<current_time_MAX<<endl;
	    Rand_is_faster=true;
	  }
	  else{
	    Rand_is_faster=false;
	  }*/
	  best_time.at(0)=current_time;
	  best_h_comb=h_comb_map2[level][h_index];
	cout<<"\t\tpartial best_comb is:";print(best_h_comb,false);
	cout<<",new best time:"<<best_time.at(0)<<",Culling_size:"<<Culling_size[h_comb_map2[level][h_index]]<<",Cull iter:"<<culling_iteration[h_comb_map2[level][h_index]]<<",L_HBF:"<<HBF[h_comb_map2[level][h_index]]<<",current_nodes"<<current_nodes<<",level;:"<<level<<endl;
	
	}
	//best_time=min(best_time,h_comb_to_degree.at(level).at(h_index)
      }
    }

    //myfile<<","<<best_h_comb<<scientific<<","<<best_time<<endl;
    cout<<"\tnew best_comb is:";
  for(int i=0;i<best_h_comb.size();i++){
    if(best_h_comb.test(i)){
      cout<<i<<",";
    }
  }
    cout<<",best time:"<<best_time.at(0)<<endl;
    /*if(gcmd_line.run_crossover){
      next_heuristic.push_back(best_h_comb);
    }
	    
    if(Rand_is_faster){
      cout<<"N:"<<problem_index<<",Rand is faster,"<<"best_time_RAND"<<best_time_RAND<<",time_for_MAX:"<<Culling_size_MAX[best_h_comb]*pow(HBF[best_h_comb],1+latest_iteration-culling_iteration[best_h_comb])*time_cost[best_h_comb.count()-1]<<",best_time_MAX"<<best_time_MAX<<endl;
      cout<<"Verifying RAND is faster by runing MAX"<<endl;
      best_h_comb=best_h_comb_MAX;
      cout<<"N:"<<problem_index<<",new best_h_comb:"<<best_h_comb<<endl;
    }
    else{
      cout<<"N:"<<problem_index<<",Max is faster,"<<"best_time_RAND"<<best_time_RAND<<",time_for_MAX:"<<Culling_size_MAX[best_h_comb]*pow(HBF[best_h_comb],1+latest_iteration-culling_iteration[best_h_comb])*time_cost[best_h_comb.count()-1]<<",best_time_MAX"<<best_time_MAX<<endl;
      cout<<"But only running Rand, so changing to best_h_comb:"<<best_h_comb_RAND;
      best_h_comb=best_h_comb_RAND;
      cout<<"Exiting problem because right now only running thos instances for which RAND is faster"<<endl;
      throw(20);
    }*/

    cout<<"\tbest_comb for iter:0 is:";print(best_h_comb,true);

    cout<<"selecting best_comb based on the first iteration"<<endl;
    return;
    //best_time.push_back(pair<bitset<hset_size>,double>);

  for(unsigned iteration_index=1;iteration_index<10;iteration_index++){
    //best_heuristic=0;
    Korf_size.push_back(Korf_size.back()*HBF[KORF_H_COMB]);
    cout<<"iter_index:"<<iteration_index<<",Korf Next Size:"<<Korf_size.back();
    Korf_time.push_back(Korf_size.back()*time_cost.at(0));
    best_time.at(iteration_index)=Korf_time.back();
    cout<<",time:"<<scientific<<Korf_time.back();
    Ariel_size.push_back(Ariel_size.back()*HBF[ARIEL_H_COMB]);
    cout<<",8h Next Size:"<<Ariel_size.back();
    Ariel_time.push_back(Ariel_size.back()*time_cost.at(hset_size-2));
    if((Ariel_time.back()>0)&&(best_time.at(iteration_index)>Ariel_time.back())){
	//best_heuristic=1;
	best_time.at(iteration_index)=Ariel_time.back();
    }
    cout<<",time:"<<Ariel_time.back();
    MAX_size.push_back(MAX_size.back()*HBF[MAX_9_COMB]);
    cout<<",Next MAX Size:"<<MAX_size.back();
    MAX_time.push_back(MAX_size.back()*time_cost.at(hset_size-1));
    cout<<",MAX_time:"<<MAX_time.back()<<endl;
    if(best_time.at(iteration_index)>MAX_time.back()){
	//best_heuristic=2;
	best_time.at(iteration_index)=MAX_time.back();
    }
    //myfile<<min_f+iter_index*2<<","<<Korf_size.back()<<","<<Korf_time.back()<<","<<Ariel_size.back()<<","<<Ariel_time.back()<<","<<MAX_size.back()<<","<<MAX_time.back();
    
    /*if(best_heuristic==0){
      //myfile<<",1h_6_6_6_6";
    }
    else if(best_heuristic==1){
      //myfile<<",8h_5_5_5_5";
    }
    else{
      //myfile<<",Max_of_9h";
    }*/
    /*if(last_best_heurirstic!=best_heuristic){
      cout<<"Crossover at N:"<<problem_index<<",iteration:"<<
    last_best_heuristic=best_heuristic;
    }
    */

    unsigned long best_h_comb_index=0;
    for(unsigned level=0;level<degree;level++){
      //for(unsigned long  h_index=0;h_index<h_comb_to_degree_wRand.at(level).size();h_index++){}
      for(unsigned long  h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
	//cout<<"\t examining Culling_size["<<level<<"]["<<h_index<<"]"<<Culling_size[h_comb_map2[level][h_index]];
	//current_time=Culling_size[h_comb_map2[level][h_index]]*pow(HBF_AVG,1+iter_index+latest_iteration-culling_iteration[h_comb_map2[level][h_index]])*time_cost[level];
	current_nodes=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],latest_iteration-culling_iteration[h_comb_map2[level][h_index]]+iter_index+1);
	//current_time=current_nodes*time_cost[level];
	current_time=current_nodes*calculate_time_costs_specific(h_comb_map2[level][h_index],orig_heuristics);
	//cout<<",culling_iteration["<<h_comb_map2[level][h_index]<<"]:"<<culling_iteration[h_comb_map2[level][h_index]]<<",HBF:"<<HBF_AVG<<",current_time:"<<current_time<<endl;
	if(current_time<best_time.at(iter_index)){
	  best_h_comb_index=h_index;
	  best_time.at(iter_index)=current_time;
	  best_h_comb=h_comb_map2[level][h_index];
	  cout<<"\t\tpartial best_comb is:";
	  for(int i=0;i<best_h_comb.size();i++){
	    if(best_h_comb.test(i)){
	      cout<<i<<",";
	    }
	  }
	  cout<<",new best time:"<<best_time.at(iter_index)<<",Culling_size:"<<Culling_size[h_comb_map2[level][h_index]]<<",Cull iter:"<<culling_iteration[h_comb_map2[level][h_index]]<<",L_HBF:"<<HBF[h_comb_map2[level][h_index]]<<",current_nodes:"<<current_nodes<<",level;:"<<level<<endl;
	}
      }
    }
	  
    cout<<"\tbest_comb for iter:"<<iter_index<<" is:";
  for(int i=0;i<best_h_comb.size();i++){
    if(best_h_comb.test(i)){
      cout<<i<<",";
    }
  }
  /*if(gcmd_line.run_crossover){
    next_heuristic.push_back(best_h_comb);
    if(next_heuristic.back()!=next_heuristic.at(0)&&best_time.at(iter_index)<10000000){//just a hack to not rerun problems with no crossovers
      crossover_prediction=true;
    }
  }*/
  if(best_h_comb_list.empty()||(best_h_comb_list.back()!=best_h_comb)){
    best_h_comb_list.push_back(best_h_comb);
    Best_h_combs[h_comb_map2[best_h_comb.count()-1][best_h_comb_index]]=true;
  }
    cout<<",best time:"<<best_time.at(iter_index)<<endl;
    return;
    //myfile<<","<<best_h_comb<<scientific<<","<<best_time<<endl;
    //best_time.push_back(pair<bitset<hset_size>,double>);
  }
  //Now calculate which heuristics will go to third phase, determined by Accuracy_Threshold
  /*for(unsigned iteration_index=0;iteration_index<10;iteration_index++){
     //double iter_best_time_threshold=best_time.at(iteration_index)+best_time.at(iteration_index)*double(ACCURACY_THRESHOLD)/100.0;
     //cout<<"iter_best_time["<<iteration_index<<"]:"<<best_time.at(iteration_index)<<",All heuristics which belong to a subset whose time is less than:"<<iter_best_time_threshold<<" will be added to third phase sampling"<<endl;
    for(unsigned level=0;level<degree;level++){
      for(unsigned long  h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
	current_time=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],latest_iteration-culling_iteration[h_comb_map2[level][h_index]]+iteration_index+1)*time_cost[level];
	//cout<<",culling_iteration["<<h_comb_map2[level][h_index]<<"]:"<<culling_iteration[h_comb_map2[level][h_index]]<<",HBF:"<<HBF_AVG<<",current_time:"<<current_time<<endl;
	*//*if((current_time<iter_best_time_threshold)&&(Best_h_combs.find(h_comb_map2[level][h_index])==Best_h_combs.end())){
	  //cout<<"subset ";print(h_comb_map2[level][h_index],false);cout<<" qualifies to third phase"<<endl;
	  best_h_comb_list.push_back(h_comb_map2[level][h_index]);
	  Best_h_combs[h_comb_map2[level][h_index]]=true;
	  }*//*
      }
    }
  }*/
  /*myfile<<"N\tF_bound\tIter_indx\t";
  string temp_h_comb;

  for(unsigned comb_index=0;comb_index<best_h_comb_list.size();comb_index++){
    temp_h_comb="";
    for(int i=0;i<best_h_comb_list.at(comb_index).size();i++){
      if(best_h_comb_list.at(comb_index).test(i)){
	//myfile<<i;
	temp_h_comb+= boost::lexical_cast<std::string>(i);
	if(i<(best_h_comb.size()-1)){
	  //myfile<<",";
	  temp_h_comb+=",";
	} 
      }
    }
    myfile<<temp_h_comb<<"_nodes\t";
    myfile<<temp_h_comb<<"_time\t";
  }

  myfile<<endl;*/
#ifdef CROSSOVER_PREDICTION
  //print any previous crossovers
  for(unsigned prev_iter_index=0;prev_iter_index<best_prev_iter_heuristic.size();prev_iter_index++){
    if(prev_iter_index>0){
      if(best_prev_iter_heuristic.at(prev_iter_index)==best_prev_iter_heuristic.at(prev_iter_index-1)){
	continue;
      }
    }

    string temp_h_comb;
      temp_h_comb="";
    for(int i=0;i<best_prev_iter_heuristic.at(prev_iter_index).size();i++){
      if(best_prev_iter_heuristic.at(prev_iter_index).test(i)){
	//myfile<<i;
	temp_h_comb+= boost::lexical_cast<std::string>(i);
	if(i<(best_prev_iter_heuristic.at(prev_iter_index).size()-1)){
	  //myfile<<",";
	  temp_h_comb+=",";
	} 
      }
    }
    myfile<<"best h_comb at prev_iter("<<prev_iter_index<<"):"<<temp_h_comb<<endl;
  }
#endif 
/*
  for(unsigned iteration_index=0;iteration_index<10;iteration_index++){
    myfile<<problem_index<<"\t"<<current_F_bound+iteration_index*ITER_STEP<<"\t"<<iteration_index;
    for(unsigned comb_index=0;comb_index<best_h_comb_list.size();comb_index++){
      cout<<"h_comb:";
      unsigned level=0;
    for(int i=0;i<best_h_comb_list.at(comb_index).size();i++){
      if(best_h_comb_list.at(comb_index).test(i)){
	cout<<i;
	cout<<",";
	level++;
      }
    }
    cout<<"level:"<<level<<endl;
	myfile<<"\t"<<Culling_size[best_h_comb_list.at(comb_index)]*pow(HBF[best_h_comb_list.at(comb_index)],latest_iteration-culling_iteration[best_h_comb_list.at(comb_index)]+iteration_index+1)<<"\t"<<Culling_size[best_h_comb_list.at(comb_index)]*pow(HBF[best_h_comb_list.at(comb_index)],latest_iteration-culling_iteration[best_h_comb_list.at(comb_index)]+iteration_index+1)*time_cost[level-1];
	if(comb_index<(best_h_comb_list.size()-1)) myfile<<"\t";
    }
    myfile<<endl;
  }
    
  myfile.close();

  //cout<<"before throw"<<endl;fflush(stdout);
  //cerr<<"exit for debugging crossovers"<<endl;throw 20;
  //cout<<"after throw"<<endl;fflush(stdout);
*/
}
void HST::select_best_heuristics(unsigned degree){
  vector<unsigned> best_next_iteration;
  vector<unsigned> best_MAX;//want to look at whether Korf,Ariel or comb is best
  boost::dynamic_bitset<> h_comb;
  vector<double> Korf_size;
  vector<double> Korf_time;
  vector<double> Ariel_size;
  vector<double> Ariel_time;
  vector<double> MAX_size;
  vector<double> MAX_time;
  char filename[MAX_LENGTH];
  //map<boost::dynamic_bitset<>,double,bitset_comp2>::iterator culling_size_index;
  //static bool first_call=true;
  sprintf(filename,"output/h_comparison_local_HBF.csv");
  ofstream myfile ( filename, ios::app );
  map<boost::dynamic_bitset<>,bool,bitset_comp2>Best_h_combs;

  cout<<"Now predicting future performance with for "<<hset_size<<" databases, degree:"<<degree<<endl;
  cout<<"Using local HBF"<<endl;
  cout<<"Current iteration:"<<iter_index<<",F_bound:"<<current_F_bound<<endl;
  int latest_iteration=iter_index;
   
  boost::dynamic_bitset<> MAX_9_COMB(hset_size); //(string("111111111"));
  boost::dynamic_bitset<> KORF_H_COMB(hset_size); //(string("000000001"));
  boost::dynamic_bitset<> ARIEL_H_COMB(hset_size);// (string("111111110"));
  MAX_9_COMB.set();
  KORF_H_COMB.reset();KORF_H_COMB.set(0);
  ARIEL_H_COMB.set();ARIEL_H_COMB.flip(0);

  //unsigned best_heuristic=0;
  //unsigned last_best_heuristic=0;
  vector<long double> best_time;
  best_time.resize(10,0);

  long double current_time=0;
  long double current_nodes=0;
  /*long double current_time_RAND=0;
  long double current_time_MAX=0;
  long double best_time_RAND=INT_MAX/2;
  long double best_time_MAX=INT_MAX/2;
  boost::dynamic_bitset<> best_h_comb_RAND;
  boost::dynamic_bitset<> best_h_comb_MAX;
  bool current_Rand_is_faster=false;*/
  //calculate_time_costs();

/*  for(culling_size_index=Culling_size.begin();culling_size_index!=Culling_size.end();culling_size_index++){
    cout<<"\tCulling_size["<<culling_size_index->first<<"]:"<<culling_size_index->second<<endl;
    cout<<"\tCulling_iteration["<<culling_size_index->first<<"]:"<<culling_iteration[culling_size_index->first]<<endl;
  }*/
  cout<<"latest HUST iteration:"<<latest_iteration<<endl;

  best_time.at(0)=INT_MAX/2;
  best_h_comb=h_comb_map2[0][0];
    for(unsigned level=0;level<degree;level++){
//      for(unsigned long  h_index=0;h_index<h_comb_to_degree_wRand.at(level).size();h_index++){}
     for(unsigned long h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
	//cout<<"\t examining Culling_size["<<level<<"]["<<h_index<<"]";cout<<Culling_size[h_comb_map2[level][h_index]];
	//current_time_RAND=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],1+latest_iteration-culling_iteration[h_comb_map2[level][h_index]])*time_cost[0];
	//current_time_MAX=Culling_size_MAX[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],1+latest_iteration-culling_iteration[h_comb_map2[level][h_index]])*time_cost[level];
	current_nodes=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],1+latest_iteration-culling_iteration[h_comb_map2[level][h_index]]);
	current_time=current_nodes*time_cost[level];
	/*if(current_time_RAND<current_time_MAX){
	  current_Rand_is_faster=true;
	  if(current_time_RAND<best_time_RAND){
	    best_h_comb_RAND=h_comb_map2[level][h_index];
	    best_time_RAND=current_time_RAND;
	  }
	}
	else{
	  current_Rand_is_faster=false;
	  if(current_time_MAX<best_time_MAX){
	    best_h_comb_MAX=h_comb_map2[level][h_index];
	    best_time_MAX=current_time_MAX;
	  }
	}*/
	//current_time=min(current_time_RAND,current_time_MAX);
	//current_time=current_time_RAND;
	//current_time=min(current_time_MAX,current_time_RAND);
	//cout<<",culling_iteration["<<h_comb_map2[level][h_index]<<"]:"<<culling_iteration[h_comb_map2[level][h_index]]<<",HBF:"<<HBF[h_comb_map2[level][h_index]]<<",current_time:"<<current_time<<endl;
	if(current_time<best_time.at(0)){
	  /*if(current_Rand_is_faster){
	    //cout<<"Rand is faster,"<<"current_time_RAND"<<current_time_RAND<<",current_time_MAX"<<current_time_MAX<<endl;
	    Rand_is_faster=true;
	  }
	  else{
	    Rand_is_faster=false;
	  }*/
	  best_time.at(0)=current_time;
	  best_h_comb=h_comb_map2[level][h_index];
	cout<<"\t\tpartial best_comb is:";print(best_h_comb,false);
	cout<<",new best time:"<<best_time.at(0)<<",Culling_size:"<<Culling_size[h_comb_map2[level][h_index]]<<",Cull iter:"<<culling_iteration[h_comb_map2[level][h_index]]<<",L_HBF:"<<HBF[h_comb_map2[level][h_index]]<<",current_nodes"<<current_nodes<<",level;:"<<level<<endl;
	leaf_selection_ratio=double(leaves_to_sample)/current_nodes;
	cout<<"Initial leaf_section_ratio based on leaves_to_sample="<<leaf_selection_ratio;
	
	}
	//best_time=min(best_time,h_comb_to_degree.at(level).at(h_index)
      }
    }

    //myfile<<","<<best_h_comb<<scientific<<","<<best_time<<endl;
    cout<<"\tnew best_comb is:";
  for(int i=0;i<best_h_comb.size();i++){
    if(best_h_comb.test(i)){
      cout<<i<<",";
    }
  }
    cout<<",best time:"<<best_time.at(0)<<endl;
    /*if(gcmd_line.run_crossover){
      next_heuristic.push_back(best_h_comb);
    }
	    
    if(Rand_is_faster){
      cout<<"N:"<<problem_index<<",Rand is faster,"<<"best_time_RAND"<<best_time_RAND<<",time_for_MAX:"<<Culling_size_MAX[best_h_comb]*pow(HBF[best_h_comb],1+latest_iteration-culling_iteration[best_h_comb])*time_cost[best_h_comb.count()-1]<<",best_time_MAX"<<best_time_MAX<<endl;
      cout<<"Verifying RAND is faster by runing MAX"<<endl;
      best_h_comb=best_h_comb_MAX;
      cout<<"N:"<<problem_index<<",new best_h_comb:"<<best_h_comb<<endl;
    }
    else{
      cout<<"N:"<<problem_index<<",Max is faster,"<<"best_time_RAND"<<best_time_RAND<<",time_for_MAX:"<<Culling_size_MAX[best_h_comb]*pow(HBF[best_h_comb],1+latest_iteration-culling_iteration[best_h_comb])*time_cost[best_h_comb.count()-1]<<",best_time_MAX"<<best_time_MAX<<endl;
      cout<<"But only running Rand, so changing to best_h_comb:"<<best_h_comb_RAND;
      best_h_comb=best_h_comb_RAND;
      cout<<"Exiting problem because right now only running thos instances for which RAND is faster"<<endl;
      throw(20);
    }*/

    cout<<"\tbest_comb for iter:0 is:";print(best_h_comb,true);

    cout<<"selecting best_comb based on the first iteration"<<endl;
    return;
    //best_time.push_back(pair<bitset<hset_size>,double>);

  for(unsigned iteration_index=1;iteration_index<10;iteration_index++){
    //best_heuristic=0;
    Korf_size.push_back(Korf_size.back()*HBF[KORF_H_COMB]);
    cout<<"iter_index:"<<iteration_index<<",Korf Next Size:"<<Korf_size.back();
    Korf_time.push_back(Korf_size.back()*time_cost.at(0));
    best_time.at(iteration_index)=Korf_time.back();
    cout<<",time:"<<scientific<<Korf_time.back();
    Ariel_size.push_back(Ariel_size.back()*HBF[ARIEL_H_COMB]);
    cout<<",8h Next Size:"<<Ariel_size.back();
    Ariel_time.push_back(Ariel_size.back()*time_cost.at(hset_size-2));
    if((Ariel_time.back()>0)&&(best_time.at(iteration_index)>Ariel_time.back())){
	//best_heuristic=1;
	best_time.at(iteration_index)=Ariel_time.back();
    }
    cout<<",time:"<<Ariel_time.back();
    MAX_size.push_back(MAX_size.back()*HBF[MAX_9_COMB]);
    cout<<",Next MAX Size:"<<MAX_size.back();
    MAX_time.push_back(MAX_size.back()*time_cost.at(hset_size-1));
    cout<<",MAX_time:"<<MAX_time.back()<<endl;
    if(best_time.at(iteration_index)>MAX_time.back()){
	//best_heuristic=2;
	best_time.at(iteration_index)=MAX_time.back();
    }
    //myfile<<min_f+iter_index*2<<","<<Korf_size.back()<<","<<Korf_time.back()<<","<<Ariel_size.back()<<","<<Ariel_time.back()<<","<<MAX_size.back()<<","<<MAX_time.back();
    
    /*if(best_heuristic==0){
      //myfile<<",1h_6_6_6_6";
    }
    else if(best_heuristic==1){
      //myfile<<",8h_5_5_5_5";
    }
    else{
      //myfile<<",Max_of_9h";
    }*/
    /*if(last_best_heurirstic!=best_heuristic){
      cout<<"Crossover at N:"<<problem_index<<",iteration:"<<
    last_best_heuristic=best_heuristic;
    }
    */

    unsigned long best_h_comb_index=0;
    for(unsigned level=0;level<degree;level++){
      //for(unsigned long  h_index=0;h_index<h_comb_to_degree_wRand.at(level).size();h_index++){}
      for(unsigned long  h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
	//cout<<"\t examining Culling_size["<<level<<"]["<<h_index<<"]"<<Culling_size[h_comb_map2[level][h_index]];
	//current_time=Culling_size[h_comb_map2[level][h_index]]*pow(HBF_AVG,1+iter_index+latest_iteration-culling_iteration[h_comb_map2[level][h_index]])*time_cost[level];
	current_nodes=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],latest_iteration-culling_iteration[h_comb_map2[level][h_index]]+iter_index+1);
	current_time=current_nodes*time_cost[level];
	//cout<<",culling_iteration["<<h_comb_map2[level][h_index]<<"]:"<<culling_iteration[h_comb_map2[level][h_index]]<<",HBF:"<<HBF_AVG<<",current_time:"<<current_time<<endl;
	if(current_time<best_time.at(iter_index)){
	  best_h_comb_index=h_index;
	  best_time.at(iter_index)=current_time;
	  best_h_comb=h_comb_map2[level][h_index];
	  cout<<"\t\tpartial best_comb is:";
	  for(int i=0;i<best_h_comb.size();i++){
	    if(best_h_comb.test(i)){
	      cout<<i<<",";
	    }
	  }
	  cout<<",new best time:"<<best_time.at(iter_index)<<",Culling_size:"<<Culling_size[h_comb_map2[level][h_index]]<<",Cull iter:"<<culling_iteration[h_comb_map2[level][h_index]]<<",L_HBF:"<<HBF[h_comb_map2[level][h_index]]<<",current_nodes:"<<current_nodes<<",level;:"<<level<<endl;
	}
      }
    }
	  
    cout<<"\tbest_comb for iter:"<<iter_index<<" is:";
  for(int i=0;i<best_h_comb.size();i++){
    if(best_h_comb.test(i)){
      cout<<i<<",";
    }
  }
  /*if(gcmd_line.run_crossover){
    next_heuristic.push_back(best_h_comb);
    if(next_heuristic.back()!=next_heuristic.at(0)&&best_time.at(iter_index)<10000000){//just a hack to not rerun problems with no crossovers
      crossover_prediction=true;
    }
  }*/
  if(best_h_comb_list.empty()||(best_h_comb_list.back()!=best_h_comb)){
    best_h_comb_list.push_back(best_h_comb);
    Best_h_combs[h_comb_map2[best_h_comb.count()-1][best_h_comb_index]]=true;
  }
    cout<<",best time:"<<best_time.at(iter_index)<<endl;
    return;
    //myfile<<","<<best_h_comb<<scientific<<","<<best_time<<endl;
    //best_time.push_back(pair<bitset<hset_size>,double>);
  }
  //Now calculate which heuristics will go to third phase, determined by Accuracy_Threshold
  for(unsigned iteration_index=0;iteration_index<10;iteration_index++){
     //double iter_best_time_threshold=best_time.at(iteration_index)+best_time.at(iteration_index)*double(ACCURACY_THRESHOLD)/100.0;
     //cout<<"iter_best_time["<<iteration_index<<"]:"<<best_time.at(iteration_index)<<",All heuristics which belong to a subset whose time is less than:"<<iter_best_time_threshold<<" will be added to third phase sampling"<<endl;
    for(unsigned level=0;level<degree;level++){
      for(unsigned long  h_index=0;h_index<h_comb_to_degree.at(level).size();h_index++){
	current_time=Culling_size[h_comb_map2[level][h_index]]*pow(HBF[h_comb_map2[level][h_index]],latest_iteration-culling_iteration[h_comb_map2[level][h_index]]+iteration_index+1)*time_cost[level];
	//cout<<",culling_iteration["<<h_comb_map2[level][h_index]<<"]:"<<culling_iteration[h_comb_map2[level][h_index]]<<",HBF:"<<HBF_AVG<<",current_time:"<<current_time<<endl;
	/*if((current_time<iter_best_time_threshold)&&(Best_h_combs.find(h_comb_map2[level][h_index])==Best_h_combs.end())){
	  //cout<<"subset ";print(h_comb_map2[level][h_index],false);cout<<" qualifies to third phase"<<endl;
	  best_h_comb_list.push_back(h_comb_map2[level][h_index]);
	  Best_h_combs[h_comb_map2[level][h_index]]=true;
	  }*/
      }
    }
  }
  myfile<<"N\tF_bound\tIter_indx\t";
  string temp_h_comb;

  for(unsigned comb_index=0;comb_index<best_h_comb_list.size();comb_index++){
    temp_h_comb="";
    for(int i=0;i<best_h_comb_list.at(comb_index).size();i++){
      if(best_h_comb_list.at(comb_index).test(i)){
	//myfile<<i;
	temp_h_comb+= boost::lexical_cast<std::string>(i);
	if(i<(best_h_comb.size()-1)){
	  //myfile<<",";
	  temp_h_comb+=",";
	} 
      }
    }
    myfile<<temp_h_comb<<"_nodes\t";
    myfile<<temp_h_comb<<"_time\t";
  }

  myfile<<endl;
#ifdef CROSSOVER_PREDICTION
  //print any previous crossovers
  for(unsigned prev_iter_index=0;prev_iter_index<best_prev_iter_heuristic.size();prev_iter_index++){
    if(prev_iter_index>0){
      if(best_prev_iter_heuristic.at(prev_iter_index)==best_prev_iter_heuristic.at(prev_iter_index-1)){
	continue;
      }
    }

    string temp_h_comb;
      temp_h_comb="";
    for(int i=0;i<best_prev_iter_heuristic.at(prev_iter_index).size();i++){
      if(best_prev_iter_heuristic.at(prev_iter_index).test(i)){
	//myfile<<i;
	temp_h_comb+= boost::lexical_cast<std::string>(i);
	if(i<(best_prev_iter_heuristic.at(prev_iter_index).size()-1)){
	  //myfile<<",";
	  temp_h_comb+=",";
	} 
      }
    }
    myfile<<"best h_comb at prev_iter("<<prev_iter_index<<"):"<<temp_h_comb<<endl;
  }
#endif 

  for(unsigned iteration_index=0;iteration_index<10;iteration_index++){
    myfile<<problem_index<<"\t"<<current_F_bound+iteration_index*ITER_STEP<<"\t"<<iteration_index;
    for(unsigned comb_index=0;comb_index<best_h_comb_list.size();comb_index++){
      cout<<"h_comb:";
      unsigned level=0;
    for(int i=0;i<best_h_comb_list.at(comb_index).size();i++){
      if(best_h_comb_list.at(comb_index).test(i)){
	cout<<i;
	cout<<",";
	level++;
      }
    }
    cout<<"level:"<<level<<endl;
	myfile<<"\t"<<Culling_size[best_h_comb_list.at(comb_index)]*pow(HBF[best_h_comb_list.at(comb_index)],latest_iteration-culling_iteration[best_h_comb_list.at(comb_index)]+iteration_index+1)<<"\t"<<Culling_size[best_h_comb_list.at(comb_index)]*pow(HBF[best_h_comb_list.at(comb_index)],latest_iteration-culling_iteration[best_h_comb_list.at(comb_index)]+iteration_index+1)*time_cost[level-1];
	if(comb_index<(best_h_comb_list.size()-1)) myfile<<"\t";
    }
    myfile<<endl;
  }
    
  myfile.close();

  //cout<<"before throw"<<endl;fflush(stdout);
  //cerr<<"exit for debugging crossovers"<<endl;throw 20;
  //cout<<"after throw"<<endl;fflush(stdout);

}
double HST::calculate_time_costs_specific(boost::dynamic_bitset<> h_comb,const vector<Heuristic *> heuristics){
  //double time_cost=1.1/pow(10,6);//node creation and expanding average cost
  double time_cost=node_gen_and_exp_cost;//node creation and expanding meassured avg cost for current problem
  for (size_t i = 0; i < h_comb.size(); i++){
    if(h_comb.test(i)){//add heuristic time cost to total if heuristic is active
      time_cost+=(heuristics[i]->get_measured_TPN()/gen_to_eval_ratio);
    }
  }
  return time_cost;
}
void HST::calculate_time_costs(){
  time_cost.clear();
  time_cost.push_back(4.25/pow(10,6));
  cout<<"Time[0] per node:"<<scientific<<time_cost.back()<<endl;fflush(stdout);
  for(int i=0;i<(hset_size-1);i++){
    time_cost.push_back(time_cost.back()+(1.1/pow(10,7)));
    cout<<"Time["<<i+1<<"] per node:"<<scientific<<time_cost.back()<<endl;fflush(stdout);
  }
}
boost::dynamic_bitset<> HST::get_best_h_comb(){
  return best_h_comb;
}
void HST::get_best_h_comb(boost::dynamic_bitset<> &selec_heur){
  selec_heur=best_h_comb;
}
/*bool HST::check_duplicate(const State* current_state){
  check_duplicate(current_state,NULL);
}*/
bool HST::check_duplicate(const State* current_state,const Operator* generating_op){
  //cout<<"calling check_duplicate"<<endl;
  //if duplicate check, only add to Hash table if state is new, otherwise return true
  int depth=0;//leave like this if root state
  static unsigned long revisited_states_at_lower_depth=0;
  //static bool print_once=revaluation_random_sampled_leaves;
  //static map<State,int> first_visited_iter; 
  if(generating_op){
    depth=get_last_depth()+generating_op->get_cost();
  }
  //cout<<"nodes.size:"<<nodes->size()<<endl;
  HashTable::iterator result = nodes->find(StateProxy(current_state));//is the state already in the hash table?
   if (result==nodes->end()) {
      // This is a new entry: Must give the state permanent lifetime.
      //cout<<"New State:";current_state->dump();cout<<"at depth"<<depth<<endl;
      //result.first->first.make_permanent();
      //first_visited_iter[*current_state]=iter_index;
      //cout<<"New entry,skipping adding until we decide to add to HST(evaluate state)"<<endl;
      return false;
   }
   else{
      //cout<<"Old_entry"<<endl;
      //cout<<"depth:"<<depth<<",Repeated State:";current_state->dump();
      if(depth>result->second.first){//duplicated at bigger depth so byebye
	//cout<<"New state is duplicated at depth:"<<depth<<",shortest depth:"<<result->second.first<<endl;
	//cout<<"Exiting for debugging"<<endl;exit(0);
	return true;
      }
      else if(depth==result->second.first){//posibly revisiting state from previous iter
	//if(revaluation_random_sampled_leaves){
	  //if(print_once){
	    //cout<<"state is revisited at same depth so treated as duplicate"<<endl;
	    //print_once=false;
	  //}
	  //return true;//Every visit is treated as the second visit, that is why the state is in the duplicate_check_list
	//}
	if(result->second.second==false){//state is revisited for 1st time this iter
	  //cout<<"revisiting state for the 1st time in this iteration"<<endl;
	  result->second.second=true;//marked as duplicate for next encounters in this iteration
	  return false;
      }
	else{//revisited for 2nd or more times at same depth
	  //cout<<"State:"<<endl;current_state->inline_dump();cout<<"is duplicated at the same depth:"<<depth<<endl;
	  //exit(0);
	  return true;
	}
      }
      else{//found depth is lower!
      //cout<<"revisited state";current_state->dump();cout<<" with lower depth:"<<depth<<",original depth:"<<result->second.first<<",inconsistent heuristic?, then need to deal with it on this function "<<endl;
      //exit(0);
      
      //This means that this node is going to be re-expanded again at a lower F-value, the heuristics are assumed to be consistent (mostly, lmcut is special case), hence we need to substract the children of these nodes so they do not get added twice
	//vector<const Operator *> applicable_ops;
	//g_successor_generator->generate_applicable_ops(*current_state, applicable_ops);
	//cout<<"found state at lower depth!"<<endl;
	//substract+=applicable_ops.size();
	revisited_states_at_lower_depth++;
      /*if(revisited_states_at_lower_depth%100000==0){
	cout<<"FOUND "<<revisited_states_at_lower_depth<<" AT LOWER DEPTH,HEUR MUST BE INCONSISTENT!"<<endl;
      }*/
	result->second.first=depth;
	return false;
      }
    }
    return true;
}
bool HST::check_duplicate(const State* current_state,const Operator* generating_op,vector<Heuristic *> &heuristics){
  //cout<<"calling check_duplicate"<<endl;
  //if duplicate check, only add to Hash table if state is new, otherwise return true
  int depth=0;//leave like this if root state
  static unsigned long revisited_states_at_lower_depth=0;
  //static bool print_once=revaluation_random_sampled_leaves;
  //static map<State,int> first_visited_iter; 
  if(generating_op){
    depth=get_last_depth()+generating_op->get_cost();
  }
  //cout<<"nodes.size:"<<nodes->size()<<endl;
  HashTable::iterator result = nodes->find(StateProxy(current_state));//is the state already in the hash table?
   if (result==nodes->end()) {
      // This is a new entry: Must give the state permanent lifetime.
      //cout<<"New State:";current_state->dump();cout<<"at depth"<<depth<<endl;
      //result.first->first.make_permanent();
      //first_visited_iter[*current_state]=iter_index;
      //cout<<"New entry,skipping adding until we decide to add to HST(evaluate state)"<<endl;
      return false;
   }
   else{
      //cout<<"Old_entry"<<endl;
      //cout<<"depth:"<<depth<<",Repeated State:";current_state->dump();
      if(depth>result->second.first){//duplicated at bigger depth so byebye
	//cout<<"New state is duplicated at depth:"<<depth<<",shortest depth:"<<result->second.first<<endl;
	//cout<<"Exiting for debugging"<<endl;exit(0);
	return true;
      }
      else if(depth==result->second.first){//posibly revisiting state from previous iter
	//if(revaluation_random_sampled_leaves){
	  //if(print_once){
	    //cout<<"state is revisited at same depth so treated as duplicate"<<endl;
	    //print_once=false;
	  //}
	  //return true;//Every visit is treated as the second visit, that is why the state is in the duplicate_check_list
	//}
	if(result->second.second==false){//state is revisited for 1st time this iter
	  //cout<<"revisiting state for the 1st time in this iteration"<<endl;
	  result->second.second=true;//marked as duplicate for next encounters in this iteration
	  return false;
      }
	else{//revisited for 2nd or more times at same depth
	  //cout<<"State:"<<endl;current_state->inline_dump();cout<<"is duplicated at the same depth:"<<depth<<endl;
	  //exit(0);
	  return true;
	}
      }
      else{//found depth is lower!
      //cout<<"revisited state";current_state->dump();cout<<" with lower depth:"<<depth<<",original depth:"<<result->second.first<<",inconsistent heuristic?, then need to deal with it on this function "<<endl;
      //exit(0);
      
      //This means that this node is going to be re-expanded again at a lower F-value, the heuristics are assumed to be consistent (mostly, lmcut is special case), hence we need to substract the children of these nodes if the first time they got added at the wrong depth, their F-value==current F value meant that the nodes were not substracted originally
      //We are doing this wrong, some tiems we get larger substract counter than the actual counterpart! so we will comment it for now, but this means that whenever we re-encounter state twice and the first time it was ont he F-boundary and the second time it was bellow the F-boundary, we will have added the same state twice wrongly.  This will bias selection towards more accurate heursitics.
	   vector<const Operator *> applicable_ops;
	g_successor_generator->generate_applicable_ops(*current_state, applicable_ops);
//	//cout<<"found state at lower depth!"<<endl;
	//substract+=applicable_ops.size(); 
	boost::dynamic_bitset<> h_index(heuristics.size());h_index.set();
	bool at_least_one_strong_heur=false;
	for(int i=0;i<heuristics.size();i++){
	  heuristics[i]->evaluate(*current_state); 
	  int H=0;
	  if(heuristics[i]->is_dead_end()||h_capped.at(i)||get_earliest_depth_h_culled(i)<depth){
	    H=INT_MAX/2;
	  }
	  else{
	    H=heuristics[i]->get_heuristic();
	  }
	  if((!at_least_one_strong_heur)&&(H+depth)<=(get_current_F_bound())){
	    if(strong_heur.find(i)!=strong_heur.end()){
	      at_least_one_strong_heur=true;//at least one strong heur expands this node
	    }
	  }
	  if((H+depth)<get_current_F_bound()&&((H+result->second.first)==(get_current_F_bound()))){
	    if(h_capped.at(i)||get_earliest_depth_h_culled(i)<depth||heuristics[i]->is_dead_end()){//path already culled for heuristic or dead_end
	      continue;
	    }
	    //cout<<"\tremoving from counter suboptimal expansion for h"<<i<<endl;
	    h_index.reset(i);
	  }
	}
	if(!at_least_one_strong_heur){
//	  //cout<<"No strong heur expanded this node so not adding to substract counter!"<<endl;
	}
	else if(h_index.count()!=heuristics.size()){
	  //add_to_substract_counter(&h_index,0);//just to see the effects of dropping wrongly added duplicates
	  add_to_substract_counter(&h_index,applicable_ops.size());
	  //  if(h_index.count()==0){
	    //cout<<"DupChck-All heuristics F-value bellow F-boundary!!"<<endl;
	  //}
	}
	revisited_states_at_lower_depth++;
      if(revisited_states_at_lower_depth%100000==0){
	cout<<"FOUND "<<revisited_states_at_lower_depth<<" AT LOWER DEPTH,HEUR MUST BE INCONSISTENT OR SAMPLING IS INCONSTENT BECAUSE OF F_boundary differences!"<<endl;
	cout<<"heuristics size:"<<heuristics.size()<<endl;//just not to get the unusued error about heursitics, we need to debug and figure out how to account for the re-openeed states described in the previous comment
      }
	result->second.first=depth;
	return false;
      }
    }
    return true;
}
void HST::add_dead_end_nodes(){
  dead_end_nodes++;
}

void HST::create_h_counters(map < vector<short>, long,shortvect_comp2 > *sampled_F_values, int F_bound){
  cout<<"Calculating CCs for F_bound:"<<F_bound<<endl;
  cout<<"size of sampled F_values:"<<sampled_F_values->size()<<endl;
  h_counter.clear();
  map<boost::dynamic_bitset<>,unsigned long,bitset_comp2> map_temp;
  h_counter.assign(hset_size+1,map_temp);
  boost::dynamic_bitset<> h_index(hset_size,0);
  int F_min=INT_MAX;
  for(map<vector<short>, long,shortvect_comp2>::iterator iter=sampled_F_values->begin();iter!=sampled_F_values->end();iter++){
    F_min=INT_MAX;
    h_index.reset();
    for(int i=0;i<iter->first.size();i++){
      //cout<<iter->first.at(i)<<",";
      //cout<<"current F_max:";fflush(stdout);
      F_min=min(F_min,int(iter->first.at(i)));
      if(iter->first.at(i)>F_bound){
	h_index.set(i);
      }
      //cout<<"\tstored_F["<<i<<"]:"<<iter->first.at(i)<<",F_bound:"<<F_bound<<",partial h_index:"<<h_index<<endl;
    }
    //cout<<"F_max:"<<F_max<<",";fflush(stdout);
    //cout<<"Final h_index:"<<h_index<<endl;
    if(F_min<=F_bound){//we are only tracking internal nodes as we can not know which of the nodes in the sampled F_values are actually leaves for this iteration
      add_to_counter(&h_index,iter->second);
      //cout<<"added_to_counter h_index["<<h_index<<"]:"<<iter->second<<endl;
    }
    /*else{
      cout<<endl;
    }*/
  }
}
    
//  int depth=0;
//  bool visited_state=false;
//  string state_id="";
//  
//    
//  depth=lsearch_space.get_last_depth()+1;//current_node_depth=father_depth+1 
//  //cout<<"generating state_id"<<endl;
//  generate_state_id(current_state,&state_id);
//
//  //print_state(current_state);fflush(stdout);
//
//  if(visited[state_id]==NULL){
//    //cout<<"State:"<<state_id<<" visited 1st time at depth:"<<depth<<"culling:"<<gcmd_line.culling<<endl;
//    visited[state_id]=depth;//for next time we visit this state is a revisit
//    visited_state=false;//state visited for first time
//    count_visited++;
//  } //asuming monotone heuristic, so the first time we ever reached a state it was with the shortest possible path
//  else if(depth>visited[state_id]){//longer path
//    /*if( (visited[state_id]-depth)==1){
//      cout << "state_id_s:" << state_id << endl;
//      cout<<"Longer path which is not gfth has been found!"<<endl;
//      exit(1);
//    }*/
//    if( ((visited[state_id]-depth)>2) ){
//      cout<<"State3 is duplicated but not grandfather!"<<"culling:"<<gcmd_line.culling<<endl;
//      cout<<"Found duplicate state @ depth of:"<<depth<<"but original is depth of:"<<visited[state_id]<<endl;
//      exit(1);//want to check out if this condition ever happens
//    }
//    else{
//      //cout<<"Found duplicate state @ depth of:"<<depth<<"but original is depth of:"<<visited[state_id]<<"culling:"<<gcmd_line.culling<<endl;
//    }
//    visited_state=true;
//  }
//  else if(depth==visited[state_id]){//same depth
//    if((revisited[state_id]==NULL)&&(gcmd_line.culling!=(visited[state_id]+1))){//state revisited on succesor iteration
//      //cout<<"State:"<<state_id<<" revisited for first time at depth:"<<depth<<",culling:"<<gcmd_line.culling<<endl;
//      visited_state=false;//state as if visited for first time
//      revisited[state_id]=true;
//    }
//    else{//state revisited second time
//      //cout<<"State:"<<state_id<<" revisited not first time so culling at depth:"<<depth<<endl;
//      visited_state=true;
//      lsearch_space.add_alternative_path();//same optimal distance alternative path found
//    }
//  }
//  else{//depth lower, that is impossible!
//    cout<<"revisited "<< state_id << " state with lower depth:"<<depth<<" so something very wrong"<<endl;
//    //lsearch_space.extract_current_path();
//    exit(1);
//  }
//  return visited_state;
//}
void HST::print_Dot_Tree(){
  FILE *fp;
  char filename[MAX_LENGTH];
  int node_id=0;
  //int cluster_element=0,cluster_id=0;
  vector<vector<int> > clusters;
  //double EBF,ED;
  string move_oper;
  
  //clusters.resize(get_hg_tree_size());
  //cout<<"hg_tree_size"<<get_hg_tree_size()<<endl;
  cout<<"starting print_Dot_Tree"<<endl;fflush(stdout);
  sprintf(filename,"draw/DrawTree_C%d_H%d_N%d.dot",current_F_bound,hset_size,problem_index);
  cout<<"printing "<<filename<<endl;fflush(stdout);
  fp=fopen(filename,"w");
  fprintf(fp,"digraph g {\n");
  //fprintf(fp,"size=\"8,6\"; ratio=fill; node[fontsize=24];\n");
  fprintf(fp,"node [shape = record,height=.1];\n");
  for( node_id=0; node_id<get_TreeDraw_size() ; node_id++ ) {
    cout<<"node_id"<<node_id<<endl;fflush(stdout);
    move_oper=get_TreeDraw_op(node_id);
    /*move_oper="";
    if(op_name>0){
      if ( goperators[gop_conn[op_name].op]->name ) {
	move_oper+=goperators[gop_conn[op_name].op]->name;
	move_oper+=" ";
      }
      for ( int i=0; i<goperators[gop_conn[op_name].op]->num_vars; i++ ) {
	move_oper+=gconstants[gop_conn[op_name].inst_table[i]];
	move_oper+=" ";
      }
    }
    cout<<"move_oper:"<<move_oper<<endl;fflush(stdout);*/

    //fprintf(fp,"n%d[label = \"<f0> %d |<f1> %d\"];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id));
    if(get_TreeDraw_gfth_status(node_id)){
      fprintf(fp,"n%d[label = \"%d|%d|%s\",color=red,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_state_id(node_id));
    }
    else{
      /*if(Current_RIDA_Phase==SAMPLING_PHASE){
	cout<<"heuristics culled by node_id:"<<node_id;fflush(stdout);cout<<"is "<<get_TreeDraw_heuristic_culling_count(node_id)<<endl;fflush(stdout);
	switch(get_TreeDraw_heuristic_culling_count(node_id)){
	  case 0:
	    fprintf(fp,"n%d[label = \"%d|%d|%d|%d\",color=blue,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_heuristic_nodes(node_id),get_TreeDraw_heuristic_culling_count(node_id));
	    break;
	  case 1:
	    fprintf(fp,"n%d[label = \"%d|%d|%d|%d\",color=gray90,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_heuristic_nodes(node_id),get_TreeDraw_heuristic_culling_count(node_id));
	    break;
	  case 2:
	    fprintf(fp,"n%d[label = \"%d|%d|%d|%d\",color=gray70,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_heuristic_nodes(node_id),get_TreeDraw_heuristic_culling_count(node_id));
	    break;
	  case 3:
	    fprintf(fp,"n%d[label = \"%d|%d|%d|%d\",color=gray50,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_heuristic_nodes(node_id),get_TreeDraw_heuristic_culling_count(node_id));
	    break;
	  case 4:
	    cout<<"case 4 start"<<endl;fflush(stdout);
	    fprintf(fp,"n%d[label = \"%d|%d|%d|%d\",color=gray30,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_heuristic_nodes(node_id),get_TreeDraw_heuristic_culling_count(node_id));
	    cout<<"case 4 end"<<endl;fflush(stdout);
	    break;
	  case 5:
	    fprintf(fp,"n%d[label = \"%d|%d|%d|%d\",color=gray10,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_heuristic_nodes(node_id),get_TreeDraw_heuristic_culling_count(node_id));
	    break;
	  default:
	    fprintf(fp,"n%d[label = \"%d|%d|%d\",color=red,style=filled];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_heuristic_nodes(node_id));
	    break;
	}
	cout<<"out of switch"<<endl;fflush(stdout);
      }
      else{*/
	fprintf(fp,"n%d[label = \"%d|%d|%s\"];\n",node_id,get_TreeDraw_depth(node_id),get_TreeDraw_F(node_id),get_TreeDraw_state_id(node_id));
      //}
    }

    cout<<"printing label for node_id:"<<node_id<<endl;fflush(stdout);
    if(node_id>0){//root has no father
      cout<<"father:"<<get_TreeDraw_father(node_id)<<endl;fflush(stdout);
      cout<<"move_oper:"<<move_oper.c_str()<<endl;fflush(stdout);
      printf("\"n%d\"->\"n%d\"[label = \"%s\"];\n",get_TreeDraw_father(node_id),node_id,move_oper.c_str());fflush(stdout);
      fprintf(fp,"\"n%d\"->\"n%d\"[label = \"%s\"];\n",get_TreeDraw_father(node_id),node_id,move_oper.c_str());
    }
    cout<<"label printed"<<endl;fflush(stdout);
    /*if(clusters.size()>0){//no clusters
      clusters.at(get_TreeDraw_hg_node(node_id)).push_back(node_id);
    }*/
    cout<<"finished with node_id"<<node_id<<endl;fflush(stdout);
  }

  /*for( cluster_id=0; cluster_id<clusters.size() ; cluster_id++ ) {
    fprintf(fp,"subgraph \"cluster_HC%d\" { ",cluster_id);
    //fprintf(fp,"style=filled;color=grey[%d];",get_hg_size(cluster_id));//
    //get_hg_EBF_ED(cluster_id,&EBF,&ED);//getting EBF & ED for Cluster
    if(EBF>1.0){//show EBF & ED
      fprintf(fp,"label=\"s:%d|c:%d|EBF:%2.2f|ED:%2.2f\";",get_hg_size(cluster_id),get_hg_children(cluster_id),EBF,ED);//
    }
    else{
      fprintf(fp,"label=\"s:%d|c:%d\";",get_hg_size(cluster_id),get_hg_children(cluster_id));
    }
    for(cluster_element=0;cluster_element<clusters.at(cluster_id).size();cluster_element++){
      fprintf(fp,"n%d; ",clusters.at(cluster_id).at(cluster_element));
    }
    fprintf(fp,"}\n");


  }*/
  fprintf(fp,"}");

  fclose(fp);
  cout<<"finished print_Dot_Tree"<<endl;fflush(stdout);
}
void HST::switch_to_max_strong_and_comp(vector<Heuristic *> &heuristics){
  heuristics=strong_and_comp_heur;
  vector<int> chosen_heur;
  for (int i=0;i<heuristics.size();i++){
    //cout<<"using heuristic("<<real_heur_pos[i]<<endl;
    chosen_heur.push_back(real_heur_pos[i]);
  }
  cout<<"SWITCHED because too close to memory_limit to chosen_heur:"<<endl;
  update_hset_size(heuristics.size());
}
void HST::select_best_estimated_heuristic_subset(SearchSpace *search_space,const vector<pair<State,int> >* chosen_Hoff_Roots,const vector<Heuristic *> orig_heuristics,vector<Heuristic *> &heuristics,int current_f, int F_boundary_size_input){
  cout<<"calling select_best_estimated_heuristic_subset,current_f:"<<current_f<<",heuristics:"<<heuristics.size()<<endl;
  F_boundary_size=F_boundary_size_input;
  //heuristics.clear();
  //heuristics.push_back(orig_heuristics.at(1));
  //cout<<"chosen_heuristic:selected_heur(1) is:";heuristics.back()->print_heur_name();cout<<endl;
  //return;
  //backup landmarks for incremental-lmcut
  for (int i=0;i<orig_heuristics.size();i++){
    orig_heuristics[i]->backup_landmarks();
  }
  int depth;
  const Operator* next_op;
  const State* succ_state;
  const State* current_state=g_initial_state;
  vector<const Operator *> applicable_ops;
  int nodes_gen_iter=0;
  int nodes_culled_iter=0;
  h_capped.assign(orig_heuristics.size(),false);//This are those heuristics which we do not want to sample
  check_consistency=true;
  static long overall_nodes_culled;
  static long overall_nodes_gen;
  vector<pair<int,double> > best_heuristics_by_fitness;
  int Hof_Roots_Max_Samp_time=5;
   
  current_sampling_timer.reset();current_sampling_timer.resume();

  //vector<pair<int,double> > best_heuristics_by_fitness;
  set<int> best_heuristics_by_fitness_set;
  Current_RIDA_Phase=SOLVING_PHASE;
  update_original_hset_size(orig_heuristics.size());
  global_duplicate_check=true;cout<<"setting global_duplicate_check to true (A* duplicated nodes still apply)"<<endl;
  

  set_current_F_bound(current_f);//The current F_value for the next trench of open nodes is the F-boundary we are sampling
  //First determine which are strong heuristics
  vector<int> winning_h;
  vector<int> overall_winning_h(orig_heuristics.size(),0);
  //FITNESS CALCULATIONS
  /*for (size_t i = 0; i < orig_heuristics.size(); i++){
	//cout<<"\tf["<<i<<"]:"<<orig_heuristics[i]->get_heuristic()<<",h["<<i<<"]:"<<orig_heuristics[i]->get_heuristic()<<"g:"<<0<<",fitness:"<<orig_heuristics[i]->get_approx_mean_finite_h()<<endl;
	best_heuristics_by_fitness.push_back(pair<int,double>(i,orig_heuristics[i]->get_approx_mean_finite_h()));
  }*/
  /*boost::dynamic_bitset<> selec_heurs(orig_heuristics.size(),0);
  sort (best_heuristics_by_fitness.begin(), best_heuristics_by_fitness.end(),compare_pairs_int_double);
  if(best_heuristics_by_fitness.size()>0){//in case none of the heuristic we are using can not be categorized by fitness
    for(int i=(best_heuristics_by_fitness.size()-1);i>=(best_heuristics_by_fitness.size()-Degree);i--){
      cout<<"best_heuristic_by_fitness(,"<<best_heuristics_by_fitness.at(i).first<<",):,"<<best_heuristics_by_fitness.at(i).second<<endl;
      selec_heurs.set(best_heuristics_by_fitness.at(i).first);
      best_heuristics_by_fitness_set.insert(best_heuristics_by_fitness.at(i).first);
    }
  }*/
  //cout<<"not adding best heuristics by fitness"<<endl;best_heuristics_by_fitness_set.clear();

  //NOW LETS FIND THOSE HEURISTICS WHICH DO WELL ON THIS PROBLEM  
  double avg_TPN=0;
  for (size_t i = 0; i < heuristics.size(); i++){
    avg_TPN+=heuristics[i]->get_measured_TPN();
  }
  avg_TPN/=heuristics.size();
  cout<<"avg_TPN:"<<avg_TPN<<endl;
  
    cout<<"Sampling time preparations before HoF Roots:"<<current_sampling_timer()<<endl;
  for(int i=0;i<chosen_Hoff_Roots->size();i++){
    //if(current_sampling_timer()>0.05*F_boundary_time&&(!full_sampling))
    if(current_sampling_timer()>Hof_Roots_Max_Samp_time*F_boundary_time&&(!full_sampling)){

      cout<<"Finishing early strong_heur_checks at Hoff_Root:"<<i<<",current_sampling_timer:"<<current_sampling_timer<<",F_boundary_time:"<<F_boundary_time<<",maximum Hoff Roots sampling time:"<<Hof_Roots_Max_Samp_time<<endl;fflush(stdout);
      break;
    }
    compare_MAX_heuristics(&(chosen_Hoff_Roots->at(i).first),winning_h,orig_heuristics,avg_TPN);
    //cout<<"Finished with compare_MAX_heuristics"<<endl;fflush(stdout);
    for(size_t j=0;j<winning_h.size();j++){
      overall_winning_h.at(winning_h.at(j))++;
    }
  }
  //vector<Heuristic *> original_heuristics=heuristics;
  real_heur_pos.clear();
  heuristics.clear();
  int best_indiv_heur=0;
  long max_winning=0;
  for(size_t i=0;i<overall_winning_h.size();i++){
    if(overall_winning_h.at(i)>0){
      cout<<"F_bound:"<<get_current_F_bound()<<",winning_h("<<i<<"):"<<overall_winning_h.at(i)<<endl;fflush(stdout);
      //cout<<"F_bound:"<<get_current_F_bound()<<",capping h:"<<i<<",because it was never one of the best heuristics for any of the Hoff trees"<<endl;
      //h_capped.at(i)=true;
    }
    if(overall_winning_h.at(i)>max_winning){
      best_indiv_heur=i;
      max_winning=overall_winning_h.at(i);
    }
  }
  cout<<"HoF roots:"<<chosen_Hoff_Roots->size()<<",overall_winning_h:"<<best_indiv_heur<<",winning times:"<<max_winning<<endl;
  //now determine a good complementary level so that a max of 15 heuristics are left
    vector<int> ordered_overall_winning=overall_winning_h;
    sort(ordered_overall_winning.begin(), ordered_overall_winning.end());
    int last_heur=ordered_overall_winning.size()-1;
    for(int i=ordered_overall_winning.size()-1;i>=ordered_overall_winning.size()-15;i--){
      if(ordered_overall_winning.at(i)==0){
	break;
      }
      last_heur=i;
    }
    double Hoff_admit_thress=double(ordered_overall_winning.at(last_heur))/double(ordered_overall_winning.back());
    cout<<"Hoff_admit_thress="<<Hoff_admit_thress<<",last_heur:"<<last_heur<<endl;
  
  if(orig_heuristics.size()<15){
    Hoff_admit_thress=0;
    cout<<"Hoff_admit_thress="<<Hoff_admit_thress<<" because the available set is less than the 15 heurs maximum we deal with at the moment"<<endl;
  }
  else{
    cout<<"Hoff_admit_thress="<<Hoff_admit_thress<<",last_heur:"<<last_heur<<endl;
  }
  

  //
  //Finally get rid of the very weak heuristics in relation to the best individually performing one
  //for now we set it at 0%
  //double Hoff_admit_thress=0.01;//when using expensive heurs,e.g. landmarks
  //double Hoff_admit_thress=0.2;
  //double Hoff_strong_heur_thress=0.05;
  double Hoff_strong_heur_thress=0.5;//for debugging reasons
  //Hoff_admit_thress=0;//for debugging reasons
  cout<<"Hoff_strong_heur_thress="<<Hoff_strong_heur_thress<<",Hoff_admit_thress:"<<Hoff_admit_thress<<endl;
  int offset=0;
  strong_heur.clear();
  for(size_t i=0;i<overall_winning_h.size();i++){
    //we allow blind to be a strong heur, otherwise it gets greedily dropped while BFS can be the best option when the available heuristics are not "value for money"
    string heur_name=orig_heuristics[i]->get_heur_name();
    if((heur_name.find("blind")==string::npos)&&Hoff_strong_heur_thress!=0&&(overall_winning_h.at(i)==0||(int(Hoff_admit_thress*double(ordered_overall_winning.back()))>overall_winning_h.at(i)))){
	//&&(best_heuristics_by_fitness_set.find(i)==best_heuristics_by_fitness_set.end()))//we want to sample those heurs which we know have a high average fitness, just in case
      cout<<orig_heuristics[i]->get_heur_name()<<",F_bound:"<<get_current_F_bound()<<",winning:"<<overall_winning_h.at(i)<<",capping h:"<<i<<",because it was lower than the threshold of "<<max(1,int(Hoff_admit_thress*double(ordered_overall_winning.back())))<<endl;
      orig_heuristics[i]->set_stop_using(true);
      //h_capped.at(i)=true;
      offset++;
    }
    else{
      if(((heur_name.find("blind")!=string::npos))||((int(Hoff_strong_heur_thress*double(max_winning)))<=int(overall_winning_h.at(i)))){
	heuristics.push_back(orig_heuristics.at(i));
	real_heur_pos[i-offset]=i;
	cout<<"real_heur_pos["<<i-offset<<"]:"<<real_heur_pos[i-offset]<<endl;
	strong_heur.insert(i-offset);
	cout<<orig_heuristics[i]->get_heur_name()<<",added h(,"<<i<<",):"<<"as a strong heur,winning "<<overall_winning_h.at(i)<<" at new pos:,"<<i-offset<<",Thresshold:"<<Hoff_strong_heur_thress*double(max_winning)<<endl;
      }
      else{
	if((orig_heuristics[i]->get_measured_TPN()/10.0)>avg_TPN){
	  cout<<orig_heuristics[i]->get_heur_name()<<",skipping adding heur("<<i<<"), as a complementary heur, it is too expensive compared to avg_tpn"<<endl;
	  orig_heuristics[i]->set_stop_using(true);
	  offset++;
	}
	else{ 
	  heuristics.push_back(orig_heuristics.at(i));
	  real_heur_pos[i-offset]=i;
	  cout<<"real_heur_pos["<<i-offset<<"]:"<<real_heur_pos[i-offset]<<endl;
	  cout<<orig_heuristics[i]->get_heur_name()<<",added h(,"<<i<<",):"<<"as a complementary heur,winning "<<overall_winning_h.at(i)<<" at new pos:,"<<i-offset<<endl;
	}
      }
    }
  }
  //We also need to check if the F-boundary is relevant to all strong heuristics,
  //if for any heuristic we find that the F-boundary is either under or over the current F-boundary
  //we need to update the current F-boundary to the min of all strong heuristics over the HoF roots
  //Otherwise, we end up with one heuristic not expanding any nodes due to the substract counters!
  int min_strong_F_boundary=0;
  vector<int> root_min_strong_f(strong_heur.size(),INT_MAX/2);
  for(int i=0;i<chosen_Hoff_Roots->size();i++){
    int j=0;
    for(set<int>::iterator iter_strong=strong_heur.begin();iter_strong!=strong_heur.end();iter_strong++){
      orig_heuristics[real_heur_pos[*iter_strong]]->evaluate(chosen_Hoff_Roots->at(i).first);
      int h=orig_heuristics[real_heur_pos[*iter_strong]]->get_heuristic();
      //cout<<"\ti:"<<i<<",j:"<<j<<",h:"<<h;fflush(stdout);
      if((h+chosen_Hoff_Roots->at(i).second)>=get_current_F_bound()){
	root_min_strong_f.at(j)=min(root_min_strong_f.at(j),h+chosen_Hoff_Roots->at(i).second);//h+depth of root
      }
      //cout<<",depth:"<<chosen_Hoff_Roots->at(i).second<<endl;fflush(stdout);
      j++;
    }
  }
    
  for(int i=0;i<strong_heur.size();i++){
    cout<<"next F boundary for h("<<i<<"):"<<root_min_strong_f.at(i)<<endl;
    if(root_min_strong_f.at(i)==(INT_MAX/2)){//the heur was always bellow the F-bound, can happen to blind for example
      root_min_strong_f.at(i)=get_current_F_bound();
    }
    min_strong_F_boundary=max(min_strong_F_boundary,root_min_strong_f.at(i));
  }
  cout<<"setting F-boundary to minimum common strong F-boundary value of:"<<min_strong_F_boundary<<endl;
  set_F_bound(min_strong_F_boundary);
  if(current_f!=min_strong_F_boundary){
    common_sampling_F_boundary=min_strong_F_boundary;
  }

  //HACK
  //We create an emergency call in case the final selection we make runs out of memory, the idea is we might get lucky with maximizing the strong heuristics.  The best approach would be to estimate the best combination given the remaining time and memory limits but we do not have time to deal with this properly before IPC 2014
  create_emergency_strong_heur_call(strong_heur,heuristics);
  strong_and_comp_heur=heuristics;

  double max_TPN=0;
  //now calculate the node_time_adjusted_reval as a function of the strongest heuristic
   for(set<int>::iterator iter_strong=strong_heur.begin();iter_strong!=strong_heur.end();iter_strong++){
     max_TPN=max(max_TPN,orig_heuristics[real_heur_pos[*iter_strong]]->get_measured_TPN());
   }
   node_time_adjusted_reval=0.5/(node_gen_and_exp_cost+max_TPN);
   node_time_adjusted_reval=max(node_time_adjusted_reval,10);
   cout<<"F_bound:"<<get_current_F_bound()<<",starting node_time_adjusted_reval:"<<node_time_adjusted_reval<<endl;
   //node_time_adjusted_reval=197776; cout<<"F_bound:"<<get_current_F_bound()<<",starting node_time_adjusted_reval:"<<node_time_adjusted_reval<<endl;
    update_hset_size(heuristics.size());
    cout<<"hset_size reduce from "<<orig_heuristics.size()<<" to:"<<heuristics.size()<<endl;
    if(heuristics.size()==1){
      cout<<"Only one strong heuristic left and no complementaries, so chosen_heuristic:selected_heur(0) is:";heuristics.back()->print_heur_name();cout<<endl;
        for (int i=0;i<orig_heuristics.size();i++){
	orig_heuristics[i]->restore_backed_up_landmarks();
      }
	delete nodes;nodes = new HashTable;//removed all visited HUST states from memory
	reset_HST(); 
      return;
    }
    cout<<"deleting all previously visited nodes"<<endl;
    delete nodes;nodes = new HashTable;//removed all visited HUST states from memory
    reset_h_counters();
    revaluation_random_sampled_leaves=true;//so that every revisitied state is treated as duplicate if found at same depth, look at check_duplicate function for details
    lsearch_space.clear();//just in case

    cout<<"Sampling time preparations after evaluating HoF Roots:"<<current_sampling_timer()<<endl;
    current_sampling_timer.reset();current_sampling_timer.resume();
    last_sampled_HoF_Root=0;
    cout<<"F_bound:"<<get_current_F_bound()<<",chosen_Hoff_Roots:"<<chosen_Hoff_Roots->size()<<endl;
    bool demoted_heur=false;
   for (int i=0;i<chosen_Hoff_Roots->size(); i++) 
   {
     if(demoted_heur){
      cout<<"deleting all previously visited nodes"<<endl;
      delete nodes;nodes = new HashTable;//removed all visited HUST states from memory
      reset_h_counters();
      revaluation_random_sampled_leaves=true;//so that every revisitied state is treated as duplicate if found at same depth, look at check_duplicate function for details
      lsearch_space.clear();//just in case
      i=0;//Back to initial root
      demoted_heur=false;
     }
     const State* S=new State(chosen_Hoff_Roots->at(i).first);
     int root_h_max=0;
     int root_h_min=INT_MAX/2;
     depth=chosen_Hoff_Roots->at(i).second;
     //Otherwise the landmarks order gets messed up and we get occasional core dumps   
     for (int j=0;j<heuristics.size();j++){
       //In case we demoted a heursitic which set the F-boundary, we always make sure the F-boundary for the root node we are sampling is the max f value of all the remaining strong heursiticss
       if(strong_heur.find(j)!=strong_heur.end()){
	 heuristics[j]->evaluate(*S);
	 root_h_max=max(root_h_max,heuristics[j]->get_heuristic());
	 root_h_min=min(root_h_min,heuristics[j]->get_heuristic());
	 //cout<<"Root:"<<i<<",evaluating strong heur:"<<heuristics[j]->get_heur_name()<<",h-value:"<<heuristics[j]->get_heuristic()<<",F-value:"<<heuristics[j]->get_heuristic()+depth<<endl;
	 //S->inline_dump();
       }
	//heuristics[j]->restore_backed_up_landmarks();
	//heuristics[j]->backup_landmarks();
	string heur_name=heuristics[j]->get_heur_name();
	if(heur_name.find("incremental_lmcut")!=string::npos){
	  heuristics[j]->erase_current_landmarks();
	  heuristics[j]->set_keep_frontier(true);//otherwise we get core dumps because regular lmcut_incremental assumes A* search, so no backtrack.  Need to study how saved landmarks work in the "IDA*" version
	}
	
	 /*  if(heur_name.find("regular_lm_cut")!=string::npos){//HACK:lmcut crashes sometimes when called from lmcut_incremental, trying to clear it out
	  heuristics[j]->initialize_hack();
	}*/
      }
     //cout<<"Root:"<<i<<" Out of "<<chosen_Hoff_Roots->size()<<",memory:"<<get_peak_memory_in_kb()<<endl;fflush(stdout);
     //if((current_sampling_timer()>1.0)&&(current_sampling_timer()>0.10*F_boundary_time&&(!one_time_sampling)))//at least 1 seconds sampling
     last_sampled_HoF_Root=i;
     if((current_sampling_timer()>sampling_time_limit)&&(!full_sampling)){//at least 1 seconds sampling
       cout<<"Finishing sampling early at HoF root:"<<last_sampled_HoF_Root<<",current_sampling_time:"<<current_sampling_timer.stop()<<",f_boundary_time:"<<F_boundary_time<<endl;
       break;
     }
     if((root_h_min+depth)>get_current_F_bound()){
       cout<<"Skipping Root"<<i<<",all strong heuristics cull the root node"<<endl;
     }
     //Initializations
     hset_size=heuristics.size();
     reset_earliest_depth();
     nodes_gen_iter=1;//Only need to add root, succesor nodes will be added as they are generated
     nodes_culled_iter=0;
     //cout<<"nodes_gen_iter:"<<nodes_gen_iter<<",nodes_culled_iter:"<<nodesculled_iter<<endl;
    
     current_state=S;
     //no need to adjust F-boundary for each root, as we are passing inital g-value to the root node
     //set_current_F_bound(root_h_max+depth);
     cout<<"Root:"<<i<<",new_F_boundary:"<<get_current_F_bound()<<endl;
     //cout<<"Root State #"<<i<<":";S->dump_fdr();cout<<",depth:"<<chosen_Hoff_Roots->at(i).second<<endl;fflush(stdout);
     //cout<<"Current State #"<<i<<":";current_state->dump_pddl();
    //int h_max=evaluate_MAX_node(S,depth);
     applicable_ops.clear();
     g_successor_generator->generate_applicable_ops(*S, applicable_ops);
     //int h_min=evaluate_HUST_node(S,depth,applicable_ops.size()+1,heuristics);//add 1 to number of children to track the root node as well in h_counters
     int h_min=evaluate_HUST_node(S,depth,applicable_ops.size(),heuristics);//No need to add 1, we only want to calculate the additional nodes for this F-boundary
     //cout<<"Root"<< i <<"added "<<applicable_ops.size()<<" to counter,";current_state->inline_dump();cout<<endl;
     //cout<<",h_min:"<<h_min<<endl;
       if(h_min==(INT_MAX/2)){
	 cout<<"Skipping Root node "<<i<<",no strong heuristic expands it for the next F-boundary"<<endl;
	 continue;
       }
     /*  if(h_min==(INT_MAX/2)){
       cout<<"All strong heurs cull this root, but this is a root node for the next f-boundary so at least one should open, pls debug me!!!"<<endl;fflush(stdout);exit(0);
       for (int j=0;j<heuristics.size();j++){
	 heuristics[j]->evaluate(chosen_Hoff_Roots->at(i).first);
	 cout<<heuristics[j]->get_heur_name()<<",h:"<<heuristics[j]->get_heuristic()<<",g:"<<depth<<",F:"<<get_current_F_bound()<<",state:";S->inline_dump();cout<<endl;
       }
	//heuristics[j]->restore_backed_up_landmarks();
     }*/
       //cout<<"Adding root to HST"<<endl;fflush(stdout);
     add_to_HST(S,NULL,h_min,depth);
     //NEED TO ADD HoF Roots and their childrens to expand all counter, unless we re-evaluate parent node when ffetched
     boost::dynamic_bitset<> h_index(heuristics.size(),false);
     //add_to_counter(&h_index,1+applicable_ops.size());
     //cout<<"adding to root counter:";fflush(stdout);cout<<h_index<<" one unit"<<endl;fflush(stdout);
     //add_to_counter(&h_index,1);
     /*int h_max=0;
     for (size_t j = 0; j < heuristics.size(); j++){
       heuristics[j]->evaluate(chosen_Hoff_Roots->at(i).first);
       h_max=max(h_max,heuristics[j]->get_heuristic());
     }*/
     //cout<<"State #"<<i<<",State:";chosen_Hoff_Roots->at(i).first.dump();cout<<",depth:"<<chosen_Hoff_Roots->at(i).second<<",h_min:"<<h_min<<endl;
     /*SearchNode root_node = search_space->get_node(chosen_Hoff_Roots->at(i).first);
     if(root_node.is_new()){
       cout<<"Root state is new"<<endl;
     }
     else{
       cout<<"Root state is old"<<endl;
     }*/
		 

     //cout<<"Memory before Sampling Root #,"<<i<<",:,"<<get_peak_memory_in_kb()<<endl;

     demoted_heur=false;
     while(!empty()){//keep generating and backtracking until no more nodes
       //if(i>50){
	 //cout<<"checking node_time_adjusted_reval,nodes_gen_iter:"<<nodes_gen_iter<<endl;fflush(stdout);
       //}
       if(nodes_gen_iter%node_time_adjusted_reval==0){//check memory is still within sampling bounds and also eliminate any runaway heuristics
	 //cout<<"checking node_time_adjusted_reval,nodes_gen_iter:"<<nodes_gen_iter<<endl;fflush(stdout);
	 if(memory_limit<get_peak_memory_in_kb()){
	   cout<<"nodes_gen_iter:"<<nodes_gen_iter;fflush(stdout);
	    
	   cout<<"exiting sampling, memory limit is:"<<memory_limit<<" and current memory is:"<<get_peak_memory_in_kb()<<endl;
	   break;
	 }
	  /* if(memory_limit<get_peak_memory_in_kb()||g_timer()>time_limit){
	    cout<<"Exiting, SAMPLING GOT OUT OF BOUNDS, SEE FOLLOWING LINES!:"<<endl;
	    cout<<"Nodes gen iter:"<<nodes_gen_iter<<", No limit on nodes_gen_iter, just info"<<endl;
	    cout<<"Exiting, memory limit is:"<<memory_limit<<" and current memory is:"<<get_peak_memory_in_kb()<<endl;
	    cout<<"Exiting, time limit is:"<<time_limit<<" and current time is:"<<g_timer()<<endl;
	    exit(0);
	  }*/

	  if(strong_heur.size()>1){//check if we have runaway heurisitc
	    //cout<<"checking for runaway heur,nodes_gen_iter:"<<nodes_gen_iter<<endl;fflush(stdout);
	    map<int,long> indiv_heur_nodes;
	    map<int,double> indiv_times;
	    //indiv_times.resize(.size());
	   //first calculate individual heuristics number of nodes
	    for ( unsigned level=0 ;level<h_counter.size(); level++ ){
	      for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter[level].begin(); it_h_values != h_counter[level].end(); it_h_values++){
		 //cout<<"it->first"<<","<<it_h_values->first<<","<<it_h_values->second<<endl;
		 for (std::set<int>::iterator it=strong_heur.begin(); it!=strong_heur.end(); ++it){
		   if(!it_h_values->first.test(*it)){//heuristic expands node
		     indiv_heur_nodes[*it]+=it_h_values->second;
		     //cout<<"\t Adding "<<it_h_values->second<<"to heur:"<<bit<<endl;
		   }
		 }
	       }
	      //NOW do the same but for the substract counters
	      //commented out, the heursitic might take too long to reach current F-boundary, then we need to demote, even if most nodes  have been found at suboptimal  depth!
	      /*for (map<boost::dynamic_bitset<>,unsigned long>::const_iterator it_h_values= h_counter_substract[level].begin(); it_h_values != h_counter_substract[level].end(); it_h_values++){
		 //cout<<"it->first"<<","<<it_h_values->first<<","<<it_h_values->second<<endl;
		 for (std::set<int>::iterator it=strong_heur.begin(); it!=strong_heur.end(); ++it){
		   if(!it_h_values->first.test(*it)){//heuristic expands node
		     indiv_heur_nodes[*it]-=it_h_values->second;
		     //cout<<"\t Adding "<<it_h_values->second<<"to heur:"<<bit<<endl;
		   }
		 }
	      }
	    */
	    }
	    double best_indiv_heur_time=INT_MAX;
	    if(gen_to_eval_ratio==0){
	      cout<<"gent_to_eval has not been set, debug now!"<<endl;
	      exit(0);
	    }
	    for (std::map<int,long>::iterator it=indiv_heur_nodes.begin(); it!=indiv_heur_nodes.end(); ++it){
	     indiv_times[it->first]=(double(it->second)*(node_gen_and_exp_cost+orig_heuristics[real_heur_pos[it->first]]->get_measured_TPN()/gen_to_eval_ratio));
	     //cout<<"individual h("<<it->first<<"),real_pos:"<<real_heur_pos[it->first]<<"heur_cost:"<<orig_heuristics[real_heur_pos[it->first]]->get_measured_TPN()<<",gen_to_eval:"<<gen_to_eval_ratio<<",nodes:"<<it->second<<",time:"<<indiv_times[it->first]<<endl;
	     if(best_indiv_heur_time>indiv_times[it->first]){
	       best_indiv_heur_time=indiv_times[it->first];
	     }
	    }
	    if(best_indiv_heur_time==INT_MAX){
	      cout<<"best_indiv_heur_time is INT_MAX,check what is going on!"<<endl;
	      for (std::map<int,long>::iterator it=indiv_heur_nodes.begin(); it!=indiv_heur_nodes.end(); ++it){
	     cout<<"individual h("<<it->first<<"),real_pos:"<<real_heur_pos[it->first]<<"heur_cost:"<<orig_heuristics[real_heur_pos[it->first]]->get_measured_TPN()<<",gen_to_eval_ratio:"<<gen_to_eval_ratio<<",nodes:"<<it->second<<",time:"<<indiv_times[it->first]<<endl;
	      }

	    }

	   ///Now demote to complementary any individual heuristic whose time is 50 times or more bigger than the fastest single heuristic
	   if(strong_heur.size()>1&&best_indiv_heur_time!=INT_MAX){
	     for (std::map<int,long>::iterator it=indiv_heur_nodes.begin(); it!=indiv_heur_nodes.end(); ++it){
	       //double time_cost=node_gen_and_exp_cost;//node creation and expanding average cost
	       //time_cost+=heuristics[real_heur_pos[it->first]]->get_measured_TPN()/gen_to_eval_ratio;
	       if(indiv_times[it->first]>50.0*best_indiv_heur_time&&(indiv_times[it->first]>0.5)){
		 cout<<"demoting heuristic:";cout<<it->first;orig_heuristics[real_heur_pos[it->first]]->print_heur_name();cout<<",best_indiv_time:"<<best_indiv_heur_time<<",indiv_time:"<<indiv_times[it->first]<<"indiv_nodes:,"<<it->second<<endl;fflush(stdout);
		 strong_heur.erase(it->first);
		 demoted_heur=true;//We need to make sure if we demote the heuristic that the F-boundary of the sampled root node is the max of the strong heuristics, then we get back to the initial root node and start sampling again
		 cout<<",strong_heurs left:"<<strong_heur.size()<<endl;fflush(stdout);
		 if((orig_heuristics[real_heur_pos[it->first]]->get_measured_TPN()/10.0)>avg_TPN){
		   cout<<"stopping using heuristic("<<it->first<<"),real_pos:"<<real_heur_pos[it->first]<<" as it is too expensive to be complementary"<<endl;fflush(stdout);
		   orig_heuristics[real_heur_pos[it->first]]->set_stop_using(true);
		   double max_TPN=0;
		   for(set<int>::iterator iter_strong=strong_heur.begin();iter_strong!=strong_heur.end();iter_strong++){
		     max_TPN=max(max_TPN,orig_heuristics[real_heur_pos[*iter_strong]]->get_measured_TPN());
		   }
		   node_time_adjusted_reval=0.5/(node_gen_and_exp_cost+max_TPN);
		   node_time_adjusted_reval=max(node_time_adjusted_reval,10);
		   cout<<"node_time_adjusted_reval recalculated to remaining most expensive strong heur,nodes for half a second now:"<<node_time_adjusted_reval<<endl;
		 }

	       }
	     }
	     //If only one heuristic is left, then we are finished sampling!
	     int remaining_heurs=0;
	     //for (std::map<int,long>::iterator it=indiv_heur_nodes.begin(); it!=indiv_heur_nodes.end(); ++it){
	     for(int j=0;j<orig_heuristics.size();j++){
	       if(orig_heuristics.at(j)->is_using()){
		 remaining_heurs++;
		 //cout<<orig_heuristics.at(j)->get_heur_name()<<" is remaining_heur"<<endl;
	       }
	     }
	     //cout<<"remaining_heurs:"<<remaining_heurs<<endl;fflush(stdout);
	     if(remaining_heurs==1){//only one heuristic is left, so that is the best heuristic!
	       heuristics.clear();
	       for(int j=0;j<orig_heuristics.size();j++){
		 if(orig_heuristics[j]->is_using()){
		   heuristics.push_back(orig_heuristics[j]);
		 }
	       }
	       cout<<"Only one heuristic left, so chosen_heuristic:selected_heur(0) is:";heuristics.back()->print_heur_name();cout<<endl;fflush(stdout);
		  cout<<"Only one strong heuristic left and no complementaries, so chosen_heuristic:selected_heur(0) is:";heuristics.back()->print_heur_name();cout<<endl;
		    for (int j=0;j<orig_heuristics.size();j++){
		      orig_heuristics[j]->restore_backed_up_landmarks();
		      orig_heuristics[j]->set_keep_frontier(false);
		  }
		    delete nodes;nodes = new HashTable;//removed all visited HUST states from memory
		    reset_HST(); 
		    cout<<"F_bound:"<<F_bound<<",Credit Assignment sampling Time:"<<credit_sampling_timer.stop()<<endl;fflush(stdout);
		  return;
	     }
	   }
	   if(demoted_heur){
	     cout<<"Heuristics have been demoted, need to restart sampling again and adjust F-boundary accordingly"<<endl;
	     break;
	   }
	    //cout<<"finished checking for runaway heur"<<endl;fflush(stdout);
	  }
	   if((current_sampling_timer()>(sampling_time_limit*2))&&(!full_sampling)){//get out if sampling this root node is simply taking too long
	     cout<<"Finishing sampling root:"<<i<<" early at HoF root:"<<last_sampled_HoF_Root<<",current_sampling_time:"<<current_sampling_timer.stop()<<",f_boundary_time:"<<F_boundary_time<<endl;
	      overall_nodes_culled+=nodes_culled_iter;
	      overall_nodes_gen+=nodes_gen_iter;
	     break;
	   }
       }
		      
       if(to_be_expanded()){//need to generate next child
		  next_op=get_next_op();//pops back last operator from list
		  //cout<<"with operator:";fflush(stdout);next_op->dump();cout<<endl;fflush(stdout);
		  //get next state
		  get_current_state(current_state);
		  //cout<<"current_state:";fflush(stdout);current_state->inline_dump();fflush(stdout);cout<<endl;
		  //if(strong_heur.size()==1){
		    //cout<<"selected state:";current_state->dump();cout<<"at depth:"<<get_last_depth()<<endl;fflush(stdout);
		  //cout<<"at depth:"<<get_last_depth()<<endl;fflush(stdout);
       //}
		  succ_state=new State(*current_state, *next_op);
		  //cout<<"succ_state:";fflush(stdout);current_state->inline_dump();fflush(stdout);cout<<endl;
		  nodes_gen_iter++;//node is generated even if duplicate check rejected it!
		  //SearchNode succ_node = search_space->get_node(chosen_Hoff_Roots->at(i).first);
		  //depth=get_last_depth()+next_op->get_cost();//move to after check_duplicate if updating counter is not necessary
		  if(search_space->check_duplicate(succ_state)){
		    //calculate_h_index_only(succ_state,depth,heuristics,h_index);
		    //substract_from_counter(&h_index,1);
		      nodes_culled_iter++;
		      //cout<<"Substracting 1 from CC:"<<h_index<<endl;
		     //cout<<"Succ state is old"<<endl;
		     continue;
		 }
		  /*  else{
		     cout<<"Succ state is new"<<endl;fflush(stdout);
		  }*/

		  if(global_duplicate_check){
		    if(check_duplicate(succ_state,next_op,heuristics)){
		      //calculate_h_index_only(succ_state,depth,heuristics,h_index);
		      //substract_from_counter(&h_index,1);
		      //cout<<"Substracting 1 from CC:"<<h_index<<endl;
		      nodes_culled_iter++;
		      //cout<<"Succ state is old in sampling stage"<<endl;
		      continue;//going to next state, this one is duplicated
		    }
		    /*  else{
		       cout<<"Succ state is new in sampling stage"<<endl;fflush(stdout);
		    }*/
		  }
		  depth=get_last_depth()+next_op->get_cost();
		}
		else{//node must be fully expanded so we backtrack until no more nodes can be expanded or HST is empty
		  //cout<<"backtracking"<<endl;fflush(stdout);
		  while((!to_be_expanded())){
		    get_current_state(current_state);
		    for(int j=0;j<heuristics.size();j++){
		      heuristics[j]->finished_state(*current_state, -1, false);
		    }
		    backtrack();
		  //cout<<"backtracking finished"<<endl;fflush(stdout);
		    if(empty()){
		      //cout<<"Finished with Root:"<<i<<endl;fflush(stdout);
		      //cout<<",States gen:"<<nodes_gen_iter<<endl;
		      //cout<<"\t States generated:"<<nodes_gen_iter<<",overall_nodes_gen:"<<nodes_gen_iter+overall_nodes_gen<<endl;
		      //lsearch_space.print_counters();
		      //cout<<"finished with root node:";S->dump();fflush(stdout);cout<<endl;
		      //cout<<"HBF[,"<<selected_HIST_leaves_counter<<",]:"<<nodes_culled_iter<<endl;
		      overall_nodes_culled+=nodes_culled_iter;
		      overall_nodes_gen+=nodes_gen_iter;
		      break;
		    }
		  }
		  //cout<<"backtracked to ancestor node:"<<lsearch_space.get_lspace_end()<<endl;fflush(stdout);
		  if(empty()){
		    break;
		  }
		  continue;
		}
		//Now we evaluate the current state to decide if we add it to the tree
    
		//h_max=evaluate_min(succ_state);
		//h_max=evaluate_MAX_node(succ_state,depth);
		applicable_ops.clear();
		//cout<<"generating applicable ops:"<<i<<endl;fflush(stdout);
		g_successor_generator->generate_applicable_ops(*succ_state, applicable_ops);
		h_min=evaluate_HUST_node(succ_state,depth,applicable_ops.size(),heuristics);
		//if(strong_heur.size()==1){
		//  cout<<",h_min:"<<h_min<<",depth:"<<depth<<",F:"<<h_min+depth<<",current F_bound:"<<get_current_F_bound();
		//}
		if((h_min+depth)<=get_current_F_bound()){//expand nodes not culled by any of the input heuristics
		//if(strong_heur.size()==1){
		//  cout<<",node added"<<endl;
		//}
		  add_to_HST(succ_state,next_op,h_min,depth);
		}
		else{//need to to list of culled nodes even if they were not generated
		//if(strong_heur.size()==1){
		//  cout<<",node culled"<<endl;
	        //}
		  nodes_culled_iter++;
		  delete succ_state;//state will not be added so remove it from memory
		}
		continue;
	      }
     //cout<<"Finished with current Root"<<endl;fflush(stdout);
	 if(memory_limit<get_peak_memory_in_kb()){
	   cout<<"Exiting because of high memory usage at root node:"<<i<<endl;
	   break;
	 }
   }
   cout<<"F_bound:"<<get_current_F_bound()<<",overall_sampled_gen_nodes:,"<<overall_nodes_gen<<",overall_nodes_culled:,"<<overall_nodes_culled<<"last sampled HoF root:"<<last_sampled_HoF_Root<<endl;
   
   credit_sampling_timer.reset();credit_sampling_timer.resume();
 //print_counters();
 //heuristics=orig_heuristics;
    
   int remaining_heurs=0;  
   for(int j=0;j<orig_heuristics.size();j++){
     if(orig_heuristics.at(j)->is_using()){
       remaining_heurs++;
       cout<<"h("<<j<<"),"<<orig_heuristics.at(j)->get_heur_name()<<" is final remaining_heur"<<endl;
     }
   }
   cout<<"Max Degree allowed:"<<Degree<<",heuristics remaining:"<<remaining_heurs<<endl;
  
   set_current_F_bound(current_f);//We need to update this of reporting the correct F-boundary we have sampled the HBF
   calculate_heuristics_to_degree(min(Degree,unsigned(remaining_heurs)),orig_heuristics);
   cout<<"finished calculate_heuristics_to_degree"<<endl;
   to_string(get_best_h_comb(),chosen_heurs);
   cout<<"chosen heur:"<<chosen_heurs<<endl;fflush(stdout);

   //Now populate the chosen heuristics
   cout<<"F_bound:"<<get_current_F_bound()<<",chosen_heuristic:";
   heuristics.clear();
   bool incremental_lmcut_present=false;
   int j=0;
   for(int i=0;i<orig_heuristics.size();i++){
     if(best_h_comb.test(i)){
       j++;
        if(orig_heuristics[i]->get_heur_name()=="incremental_lmcut"){
	cout<<"Restoring landmarks and removing backup, now landmarks should be ok"<<endl;
	orig_heuristics[i]->restore_backed_up_landmarks();
	orig_heuristics[i]->set_keep_frontier(false);
	incremental_lmcut_present=true;
      }
       heuristics.push_back(orig_heuristics.at(i));
       cout<<"selected_heur("<<j<<") is:";heuristics.back()->print_heur_name();cout<<endl;
       //cout<<"orig_heuristics at "<<i<<" pushed into heuristics"<<endl;
     }
     else{
       orig_heuristics[i]->free_up_memory();
     }
   }
   if(!incremental_lmcut_present){
     cout<<"incremental_lmcut not present so erasing all landmarks"<<endl;
     for(int i=0;i<orig_heuristics.size();i++){
      if(orig_heuristics[i]->get_heur_name()=="incremental_lmcut"){
	  orig_heuristics[i]->erase_all_landmarks();
      }
     }
   }
   cout<<"heuristics,size:"<<heuristics.size()<<",creating selected_heur_call"<<endl;
   create_selected_heur_call(heuristics);
   annotate_sampling_data(orig_heuristics,heuristics);
   //cout<<"Exiting now because we want to test performance of selected heuristics from the beginning"<<endl;
   //exit(2);//special return code so we now we want to run from the start with the selected heuristics
   //cout<<"only want to know which heuristics were selected, so exiting"<<endl;
   //exit(0);
   reset_HST(); 
   delete nodes;nodes = new HashTable;//removed all visited HUST states from memory
   cout<<"F_bound:"<<F_bound<<",Credit Assignment sampling Time:"<<credit_sampling_timer.stop()<<endl;

  cout<<"finished calling select_best_estimated_heuristic_subset"<<endl;
}
//we need this function to eliminate from counter those nodes who were never evaluated because of dup_check
void HST::calculate_h_index_only(const State* S, int depth, vector<Heuristic *> heuristics, boost::dynamic_bitset<> &h_index){
  int H;
  h_index.resize(heuristics.size());h_index.reset();
  for (size_t i = 0; i < heuristics.size(); i++){
    if(h_capped.at(i)||get_earliest_depth_h_culled(i)<depth){//path already culled for database
      h_index.set(i);
    }
    else{
      heuristics[i]->evaluate(*S);
      H=heuristics[i]->get_heuristic();
    //Now update CC index with either culling (1) or generating (0) bit
      if((H+depth)>get_current_F_bound()){//node culled is marked as 1 in binary switch list
	h_index.set(i);
      }
    }
  }
}

//We need this function on this class (moved from HustSearch) when doing A* sampling from eagersearch.cc
//better solution would be to modify HustSearch, future work
int HST::evaluate_HUST_node(const State* S,int depth,int children,vector<Heuristic *> heuristics){
  int h_min=INT_MAX/2;
  int h_strong_min=INT_MAX/2;
  int H=0;
  boost::dynamic_bitset<> h_index(heuristics.size());
  boost::dynamic_bitset<> h_index_substract(heuristics.size());h_index_substract.set();
  //static bool first_reported=false;
  vector<bool> first_culling_added;//needed temporary marker because no need to update culling trackers if all heuristics cull current node

  //static long LMCUT_counter=0;

  //cout<<"calling HST::evaluate_HUST_node"<<endl;fflush(stdout);
  //cout<<"Current_F_bound:"<<get_current_F_bound()<<endl;
  bool at_least_one_strong_heur=false;
  //cout<<"h_capped:"<<h_capped.count()<<endl;
  for (size_t i = 0; i < heuristics.size(); i++){
    //cout<<"\ti:"<<i<<endl;fflush(stdout);cout<<heuristics[i]->get_heur_name()<<endl;fflush(stdout);
    if(h_capped.at(i)||get_earliest_depth_h_culled(i)<depth){//path already culled for database
      /*cout<<"\tskipping heur eval of"<<heuristics[i]->get_heur_name();;fflush(stdout);
      if(h_capped.at(i)){
	cout<<",heuristic is capped"<<endl; 
      }
      else{
	cout<<",earlieast_depth_h_culled:"<<get_earliest_depth_h_culled(i)<<".depth:"<<depth<<endl;
      }*/
      H=INT_MAX/2;//INT_MAX on its own leads to bug as we add depth to it, resulting in the counter restarting
      heuristics[i]->finished_state(*S, H+depth, true);
      /*  if(heuristics[i]->is_dead_end()){
	heuristics[i]->finished_state(*S, -1, false);
      }
      else{
	heuristics[i]->finished_state(*S, -1, true);
      }*/
      /*if(h_capped.at(i)){
	if(!first_reported)
	if(lsearch_space.get_current_F_bound()>30){
	  cout<<"h("<<i<<") is capped, so H="<<H<<endl;
	  first_reported=true;
	}
      }*/
      //cout<<"\tH["<<i<<"]:"<<H<<",depth:"<<depth<<",F:"<<depth+H<<endl;fflush(stdout);
      //
      /*if(i==heuristics.size()-1){
	if(h_capped.at(i)){
	    cout<<"H("<<i<<") is capped, so H="<<H<<endl;
	}
	else{
	    cout<<"H("<<i<<") was culled @ depth:"<<get_earliest_depth_h_culled(i)<<endl;
	}
      }*/
    }
    else{
      //cout<<"\tevaluating heuristic"<<endl;fflush(stdout);
      //cout<<"\t evaluating"<<heuristics[i]->get_heur_name();fflush(stdout);
      /*if(heuristics[i]->get_time_cost()>0.000001){//LMCUT RELATIVE POSITION
	LMCUT_counter++;
	if(LMCUT_counter%500==0){
	  cout<<"LMCUT_counter:"<<LMCUT_counter<<endl;
	}
      }*/
      //if(get_current_F_bound()>26){exit(0);};
      //cout<<"calling evaluate"<<endl;fflush(stdout);
      heuristics[i]->evaluate(*S);
      //cout<<"calling get_heuristic"<<endl;fflush(stdout);
      if(heuristics[i]->is_dead_end()){
	//cout<<",heuristic is dead_end"<<endl;
	H=INT_MAX/2;
      }
      else{
	H=heuristics[i]->get_heuristic();
      }
     // HACK try to maintain only met information for boundary nodes
     // only useful for incremental lmcut atm
    // HACK: should call this whenever memory is about to run out
	//Heuristic *h = heuristics[j];
	//heuristics[i]->free_up_memory(search_space);
      //cout<<"\tH["<<i<<"]:"<<H<<",depth:"<<depth<<",F:"<<depth+H<<endl;fflush(stdout);
    }
    //cout<<"\tH["<<i<<"]:"<<H<<",depth:"<<depth<<",F_bound:"<<F_bound<<endl;fflush(stdout);
    //Now update CC index with either culling (1) or generating (0) bit
    if((H+depth)>get_current_F_bound()){//node culled is marked as 1 in binary switch list
      //cout<<"culling heur"<<endl;fflush(stdout);
      //heuristics[i]->finished_state(*S, H+depth, true);
      heuristics[i]->finished_state(*S, H+depth, true);
      if(Current_RIDA_Phase==SOLVING_PHASE
	  &&(strong_heur.find(i)!=strong_heur.end())){//in case we get here because of chosen_heurs being populated by the user
	h_strong_min=min(h_strong_min,H);
      }
      //cout<<"\t culling heuristic, adding one to h_index"<<endl;
      h_min=min(h_min,H);
      h_index.set(i);
      if(checking_consistency()){
	if(get_earliest_depth_h_culled(i)==INT_MAX/2){//first culled in path
	  //cout<<"\th("<<i<<"),first culling at:"<<depth<<endl;
	  first_culling_added.push_back(true);
	 //h_culling_index+=pow(3,database);//value is one for first culling
	}
	else{
	  first_culling_added.push_back(false);
	  //h_culling_index+=pow(3,database)*2;//value is two for previous culling
	}
      }
    }
    else if(checking_consistency()){//h_min is not updated if path would have been culled by MAXTREE
      if((!at_least_one_strong_heur)&&Current_RIDA_Phase==SOLVING_PHASE
	  &&(strong_heur.find(i)!=strong_heur.end())){//in case we get here because of chosen_heurs being populated by the user
	at_least_one_strong_heur=true;
	//cout<<"at_least_one_strong_heur is true"<<endl;fflush(stdout);
      }
      /*  else{
	cout<<"at least one_strong_heur is false"<<endl;fflush(stdout);
      }*/
      first_culling_added.push_back(false);
      if(get_earliest_depth_h_culled(i)<depth){//h_min is not updated as path would have been culled by MAXTREE
	//cout<<"\th_index["<<i<<"] set because earliest depth culled:"<<get_earliest_depth_h_culled(i)<<endl;
	h_index.set(i);
      }
      else{//h_min is updated as there is no inconsistency
	if(h_capped.at(i)){
	  cout<<"If the heuristic is capped, it must cull the node, check the code!"<<endl;fflush(stdout);exit(0);
	}
	  h_min=min(h_min,H);
	  if((H+depth)<get_current_F_bound()){//This node belongs to a previous F-boundary so do not add for comparison purposes, we are only interested on the ratio for the last frontier
	    //cout<<"substracting heur"<<endl;fflush(stdout);
	   //substract+=children;
	   h_index_substract.reset(i);
	   //cout<<"\tReset h_index["<<i<<"] to true"<<endl;
	  }
	  //cout<<"\th_min[:"<<i<<"]:"<<h_min<<endl;fflush(stdout);
      }
    }
    else{//update h_min
      if((!at_least_one_strong_heur)&&Current_RIDA_Phase==SOLVING_PHASE
	  &&(strong_heur.find(i)!=strong_heur.end())){//in case we get here because of chosen_heurs being populated by the user
	at_least_one_strong_heur=true;
	//cout<<"at_least_one_strong_heur is true"<<endl;fflush(stdout);
      }
      h_min=min(h_min,H);
      if((H+depth)<get_current_F_bound()){//This node belongs to a previous F-boundary so do not add for comparison purposes, we are only interested on the ratio for the last frontier
	//cout<<"substracting heur"<<endl;fflush(stdout);
       //substract+=children;
       h_index_substract.reset(i);
       //cout<<"\tReset h_index["<<i<<"] to true"<<endl;
      }
    }
  }
	  
  //If no strong_heur expanding this node when doing revaluation, do not add this node to any counter and cull node by returning int_max
  if((!at_least_one_strong_heur)&&Current_RIDA_Phase==SOLVING_PHASE){//no strong heur expanding node, so we cull it
    //if(strong_heur.size()==1){
    //  cout<<"returning INT_MAX/2 as no strong_heur is expanding the node"<<endl;
    //}
    //cout<<"returning h_strong_min not expanding node"<<h_strong_min<<endl;fflush(stdout);
    return h_strong_min;
  }



  if(children==0){//no CC counter to update as node has no children
    //cout<<"\tNo CC to update as children=0"<<endl;fflush(stdout);
    return h_min;
  }

  //Only update CCs if at least one heuristic is expanding, this way we reduce CC updates.  We add all children nodes for any "internal" CC
  
  if(h_index.count()!=heuristics.size()){//all heuristics culling so no child generated, so nothing to add to counters
     //cout<<"\tF_bound:"<<get_current_F_bound()<<",h_min:"<<h_min<<"heuristic combination:"<<h_index<<",depth:"<<depth<<"adding to counter nodes:"<<children<<endl;fflush(stdout);
      add_to_counter(&h_index,children);
     //cout<<"added "<<children<<" to counter,"<<h_index<<",";S->inline_dump();cout<<endl;
      if(h_index_substract.count()!=heuristics.size()){//So at least one heursitic was not at the correct F-boundary
	//cout<<"adding h_index_substract:"<<h_index_substract<<",childen:"<<children<<endl;fflush(stdout);
	      /*if(h_index.count()==0){
		cout<<"evaluate_HUST-All heuristics F-value bellow F-boundary!!"<<endl;
	      }*/
	add_to_substract_counter(&h_index_substract,children);
      }
      //cout<<"\tmin_f:"<<gcmd_line.culling<<"h_min:"<<h_min<<",heuristic combination:"<<h_index<<",depth:"<<depth<<",children nodes added to counter:"<<gnum_A-1<<endl;
  //If checking consistency, update relevant culling depth trackers

    if(checking_consistency()){
      for (int i=0;i<first_culling_added.size(); i++ ){
	if(first_culling_added.at(i)){//heuristic culled for first time
	  //cout<<"heuristic "<<i<<" culled at depth"<<depth<<endl;fflush(stdout);
	  set_earliest_depth_h_culled(i,depth);
	}
      }
    }
    //cout<<"h_min:"<<h_min<<",depth:"<<depth<<",F:"<<h_min+depth<<",F_bound:"<<lsearch_space.get_current_F_bound()<<",added children:"<<children<<endl;
  }
  else{
      //cout<<"\tF_bound:"<<get_current_F_bound()<<",h_min:"<<h_min<<"heuristic combination:"<<h_index<<",depth:"<<depth<<"all heuristics culling so not adding"<<endl;fflush(stdout);
    //cout<<"h_min:"<<h_min<<",depth:"<<depth<<",F:"<<h_min+depth<<",F_bound:"<<lsearch_space.get_current_F_bound()<<endl;
  }
  //cout<<"final h_min for current node:"<<h_min<<endl;fflush(stdout);

  return h_min;
}
void HST::compare_MAX_heuristics(const State* S,vector<int> &winning_h,vector<Heuristic *> heuristics, double avg_TPN){
  int h_max_min_tpn=0;
  vector<int> h_vals;
  winning_h.clear();
  //cout<<"calling compare_MAX_heuristics"<<endl;fflush(stdout);
  //First calculate the avg_TPN, heuristics can also be strong if their TPN is not above 10 times the maxium TPN
  
 // cout<<"\t";
  for (size_t i = 0; i < heuristics.size(); i++){
    //cout<<heuristics[i]->get_heur_name()<<",before evaluate["<<i<<endl;fflush(stdout);
      heuristics[i]->evaluate(*S);
      //cout<<"compare_max,\t"<<heuristics[i]->get_heur_name()<<",["<<i<<"],h:"<<heuristics[i]->get_value()<<endl;
    //cout<<"after evaluate["<<i<<endl;fflush(stdout);
      //If dealing with a group of very expensive heuristics, then allowing cheaper heuristics to become strong heuristics as well
      //heuristics[i]->finished_state(*S, depth + heuristics[i]->get_value() , true);
      h_vals.push_back(heuristics[i]->get_value());
      heuristics[i]->finished_state(*S, -1, false);
      //h_vals.push_back(heuristics[i]->get_heuristic());
      if(heuristics[i]->get_measured_TPN()<10*avg_TPN){
	h_max_min_tpn=max(h_max_min_tpn,h_vals.back());
      }
      //if(heuristics[i]->is_dead_end()){
	//cout<<"h["<<i<<"] is a dead_end"<<endl;fflush(stdout);
	//`heuristics[i]->finished_state(*S, -1, false);
      //}
      //cout<<"\th_vals("<<i<<"):"<<h_vals.back()<<",";fflush(stdout);
  }
  //cout<<"h_max_min_tpn:"<<h_max_min_tpn<<endl;fflush(stdout);
  /*if(h_max_min_tpn>10000000){//dead end or something weird!
    for (size_t i = 0; i < heuristics.size(); i++){
      cout<<"h("<<i<<"):"<<heuristics[i]->get_heuristic()<<",tpn:"<<heuristics[i]->get_time_cost()<<endl;
    }
    cout<<",h_max:"<<h_max_min_tpn<<",exit for debugging"<<endl;
    exit(0);
  }*/
  for (size_t i = 0; i < h_vals.size(); i++){
    if(h_vals.at(i)>=h_max_min_tpn){
      winning_h.push_back(i);
      //cout<<"\tadded winning h("<<i<<")"<<endl;fflush(stdout);
    }
  }
}
