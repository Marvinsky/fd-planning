
/*********************************************************************
 * (C) Copyright 1999 Albert Ludwigs University Freiburg
 *     Institute of Computer Science
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 *********************************************************************/








/*********************************************************************
 * File: relax.h
 * Description: headers for relaxed planning
 *
 * Author: Joerg Hoffmann 1999
 *
 *********************************************************************/ 






#ifndef _RELAX_H
#define _RELAX_H


bool LESS( int a, int b );
bool LEQ( int a, int b );
bool GEQ( int a, int b );



int get_1P_and_AH( State S );
int get_1P( State S );



int build_fixpoint( State S, bool A_info );
int build_fixpoint_heuristic( State S, bool A_info, int calls );
void build_fixpoint_IDA( State S );
void build_fixpoint_BFS(const State* S );
void initialize_fixpoint( State S );
void initialize_fixpoint_BFS(const State* S );
void activate_ft( int index, int time );
void activate_op( int index, int time );
void new_fact( int index );
void new_op( int index );
void reset_fixpoint( void );
bool all_goals_activated( int time ); 
void collect_A_info( void );
void collect_A_info_heuristic( int calls );
void print_fixpoint_result( void );



int extract_1P( int max, bool H_info );
int initialize_goals( int max );
void achieve_goals( int time );
void collect_H_info( void );
void reset_search_info( void );


bool copy_1P_and_AH(void);
void copy_1P_and_AH_recursive(void);
void restore_1P_and_AH_recursive(void);
bool restore_1P_and_AH(void);
bool reset_graph_heuristic(void);
int get_1P_and_AH_heuristic( State S, int calls );

#endif /* _RELAX_H */





