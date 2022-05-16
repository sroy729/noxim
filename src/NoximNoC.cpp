/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the Network-on-Chip
 */
#include "NoximNoC.h"
#include "NoximGlobalStats.h"
#include <string>
#include <iomanip>
int throttling[10][10][4];
int beltway[10][10][4];
int reShot[10][10][4]; //Jimmy added on 2017.05.29
double weight[64][19];	
double Q_table[10][10][5][3] = {0};
double R[10][10];
double D[10][10][10000][23] = {0}; // 0-5 S_t 6-8 Q_t 9-14 S_t+1 15-17 Q_t+1 18 max_Q_t 19 max_Q_t+1 20 r_t 21 a_t 22 cycle
int D_MAX = 10000;
int D_col = 23;
double tar_L1_w[10][10][7][3];
double eva_L1_w[10][10][7][3];
float base_cycle=-2;
double init_temp[256]={0};

extern ofstream transient_log_throughput;

void NoximNoC::buildMesh()
{
    // Check for routing table availability
    if (NoximGlobalParams::routing_algorithm == ROUTING_TABLE_BASED)
		assert(grtable.load(NoximGlobalParams::routing_table_filename));

    // Check for traffic table availability
    if (NoximGlobalParams::traffic_distribution == TRAFFIC_TABLE_BASED)
		assert(gttable.load(NoximGlobalParams::traffic_table_filename));
 
 	// Check for sensor table availability (Jimmy added on 2016.03.19)
    if (NoximGlobalParams::sensor_allocate == SENSOR_ALLOCATE_TABLE)
		assert(gstable.load(NoximGlobalParams::sensor_table_filename));

    //assert(gltable.load(NoximGlobalParams::rector_linear_coef_filename));

    // Create the mesh as a matrix of tiles
	int i,j,k;
	char _name[20];
	for ( i = 0; i < NoximGlobalParams::mesh_dim_x; i++) 
		for ( j = 0; j < NoximGlobalParams::mesh_dim_y; j++){
			sprintf( _name, "VLink[%02d][%02d]", i, j);
			v[i][j] = new NoximVLink( _name );
			v[i][j]->clock(clock);
			v[i][j]->reset(reset);
			v[i][j]->setId( i + NoximGlobalParams::mesh_dim_x*j );
			for ( k = 0; k < NoximGlobalParams::mesh_dim_z; k++){
				// Create the single Tile with a proper name
				sprintf(_name, "Tile[%02d][%02d][%02d]", i, j, k);
				t[i][j][k] = new NoximTile(_name);
				// Tell to the router its coordinates
				// cout<<"Set configuration para. of router"<<endl;
				
				t[i][j][k]->r->configure( xyz2Id( i , j , k ), NoximGlobalParams::stats_warm_up_time,
						NoximGlobalParams::buffer_depth, grtable);
				
				// Tell to the PE its coordinates
				t[i][j][k]->pe->local_id       = xyz2Id( i , j , k );
				t[i][j][k]->pe->traffic_table  = &gttable;	// Needed to choose destination
				t[i][j][k]->pe->never_transmit = (gttable.occurrencesAsSource(t[i][j][k]->pe->local_id) == 0);
				
				// Map clock and reset
				t[i][j][k]->clock(clock);
				t[i][j][k]->reset(reset);
				
				// Map Rx signals                                                
				t[i][j][k]->req_rx             [DIRECTION_NORTH] (req_to_south       [i  ][j  ][k  ]);
				t[i][j][k]->flit_rx            [DIRECTION_NORTH] (flit_to_south      [i  ][j  ][k  ]);
				t[i][j][k]->ack_rx             [DIRECTION_NORTH] (ack_to_north       [i  ][j  ][k  ]);
																								
				t[i][j][k]->req_rx             [DIRECTION_EAST ] (req_to_west        [i+1][j  ][k  ]);
				t[i][j][k]->flit_rx            [DIRECTION_EAST ] (flit_to_west       [i+1][j  ][k  ]);
				t[i][j][k]->ack_rx             [DIRECTION_EAST ] (ack_to_east        [i+1][j  ][k  ]);
																								
				t[i][j][k]->req_rx             [DIRECTION_SOUTH] (req_to_north       [i  ][j+1][k  ]);
				t[i][j][k]->flit_rx            [DIRECTION_SOUTH] (flit_to_north      [i  ][j+1][k  ]);
				t[i][j][k]->ack_rx             [DIRECTION_SOUTH] (ack_to_south       [i  ][j+1][k  ]);
																					
				t[i][j][k]->req_rx             [DIRECTION_WEST ] (req_to_east        [i  ][j  ][k  ]);
				t[i][j][k]->flit_rx            [DIRECTION_WEST ] (flit_to_east       [i  ][j  ][k  ]);
				t[i][j][k]->ack_rx             [DIRECTION_WEST ] (ack_to_west        [i  ][j  ][k  ]);
				
				//To VLink																	
				t[i][j][k]->req_rx             [DIRECTION_UP   ] (req_toT_down       [i  ][j  ][k  ]);
				t[i][j][k]->flit_rx            [DIRECTION_UP   ] (flit_toT_down      [i  ][j  ][k  ]);
				t[i][j][k]->ack_rx             [DIRECTION_UP   ] (ack_toV_up         [i  ][j  ][k  ]);
				//To VLink																			
				t[i][j][k]->req_rx             [DIRECTION_DOWN ] (req_toT_up         [i  ][j  ][k  ]);
				t[i][j][k]->flit_rx            [DIRECTION_DOWN ] (flit_toT_up        [i  ][j  ][k  ]);
				t[i][j][k]->ack_rx             [DIRECTION_DOWN ] (ack_toV_down       [i  ][j  ][k  ]);
				
				// Map Tx signals                                                    
				t[i][j][k]->req_tx             [DIRECTION_NORTH] (req_to_north       [i  ][j  ][k  ]);
				t[i][j][k]->flit_tx            [DIRECTION_NORTH] (flit_to_north      [i  ][j  ][k  ]);
				t[i][j][k]->ack_tx             [DIRECTION_NORTH] (ack_to_south       [i  ][j  ][k  ]);
																					
				t[i][j][k]->req_tx             [DIRECTION_EAST ] (req_to_east        [i+1][j  ][k  ]);
				t[i][j][k]->flit_tx            [DIRECTION_EAST ] (flit_to_east       [i+1][j  ][k  ]);
				t[i][j][k]->ack_tx             [DIRECTION_EAST ] (ack_to_west        [i+1][j  ][k  ]);
																					
				t[i][j][k]->req_tx             [DIRECTION_SOUTH] (req_to_south       [i  ][j+1][k  ]);
				t[i][j][k]->flit_tx            [DIRECTION_SOUTH] (flit_to_south      [i  ][j+1][k  ]);
				t[i][j][k]->ack_tx             [DIRECTION_SOUTH] (ack_to_north       [i  ][j+1][k  ]);
																					
				t[i][j][k]->req_tx             [DIRECTION_WEST ] (req_to_west        [i  ][j  ][k  ]);
				t[i][j][k]->flit_tx            [DIRECTION_WEST ] (flit_to_west       [i  ][j  ][k  ]);
				t[i][j][k]->ack_tx             [DIRECTION_WEST ] (ack_to_east        [i  ][j  ][k  ]);
				
				//To VLink																	
				t[i][j][k]->req_tx             [DIRECTION_UP   ] (req_toV_up         [i  ][j  ][k  ]);
				t[i][j][k]->flit_tx            [DIRECTION_UP   ] (flit_toV_up        [i  ][j  ][k  ]);
				t[i][j][k]->ack_tx             [DIRECTION_UP   ] (ack_toT_down       [i  ][j  ][k  ]);
				//To VLink														     
				t[i][j][k]->req_tx             [DIRECTION_DOWN ] (req_toV_down       [i  ][j  ][k  ]);
				t[i][j][k]->flit_tx            [DIRECTION_DOWN ] (flit_toV_down      [i  ][j  ][k  ]);
				t[i][j][k]->ack_tx             [DIRECTION_DOWN ] (ack_toT_up         [i  ][j  ][k  ]);
				
				// Map buffer level signals (analogy with req_tx/rx port mapping)
				t[i][j][k]->free_slots         [DIRECTION_NORTH] (free_slots_to_north[i  ][j  ][k  ]);
				t[i][j][k]->free_slots         [DIRECTION_EAST ] (free_slots_to_east [i+1][j  ][k  ]);
				t[i][j][k]->free_slots         [DIRECTION_SOUTH] (free_slots_to_south[i  ][j+1][k  ]);
				t[i][j][k]->free_slots         [DIRECTION_WEST ] (free_slots_to_west [i  ][j  ][k  ]);
				t[i][j][k]->free_slots         [DIRECTION_UP   ] (free_slots_to_up   [i  ][j  ][k  ]);
				t[i][j][k]->free_slots         [DIRECTION_DOWN ] (free_slots_to_down [i  ][j  ][k+1]);
				
				t[i][j][k]->free_slots_neighbor[DIRECTION_NORTH] (free_slots_to_south[i  ][j  ][k  ]);
				t[i][j][k]->free_slots_neighbor[DIRECTION_EAST ] (free_slots_to_west [i+1][j  ][k  ]);
				t[i][j][k]->free_slots_neighbor[DIRECTION_SOUTH] (free_slots_to_north[i  ][j+1][k  ]);
				t[i][j][k]->free_slots_neighbor[DIRECTION_WEST ] (free_slots_to_east [i  ][j  ][k  ]);
				t[i][j][k]->free_slots_neighbor[DIRECTION_UP   ] (free_slots_to_down [i  ][j  ][k  ]);
				t[i][j][k]->free_slots_neighbor[DIRECTION_DOWN ] (free_slots_to_up   [i  ][j  ][k+1]);
				// NoP 
				t[i][j][k]->NoP_data_out       [DIRECTION_NORTH] (NoP_data_to_north  [i  ][j  ][k  ]);
				t[i][j][k]->NoP_data_out       [DIRECTION_EAST ] (NoP_data_to_east   [i+1][j  ][k  ]);
				t[i][j][k]->NoP_data_out       [DIRECTION_SOUTH] (NoP_data_to_south  [i  ][j+1][k  ]);
				t[i][j][k]->NoP_data_out       [DIRECTION_WEST ] (NoP_data_to_west   [i  ][j  ][k  ]);
				t[i][j][k]->NoP_data_out       [DIRECTION_UP   ] (NoP_data_to_up     [i  ][j  ][k  ]);
				t[i][j][k]->NoP_data_out       [DIRECTION_DOWN ] (NoP_data_to_down   [i  ][j  ][k+1]);
																					
				t[i][j][k]->NoP_data_in        [DIRECTION_NORTH] (NoP_data_to_south  [i  ][j  ][k  ]);
				t[i][j][k]->NoP_data_in        [DIRECTION_EAST ] (NoP_data_to_west   [i+1][j  ][k  ]);
				t[i][j][k]->NoP_data_in        [DIRECTION_SOUTH] (NoP_data_to_north  [i  ][j+1][k  ]);
				t[i][j][k]->NoP_data_in        [DIRECTION_WEST ] (NoP_data_to_east   [i  ][j  ][k  ]);
				t[i][j][k]->NoP_data_in        [DIRECTION_UP   ] (NoP_data_to_down   [i  ][j  ][k  ]);
				t[i][j][k]->NoP_data_in        [DIRECTION_DOWN ] (NoP_data_to_up     [i  ][j  ][k+1]);
				
				/*** RCA port connection (Jimmy added on 2012.06.27) ***/
				t[i][j][k]->RCA_data_out[DIRECTION_NORTH*2+0](RCA_data_to_north0[i][j][k]);
				t[i][j][k]->RCA_data_out[DIRECTION_NORTH*2+1](RCA_data_to_north1[i][j][k]);
				t[i][j][k]->RCA_data_out[DIRECTION_EAST*2+0](RCA_data_to_east0[i+1][j][k]);
				t[i][j][k]->RCA_data_out[DIRECTION_EAST*2+1](RCA_data_to_east1[i+1][j][k]);
				t[i][j][k]->RCA_data_out[DIRECTION_SOUTH*2+0](RCA_data_to_south0[i][j+1][k]);
				t[i][j][k]->RCA_data_out[DIRECTION_SOUTH*2+1](RCA_data_to_south1[i][j+1][k]);
				t[i][j][k]->RCA_data_out[DIRECTION_WEST*2+0](RCA_data_to_west0[i][j][k]);
				t[i][j][k]->RCA_data_out[DIRECTION_WEST*2+1](RCA_data_to_west1[i][j][k]);
				
				t[i][j][k]->RCA_data_in[DIRECTION_NORTH*2+0](RCA_data_to_south1[i][j][k]);    //***0 1 inverse
				t[i][j][k]->RCA_data_in[DIRECTION_NORTH*2+1](RCA_data_to_south0[i][j][k]);    //***0 1 inverse
				t[i][j][k]->RCA_data_in[DIRECTION_EAST*2+0](RCA_data_to_west1[i+1][j][k]);    //***0 1 inverse
				t[i][j][k]->RCA_data_in[DIRECTION_EAST*2+1](RCA_data_to_west0[i+1][j][k]);    //***0 1 inverse
				t[i][j][k]->RCA_data_in[DIRECTION_SOUTH*2+0](RCA_data_to_north1[i][j+1][k]);    //***0 1 inverse
				t[i][j][k]->RCA_data_in[DIRECTION_SOUTH*2+1](RCA_data_to_north0[i][j+1][k]);    //***0 1 inverse
				t[i][j][k]->RCA_data_in[DIRECTION_WEST*2+0](RCA_data_to_east1[i][j][k]);    //***0 1 inverse
				t[i][j][k]->RCA_data_in[DIRECTION_WEST*2+1](RCA_data_to_east0[i][j][k]);    //***0 1 inverse		
				/*******************************************************/
				
				t[i][j][k]->monitor_out        [DIRECTION_NORTH] (RCA_to_north       [i  ][j  ][k  ]);
				t[i][j][k]->monitor_out        [DIRECTION_EAST ] (RCA_to_east        [i+1][j  ][k  ]);
				t[i][j][k]->monitor_out        [DIRECTION_SOUTH] (RCA_to_south       [i  ][j+1][k  ]);
				t[i][j][k]->monitor_out        [DIRECTION_WEST ] (RCA_to_west        [i  ][j  ][k  ]);
				t[i][j][k]->monitor_out        [DIRECTION_UP   ] (RCA_to_up          [i  ][j  ][k  ]);
				t[i][j][k]->monitor_out        [DIRECTION_DOWN ] (RCA_to_down        [i  ][j  ][k+1]);
				t[i][j][k]->monitor_in         [DIRECTION_NORTH] (RCA_to_south       [i  ][j  ][k  ]);
				t[i][j][k]->monitor_in         [DIRECTION_EAST ] (RCA_to_west        [i+1][j  ][k  ]);
				t[i][j][k]->monitor_in         [DIRECTION_SOUTH] (RCA_to_north       [i  ][j+1][k  ]);
				t[i][j][k]->monitor_in         [DIRECTION_WEST ] (RCA_to_east        [i  ][j  ][k  ]);
				t[i][j][k]->monitor_in         [DIRECTION_UP   ] (RCA_to_down        [i  ][j  ][k  ]);
				t[i][j][k]->monitor_in         [DIRECTION_DOWN ] (RCA_to_up          [i  ][j  ][k+1]);
				
				// on/off ports in emergency mode
				t[i][j][k]->on_off             [DIRECTION_NORTH] (on_off_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->on_off             [DIRECTION_EAST ] (on_off_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->on_off             [DIRECTION_SOUTH] (on_off_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->on_off             [DIRECTION_WEST ] (on_off_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->on_off_neighbor    [DIRECTION_NORTH] (on_off_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->on_off_neighbor    [DIRECTION_EAST ] (on_off_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->on_off_neighbor    [DIRECTION_SOUTH] (on_off_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->on_off_neighbor    [DIRECTION_WEST ] (on_off_to_east     [i  ][j  ][k  ]);
				
				/************ Jimmy added on 2014.12.16 ****************/
				// predict temp.
				t[i][j][k]->PDT             [DIRECTION_NORTH] (PDT_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->PDT             [DIRECTION_EAST ] (PDT_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->PDT             [DIRECTION_SOUTH] (PDT_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->PDT             [DIRECTION_WEST ] (PDT_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->PDT             [DIRECTION_UP ]   (PDT_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->PDT             [DIRECTION_DOWN ] (PDT_to_down     [i  ][j  ][k+1]);

				t[i][j][k]->PDT_neighbor    [DIRECTION_NORTH] (PDT_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->PDT_neighbor    [DIRECTION_EAST ] (PDT_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->PDT_neighbor    [DIRECTION_SOUTH] (PDT_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->PDT_neighbor    [DIRECTION_WEST ] (PDT_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->PDT_neighbor    [DIRECTION_UP]    (PDT_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->PDT_neighbor    [DIRECTION_DOWN ] (PDT_to_up       [i  ][j  ][k+1]);
		
				//buffer information
				t[i][j][k]->buf[0]             [DIRECTION_NORTH] (buf0_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[0]             [DIRECTION_EAST ] (buf0_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[0]             [DIRECTION_SOUTH] (buf0_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[0]             [DIRECTION_WEST ] (buf0_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[0]             [DIRECTION_UP ]   (buf0_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[0]             [DIRECTION_DOWN ] (buf0_to_down     [i  ][j  ][k+1]);
				
				t[i][j][k]->buf_neighbor[0]    [DIRECTION_NORTH] (buf0_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[0]    [DIRECTION_EAST ] (buf0_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[0]    [DIRECTION_SOUTH] (buf0_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[0]    [DIRECTION_WEST ] (buf0_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[0]    [DIRECTION_UP]    (buf0_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[0]    [DIRECTION_DOWN ] (buf0_to_up       [i  ][j  ][k+1]);

				//buffer information
				t[i][j][k]->buf[1]             [DIRECTION_NORTH] (buf1_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[1]             [DIRECTION_EAST ] (buf1_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[1]             [DIRECTION_SOUTH] (buf1_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[1]             [DIRECTION_WEST ] (buf1_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[1]             [DIRECTION_UP ]   (buf1_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[1]             [DIRECTION_DOWN ] (buf1_to_down     [i  ][j  ][k+1]);
			   
				t[i][j][k]->buf_neighbor[1]    [DIRECTION_NORTH] (buf1_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[1]    [DIRECTION_EAST ] (buf1_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[1]    [DIRECTION_SOUTH] (buf1_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[1]    [DIRECTION_WEST ] (buf1_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[1]    [DIRECTION_UP]    (buf1_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[1]    [DIRECTION_DOWN ] (buf1_to_up       [i  ][j  ][k+1]);

				//buffer information
				t[i][j][k]->buf[2]             [DIRECTION_NORTH] (buf2_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[2]             [DIRECTION_EAST ] (buf2_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[2]             [DIRECTION_SOUTH] (buf2_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[2]             [DIRECTION_WEST ] (buf2_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[2]             [DIRECTION_UP ]   (buf2_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[2]             [DIRECTION_DOWN ] (buf2_to_down     [i  ][j  ][k+1]);

				t[i][j][k]->buf_neighbor[2]    [DIRECTION_NORTH] (buf2_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[2]    [DIRECTION_EAST ] (buf2_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[2]    [DIRECTION_SOUTH] (buf2_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[2]    [DIRECTION_WEST ] (buf2_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[2]    [DIRECTION_UP]    (buf2_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[2]    [DIRECTION_DOWN ] (buf2_to_up       [i  ][j  ][k+1]);

				//buffer information
				t[i][j][k]->buf[3]             [DIRECTION_NORTH] (buf3_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[3]             [DIRECTION_EAST ] (buf3_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[3]             [DIRECTION_SOUTH] (buf3_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[3]             [DIRECTION_WEST ] (buf3_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[3]             [DIRECTION_UP ]   (buf3_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[3]             [DIRECTION_DOWN ] (buf3_to_down     [i  ][j  ][k+1]);

				t[i][j][k]->buf_neighbor[3]    [DIRECTION_NORTH] (buf3_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[3]    [DIRECTION_EAST ] (buf3_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[3]    [DIRECTION_SOUTH] (buf3_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[3]    [DIRECTION_WEST ] (buf3_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[3]    [DIRECTION_UP]    (buf3_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[3]    [DIRECTION_DOWN ] (buf3_to_up       [i  ][j  ][k+1]);

				//buffer information
				t[i][j][k]->buf[4]             [DIRECTION_NORTH] (buf4_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[4]             [DIRECTION_EAST ] (buf4_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[4]             [DIRECTION_SOUTH] (buf4_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[4]             [DIRECTION_WEST ] (buf4_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[4]             [DIRECTION_UP ]   (buf4_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[4]             [DIRECTION_DOWN ] (buf4_to_down     [i  ][j  ][k+1]);

				t[i][j][k]->buf_neighbor[4]    [DIRECTION_NORTH] (buf4_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[4]    [DIRECTION_EAST ] (buf4_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[4]    [DIRECTION_SOUTH] (buf4_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[4]    [DIRECTION_WEST ] (buf4_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[4]    [DIRECTION_UP]    (buf4_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[4]    [DIRECTION_DOWN ] (buf4_to_up       [i  ][j  ][k+1]);

				//buffer information
				t[i][j][k]->buf[5]             [DIRECTION_NORTH] (buf5_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[5]             [DIRECTION_EAST ] (buf5_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[5]             [DIRECTION_SOUTH] (buf5_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[5]             [DIRECTION_WEST ] (buf5_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[5]             [DIRECTION_UP ]   (buf5_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[5]             [DIRECTION_DOWN ] (buf5_to_down     [i  ][j  ][k+1]);

				t[i][j][k]->buf_neighbor[5]    [DIRECTION_NORTH] (buf5_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[5]    [DIRECTION_EAST ] (buf5_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[5]    [DIRECTION_SOUTH] (buf5_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[5]    [DIRECTION_WEST ] (buf5_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[5]    [DIRECTION_UP]    (buf5_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[5]    [DIRECTION_DOWN ] (buf5_to_up       [i  ][j  ][k+1]);

				//buffer information
				t[i][j][k]->buf[6]             [DIRECTION_NORTH] (buf6_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[6]             [DIRECTION_EAST ] (buf6_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[6]             [DIRECTION_SOUTH] (buf6_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[6]             [DIRECTION_WEST ] (buf6_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[6]             [DIRECTION_UP ]   (buf6_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[6]             [DIRECTION_DOWN ] (buf6_to_down     [i  ][j  ][k+1]);

				t[i][j][k]->buf_neighbor[6]    [DIRECTION_NORTH] (buf6_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[6]    [DIRECTION_EAST ] (buf6_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[6]    [DIRECTION_SOUTH] (buf6_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[6]    [DIRECTION_WEST ] (buf6_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[6]    [DIRECTION_UP]    (buf6_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[6]    [DIRECTION_DOWN ] (buf6_to_up       [i  ][j  ][k+1]);

				//buffer information
				t[i][j][k]->buf[7]             [DIRECTION_NORTH] (buf7_to_north    [i  ][j  ][k  ]);
				t[i][j][k]->buf[7]             [DIRECTION_EAST ] (buf7_to_east     [i+1][j  ][k  ]);
				t[i][j][k]->buf[7]             [DIRECTION_SOUTH] (buf7_to_south    [i  ][j+1][k  ]);
				t[i][j][k]->buf[7]             [DIRECTION_WEST ] (buf7_to_west     [i  ][j  ][k  ]);
				t[i][j][k]->buf[7]             [DIRECTION_UP ]   (buf7_to_up       [i  ][j  ][k  ]);
				t[i][j][k]->buf[7]             [DIRECTION_DOWN ] (buf7_to_down     [i  ][j  ][k+1]);

				t[i][j][k]->buf_neighbor[7]    [DIRECTION_NORTH] (buf7_to_south    [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[7]    [DIRECTION_EAST ] (buf7_to_west     [i+1][j  ][k  ]);
				t[i][j][k]->buf_neighbor[7]    [DIRECTION_SOUTH] (buf7_to_north    [i  ][j+1][k  ]);
				t[i][j][k]->buf_neighbor[7]    [DIRECTION_WEST ] (buf7_to_east     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[7]    [DIRECTION_UP]    (buf7_to_down     [i  ][j  ][k  ]);
				t[i][j][k]->buf_neighbor[7]    [DIRECTION_DOWN ] (buf7_to_up       [i  ][j  ][k+1]);
				/*******************************************************/
				
				
				//NoximVlink 
				if( k < NoximGlobalParams::mesh_dim_z - 1){
					v[i][j]   ->ack_rx_to_UP   [k]( ack_toV_down  [i][j][k  ] );
					v[i][j]   ->req_rx_to_UP   [k]( req_toT_up    [i][j][k  ] );
					v[i][j]   ->flit_rx_to_UP  [k]( flit_toT_up   [i][j][k  ] );
					v[i][j]   ->ack_tx_to_UP   [k]( ack_toT_up    [i][j][k  ] );
					v[i][j]   ->req_tx_to_UP   [k]( req_toV_down  [i][j][k  ] );
					v[i][j]   ->flit_tx_to_UP  [k]( flit_toV_down [i][j][k  ] );
					
					v[i][j]   ->ack_rx_to_DOWN [k]( ack_toT_down  [i][j][k+1] );
					v[i][j]   ->req_rx_to_DOWN [k]( req_toV_up    [i][j][k+1] );
					v[i][j]   ->flit_rx_to_DOWN[k]( flit_toV_up   [i][j][k+1] );
					v[i][j]   ->ack_tx_to_DOWN [k]( ack_toV_up    [i][j][k+1] );
					v[i][j]   ->req_tx_to_DOWN [k]( req_toT_down  [i][j][k+1] );
					v[i][j]   ->flit_tx_to_DOWN[k]( flit_toT_down [i][j][k+1] );
				}
			}
		}
    cout<<"dummy NoximNoP_data structure..."<<endl;
	NoximNoP_data tmp_NoP;
    tmp_NoP.sender_id = NOT_VALID;

    for ( i = 0; i < DIRECTIONS; i++) {
		tmp_NoP.channel_status_neighbor[i].free_slots = NOT_VALID;
		tmp_NoP.channel_status_neighbor[i].available  = false;
    }

    cout<<"Clear signals for borderline nodes..."<<endl;
	for( i=0; i<=NoximGlobalParams::mesh_dim_x; i++){
		for( k=0; k<=NoximGlobalParams::mesh_dim_z; k++){
			j = NoximGlobalParams::mesh_dim_y;
			req_to_south       [i][0][k] = 0;
			ack_to_north       [i][0][k] = 0;
			req_to_north       [i][j][k] = 0;
			ack_to_south       [i][j][k] = 0;
			
			free_slots_to_south[i][0][k].write(NOT_VALID);
			free_slots_to_north[i][j][k].write(NOT_VALID);
		
			RCA_to_south       [i][0][k].write(0);
			RCA_to_north       [i][j][k].write(0);
			
			/*** RCA port connection (Jimmy added on 2012.06.27)***/
			RCA_data_to_south0[i][0][k].write(0);
			RCA_data_to_south1[i][0][k].write(0);
			RCA_data_to_north0[i][NoximGlobalParams::mesh_dim_y][k].write(0);
			RCA_data_to_north1[i][NoximGlobalParams::mesh_dim_y][k].write(0);					
			/********************************************************/	
			
			on_off_to_south    [i][0][k].write(NOT_VALID);
			on_off_to_north    [i][j][k].write(NOT_VALID);
			
			/**** Port connection (Jimmy added on 2014.12.16) ****/
			buf0_to_south    [i][0][k].write(NOT_VALID); 
			buf0_to_north    [i][j][k].write(NOT_VALID);

			buf1_to_south    [i][0][k].write(NOT_VALID);
			buf1_to_north    [i][j][k].write(NOT_VALID);

			buf2_to_south    [i][0][k].write(NOT_VALID);
			buf2_to_north    [i][j][k].write(NOT_VALID);

			buf3_to_south    [i][0][k].write(NOT_VALID);
			buf3_to_north    [i][j][k].write(NOT_VALID);

			buf4_to_south    [i][0][k].write(NOT_VALID);
			buf4_to_north    [i][j][k].write(NOT_VALID);

			buf5_to_south    [i][0][k].write(NOT_VALID);
			buf5_to_north    [i][j][k].write(NOT_VALID);

			buf6_to_south    [i][0][k].write(NOT_VALID);
			buf6_to_north    [i][j][k].write(NOT_VALID);

			buf7_to_south    [i][0][k].write(NOT_VALID);
			buf7_to_north    [i][j][k].write(NOT_VALID);

			PDT_to_south    [i][0][k].write(100);
			PDT_to_north    [i][j][k].write(100);
			/*****************************************************/
			
			NoP_data_to_south  [i][0][k].write(tmp_NoP);
			NoP_data_to_north  [i][j][k].write(tmp_NoP);
		}
	}
	for( j=0; j<=NoximGlobalParams::mesh_dim_y; j++)
		for( k=0; k<=NoximGlobalParams::mesh_dim_z; k++){
			i = NoximGlobalParams::mesh_dim_x;
			req_to_east       [0][j][k] = 0;
			ack_to_west       [0][j][k] = 0;
			req_to_west       [i][j][k] = 0;
			ack_to_east       [i][j][k] = 0;

			free_slots_to_east[0][j][k].write(NOT_VALID);
			free_slots_to_west[i][j][k].write(NOT_VALID);
		
			RCA_to_east       [0][j][k].write(0);
			RCA_to_west       [i][j][k].write(0);

			on_off_to_east    [0][j][k].write(NOT_VALID);
			on_off_to_west    [i][j][k].write(NOT_VALID);

			/**** Port connection (Jimmy added on 2014.12.16) ****/
			buf0_to_east    [0][j][k].write(NOT_VALID);
			buf0_to_west    [i][j][k].write(NOT_VALID);

			buf1_to_east    [0][j][k].write(NOT_VALID);
			buf1_to_west    [i][j][k].write(NOT_VALID);

			buf2_to_east    [0][j][k].write(NOT_VALID);
			buf2_to_west    [i][j][k].write(NOT_VALID);

			buf3_to_east    [0][j][k].write(NOT_VALID);
			buf3_to_west    [i][j][k].write(NOT_VALID);

			buf4_to_east    [0][j][k].write(NOT_VALID);
			buf4_to_west    [i][j][k].write(NOT_VALID);

			buf5_to_east    [0][j][k].write(NOT_VALID);
			buf5_to_west    [i][j][k].write(NOT_VALID);

			buf6_to_east    [0][j][k].write(NOT_VALID);
			buf6_to_west    [i][j][k].write(NOT_VALID);

			buf7_to_east    [0][j][k].write(NOT_VALID);
			buf7_to_west    [i][j][k].write(NOT_VALID);

			PDT_to_east    [0][j][k].write(100);
			PDT_to_west    [i][j][k].write(100);
			/*****************************************************/
			
			NoP_data_to_east  [0][j][k].write(tmp_NoP);
			NoP_data_to_west  [i][j][k].write(tmp_NoP);
		}
	for( i=0; i<=NoximGlobalParams::mesh_dim_x; i++){
		for( j=0; j<=NoximGlobalParams::mesh_dim_y; j++){
			k = NoximGlobalParams::mesh_dim_z - 2; //?????????????????????????????????????????
			req_toT_down       [i][j][0] = 0;
			ack_toT_up         [i][j][k] = 0;
			req_toT_up         [i][j][k] = 0;
			ack_toT_down       [i][j][0] = 0;
		
			free_slots_to_down[i][j][0].write(NOT_VALID);
			free_slots_to_up  [i][j][k].write(NOT_VALID);
		
			RCA_to_down       [i][j][0].write(0);
			RCA_to_up         [i][j][k].write(0);
		
			/**** Port connection (Jimmy added on 2014.12.16) ****/
			buf0_to_down    [i][j][0].write(NOT_VALID);
			buf0_to_up    [i][j][k].write(NOT_VALID);

			buf1_to_down    [i][j][0].write(NOT_VALID);
			buf1_to_up    [i][j][k].write(NOT_VALID);

			buf2_to_down    [i][j][0].write(NOT_VALID);
			buf2_to_up    [i][j][k].write(NOT_VALID);

			buf3_to_down    [i][j][0].write(NOT_VALID);
			buf3_to_up    [i][j][k].write(NOT_VALID);

			buf4_to_down    [i][j][0].write(NOT_VALID);
			buf4_to_up    [i][j][k].write(NOT_VALID);

			buf5_to_down    [i][j][0].write(NOT_VALID);
			buf5_to_up    [i][j][k].write(NOT_VALID);

			buf6_to_down    [i][j][0].write(NOT_VALID);
			buf6_to_up    [i][j][k].write(NOT_VALID);

			buf7_to_down    [i][j][0].write(NOT_VALID);
			buf7_to_up    [i][j][k].write(NOT_VALID);
		
			PDT_to_down    [i][j][0].write(100);
			PDT_to_up    [i][j][k].write(100);
			/*****************************************************/
		
			NoP_data_to_down  [i][j][0].write(tmp_NoP);
			NoP_data_to_up    [i][j][k].write(tmp_NoP);
		}
    }
    // invalidate reservation table entries for non-exhistent channels
 	for( i=0; i<NoximGlobalParams::mesh_dim_x; i++)
		for( k=0; k<NoximGlobalParams::mesh_dim_z; k++){
			j = NoximGlobalParams::mesh_dim_y;
			t[i][0  ][k]->r->reservation_table.invalidate(DIRECTION_NORTH);
			t[i][j-1][k]->r->reservation_table.invalidate(DIRECTION_SOUTH);
		}
	for( j=0; j<NoximGlobalParams::mesh_dim_y; j++)
		for( k=0; k<NoximGlobalParams::mesh_dim_z; k++){
			i = NoximGlobalParams::mesh_dim_x;
			t[0  ][j][k]->r->reservation_table.invalidate(DIRECTION_WEST);
			t[i-1][j][k]->r->reservation_table.invalidate(DIRECTION_EAST);
		}   
	
	for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++)
		for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++)
			for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++){
				t[x][y][z]->vertical_free_slot_out(vertical_free_slot[x][y][z]);
				for( int i=0; i < NoximGlobalParams::mesh_dim_z; i++ )
					t[x][y][z]->vertical_free_slot_in[i]( vertical_free_slot[x][y][i] );
			}
	
	cout<<"Initial emergency mode..."<<endl;
	for (int k=0; k<NoximGlobalParams::mesh_dim_z; k++)
		for (int j=0; j<NoximGlobalParams::mesh_dim_y; j++)
			for (int i=0; i<NoximGlobalParams::mesh_dim_x; i++){
				if(NoximGlobalParams::throt_type == THROT_TEST){
					if(k < (NoximGlobalParams::mesh_dim_z - 1)){
				//if(((i == 3))&&((j == 3))){
				//if( ((i == 1)&&(j == 1)) || ((i == 1)&&(j == 2)) || ((i == 2)&&(j == 2)) || ((i == 2)&&(j == 1))
				// || ((i == 6)&&(j == 5)) || ((i == 5)&&(j == 6)) || ((i == 6)&&(j == 6)) || ((i == 5)&&(j == 5)) ){
				//if( ((i == 5)&&(j == 5)) || ((i == 5)&&(j == 4)) || ((i == 4)&&(j == 4)) || ((i == 4)&&(j == 5)) ){
				//if( ((i == 1)&&(j == 1)) || ((i == 6)&&(j == 6)) ){
						if ( false ){
							throttling[i][j][k] = 1;
							t[i][j][k]->pe->IntoEmergency();
							t[i][j][k]->r ->IntoEmergency();
						}
						else{
							throttling[i][j][k] = 0;
							t[i][j][k]->pe->OutOfEmergency();
							t[i][j][k]->r ->OutOfEmergency();
						}
						if( i >= NoximGlobalParams::ROC_DOWN && i <= NoximGlobalParams::ROC_UP && 
							j >= NoximGlobalParams::ROC_DOWN && j <= NoximGlobalParams::ROC_UP )
							beltway[i][j][k]    = true;
						else
							beltway[i][j][k]    = false;
				
					}
					else {
						throttling[i][j][k] = 0;
						beltway[i][j][k]    = false;
						reShot[i][j][k]     = 0; //Jimmy added on 2017.05.29
						t[i][j][k]->pe->OutOfEmergency();
						t[i][j][k]->r ->OutOfEmergency();
					}
				}
				else{
					throttling[i][j][k] = 0;
					beltway[i][j][k]    = false;
					t[i][j][k]->pe->OutOfEmergency();
					t[i][j][k]->r ->OutOfEmergency();
				}		
				if(NoximGlobalParams::init_throttle)
				{
					throttling[i][j][k] = 1;
					t[i][j][k]->pe->IntoEmergency();
					t[i][j][k]->r ->IntoEmergency();
				}
			}
	int non_beltway_layer,non_throt_layer;
	int col_max,col_min,row_max,row_min;
	findNonXLayer(non_throt_layer,non_beltway_layer);
	calROC(col_max,col_min,row_max,row_min,non_beltway_layer);
	for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++ )
		for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++ ) 
			for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++ ){
				if(NoximGlobalParams::throt_type != THROT_TEST)
					t[x][y][z]->pe->RTM_set_var(non_throt_layer, non_beltway_layer, col_max, col_min, row_max, row_min);
				else
					t[x][y][z]->pe->RTM_set_var(non_throt_layer, non_beltway_layer, NoximGlobalParams::ROC_UP, NoximGlobalParams::ROC_DOWN, NoximGlobalParams::ROC_UP, NoximGlobalParams::ROC_DOWN);
			}
	cout<<"initial prediction files..."<<endl;
	string TEMP_filename0 = string("results/TEMP/TEMP0");
	TEMP_filename0 = shortMarkFileName( TEMP_filename0 );
	string TEMP_filename1 = string("results/TEMP/TEMP1");
	TEMP_filename1 = MarkFileName( TEMP_filename1 );
	string TEMP_filename2 = string("results/TEMP/TEMP2");
	TEMP_filename2 = MarkFileName( TEMP_filename2 );
	string TEMP_filename3 = string("results/TEMP/TEMP3");
	TEMP_filename3 = MarkFileName( TEMP_filename3 );
	string TEMP_filename4 = string("results/TEMP/TEMP4");
	TEMP_filename4 = MarkFileName( TEMP_filename4 );
	string TEMP_filename5 = string("results/TEMP/TEMP5");
	TEMP_filename5 = MarkFileName( TEMP_filename5 );
	string TEMP_filename6 = string("results/TEMP/TEMP6");
	TEMP_filename6 = MarkFileName( TEMP_filename6 );
	string Predict_filename = string("results/Predict/pre_temp");//Jack added on 2019.02.24
	Predict_filename = MarkFileName( Predict_filename );//Jack added on 2019.02.24
	string THROT_filename = string("results/THROT/throttling");//Jack added on 2019.04.22
	THROT_filename = shortMarkFileName( THROT_filename );//Jack added on 2019.04.22
	string WEIGHT_filename = string("results/WEIGHT/weight");//Jack added on 2019.09.30
	WEIGHT_filename = MarkFileName( WEIGHT_filename );//Jack added on 2019.09.30
	results_log_temp0.open( TEMP_filename0.c_str(), ios::out );
	results_log_temp1.open( TEMP_filename1.c_str(), ios::out );
	results_log_temp2.open( TEMP_filename2.c_str(), ios::out );
	results_log_temp3.open( TEMP_filename3.c_str(), ios::out );
	results_log_temp4.open( TEMP_filename4.c_str(), ios::out );
	results_log_temp5.open( TEMP_filename5.c_str(), ios::out );
	results_log_temp6.open( TEMP_filename6.c_str(), ios::out );	
	results_log_pre_temp.open( Predict_filename.c_str(), ios::out );//Jack added on 2019.02.24
	results_log_throttling.open( THROT_filename.c_str(), ios::out );//Jack added on 2019.04.22
	results_log_weight.open( WEIGHT_filename.c_str(), ios::out );//Jack added on 2019.04.22
	if(!results_log_temp0.is_open())cout<<"Cannot open "<< TEMP_filename0.c_str() <<endl;
	if(!results_log_temp1.is_open())cout<<"Cannot open "<< TEMP_filename1.c_str() <<endl;
	if(!results_log_temp2.is_open())cout<<"Cannot open "<< TEMP_filename2.c_str() <<endl;
	if(!results_log_temp3.is_open())cout<<"Cannot open "<< TEMP_filename3.c_str() <<endl;
	if(!results_log_temp4.is_open())cout<<"Cannot open "<< TEMP_filename4.c_str() <<endl;
	if(!results_log_temp5.is_open())cout<<"Cannot open "<< TEMP_filename5.c_str() <<endl;
	if(!results_log_temp6.is_open())cout<<"Cannot open "<< TEMP_filename6.c_str() <<endl;	
	for(int z = 0; z < NoximGlobalParams::mesh_dim_z; z++)
		for(int y = 0; y < NoximGlobalParams::mesh_dim_y; y++)
			for(int x = 0; x < NoximGlobalParams::mesh_dim_x; x++){
				results_log_temp0<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_temp1<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_temp2<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_temp3<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_temp4<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_temp5<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_temp6<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";	
			}
	results_log_temp0<<"\n";
	results_log_temp1<<"\n";
	results_log_temp2<<"\n";
	results_log_temp3<<"\n";
	results_log_temp4<<"\n";
	results_log_temp5<<"\n";
	results_log_temp6<<"\n";
	
	/*** Record the Prediciton Error ***/
	cout<<"initial prediction error..."<<endl;
	string PREDERR_filename0 = string("results/PredictError/PredictError0");
	PREDERR_filename0 = MarkFileName( PREDERR_filename0 );
	string PREDERR_filename1 = string("results/PredictError/PredictError1");
	PREDERR_filename1 = MarkFileName( PREDERR_filename1 );
	string PREDERR_filename2 = string("results/PredictError/PredictError2");
	PREDERR_filename2 = MarkFileName( PREDERR_filename2 );
	string PREDERR_filename3 = string("results/PredictError/PredictError3");
	PREDERR_filename3 = MarkFileName( PREDERR_filename3 );
	string PREDERR_filename4 = string("results/PredictError/PredictError4");
	PREDERR_filename4 = MarkFileName( PREDERR_filename4 );
	string PREDERR_filename5 = string("results/PredictError/PredictError5");
	PREDERR_filename5 = MarkFileName( PREDERR_filename5 );
	string PREDERR_filename6 = string("results/PredictError/PredictError6");
	PREDERR_filename6 = MarkFileName( PREDERR_filename6 );	
	results_log_prederr0.open( PREDERR_filename0.c_str(), ios::out );
	results_log_prederr1.open( PREDERR_filename1.c_str(), ios::out );
	results_log_prederr2.open( PREDERR_filename2.c_str(), ios::out );
	results_log_prederr3.open( PREDERR_filename3.c_str(), ios::out );
	results_log_prederr4.open( PREDERR_filename4.c_str(), ios::out );
	results_log_prederr5.open( PREDERR_filename5.c_str(), ios::out );
	results_log_prederr6.open( PREDERR_filename6.c_str(), ios::out );	
	if(!results_log_prederr0.is_open())cout<<"Cannot open "<< PREDERR_filename0.c_str() <<endl;
	if(!results_log_prederr1.is_open())cout<<"Cannot open "<< PREDERR_filename1.c_str() <<endl;
	if(!results_log_prederr2.is_open())cout<<"Cannot open "<< PREDERR_filename2.c_str() <<endl;
	if(!results_log_prederr3.is_open())cout<<"Cannot open "<< PREDERR_filename3.c_str() <<endl;
	if(!results_log_prederr4.is_open())cout<<"Cannot open "<< PREDERR_filename4.c_str() <<endl;
	if(!results_log_prederr5.is_open())cout<<"Cannot open "<< PREDERR_filename5.c_str() <<endl;
	if(!results_log_prederr6.is_open())cout<<"Cannot open "<< PREDERR_filename6.c_str() <<endl;	
	for(int z = 0; z < NoximGlobalParams::mesh_dim_z; z++)
		for(int y = 0; y < NoximGlobalParams::mesh_dim_y; y++)
			for(int x = 0; x < NoximGlobalParams::mesh_dim_x; x++){
				results_log_prederr0<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_prederr1<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_prederr2<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_prederr3<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_prederr4<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_prederr5<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_prederr6<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";	
			}
	results_log_prederr0<<"\n";
	results_log_prederr1<<"\n";
	results_log_prederr2<<"\n";
	results_log_prederr3<<"\n";
	results_log_prederr4<<"\n";
	results_log_prederr5<<"\n";
	results_log_prederr6<<"\n";
	
	/*sensor allocation colin added on 2015.06.15  */
	cout<<"initial prediction sensor files...temp_s, temp_s_r"<<endl;  
	string TEMP_filename0_s   = string("results/TEMP/TEMP0_s");
	string TEMP_filename0_s_r = string("results/TEMP/TEMP0_s_r");
	
	TEMP_filename0_s   = MarkFileName( TEMP_filename0_s );
	TEMP_filename0_s_r = MarkFileName( TEMP_filename0_s_r );
	
	results_log_temp0_s.open  ( TEMP_filename0_s.c_str(), ios::out );
	results_log_temp0_s_r.open( TEMP_filename0_s_r.c_str(), ios::out );
	
	if(!results_log_temp0_s.is_open())cout  <<"Cannot open "<< TEMP_filename0_s.c_str()   <<endl;
	if(!results_log_temp0_s_r.is_open())cout<<"Cannot open "<< TEMP_filename0_s_r.c_str() <<endl;
	
	for(int z = 0; z < NoximGlobalParams::mesh_dim_z; z++)
		for(int y = 0; y < NoximGlobalParams::mesh_dim_y; y++)
			for(int x = 0; x < NoximGlobalParams::mesh_dim_x; x++){
				results_log_temp0_s  <<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
				results_log_temp0_s_r<<"router["<<x<<"]["<<y<<"]["<<z<<"]\t";
			}
	results_log_temp0_s  <<"\n";
	results_log_temp0_s_r<<"\n";
	/*Prediction model weight file Jack add on 2019.06.29*/
	win.open("weight.txt",ios::in);
	for(int i=0;i<19;i++){
		win >> weight[0][i];
		//cout << weight[0][i] << endl;
	}
	for(int i=1;i<64;i++)
		for(int j=0;j<19;j++)
			weight[i][j] = weight[0][j];
			
		
	cycle = 0;
	avg_error = 0;
	node = 0;
	partial_throttled_node = 0;
	for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++)	
		for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
			full_throttling[m][n] = 0;
			par_throttling[m][n] = 0;
	}
	
	win.close();
	/*DQN_weight Jack add on 2019.09.23*/
	win_tar.open("NN_weight_tar.txt",ios::in);
	win_eva.open("NN_weight_eva.txt",ios::in);
	
	
	//srand(time(NULL));
	for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++)	
		for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
			for(int i=0;i<3;i++){
				for(int j=0;j<7;j++){
					double max = 1.0;
					double min = -1.0;
					//if(j == 5){ 
						//eva_L1_w[m][n][j][i] = (double) (rand() - (RAND_MAX/2)) / (RAND_MAX + 1.0);
						//tar_L1_w[m][n][j][i] = (double) (rand() - (RAND_MAX/2)) / (RAND_MAX + 1.0); 
					//}
					win_tar>>tar_L1_w[m][n][j][i];
					win_eva>>eva_L1_w[m][n][j][i];
					//cout<<t[m][n][0]->r->eva_L1_w[i][j] << " ";
					if(NoximGlobalParams::DQN_init)
					{
						for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
							for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
								for(int i=0;i<5;i++)
									for(int j=0;j<3;j++)
										Q_table[m][n][i][j] = 0;
							}
						}
						//eva_L1_w[m][n][j][i] = /*1.0/(i+1) +*/ (double) (rand() - (RAND_MAX/2)) / (RAND_MAX + 1.0) / 10;
						//tar_L1_w[m][n][j][i] = eva_L1_w[m][n][j][i];//1.0/(2-i%2) + (double) (rand() - (RAND_MAX/2)) / (RAND_MAX + 1.0);
						//eva_L1_w[m][n][6][i] = 0;
						//tar_L1_w[m][n][6][i] = 0;
					}
				}
			}
			//cout << endl;
		}
	win_tar.close();
	win_eva.close();
	total_transmit = 0;
	terminal = 0;
	count = 0;
	
	if(!NoximGlobalParams::DQN_init)
	{
		/*din.open("DQN_D.txt",ios::in);
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				for(int i=0;i<D_MAX;i++){
					for(int j=0;j<D_col;j++){
						din >> D[m][n][i][j];
					}
					for(int idx=0;idx<6;++idx)//S_t
						din >> D[m][n][i][idx];
					for(int idx=0;idx<6;++idx)//S_t+1
						din >> D[m][n][i][idx+9];
					din >> D[m][n][i][20];//a_t
					din >> D[m][n][i][21];//r_t
					din >> D[m][n][i][22];//cycle
				}
			}
		}
		din >> count;
		din.close();*/
		din.open("QN_D.txt",ios::in);
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				for(int i=0;i<D_MAX;i++){
					/*for(int j=0;j<D_col;j++){
						din >> D[m][n][i][j];
					}*/
					for(int idx=0;idx<6;++idx)//S_t
						din >> D[m][n][i][idx];
					for(int idx=0;idx<6;++idx)//S_t+1
						din >> D[m][n][i][idx+9];
					din >> D[m][n][i][20];//a_t
					din >> D[m][n][i][21];//r_t
					din >> D[m][n][i][22];//cycle
				}
			}
		}
		din >> count;
		din.close();
		qt_in.open("Q_table.txt",ios::in);
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				for(int i=0;i<5;i++){
					//cout<< ((n*8+m)*200+(n*8+m)) <<": ";
					double temp;
					qt_in>> temp;
					
					for(int j=0;j<3;j++){
						qt_in>>Q_table[m][n][i][j];
						//cout<< Q_table[m][n][i][j] << " ";
					}
					//cout<< endl;
				}
			}
		}
		qt_in.close();
	}
	
	temp_log_csv.open("../temp.csv",ios::out);
	temp_log_csv<<NoximGlobalParams::mesh_dim_x<<','<<NoximGlobalParams::mesh_dim_y<<','<<NoximGlobalParams::mesh_dim_z;
	for(int ridx=0;ridx<NoximGlobalParams::mesh_dim_z*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_x-3;++ridx)
		temp_log_csv<<','<<0;
	temp_log_csv<<'\n';
	temp_log_csv.close();
	number_in.open("number.txt",ios::in);
	number_in >> number;
	number_in.close();
	cout << number << endl;
	srand(time(NULL));
	
}

void NoximNoC::entry(){  //Foster big modified - 09/11/12
	//reset power
	if (reset.read()) {
		//in reset phase, reset power value 
		for(int k=0; k < NoximGlobalParams::mesh_dim_z; k++){
			for(int j=0; j < NoximGlobalParams::mesh_dim_y; j++){
				for(int i=0; i < NoximGlobalParams::mesh_dim_x; i++){		
			
					t[i][j][k]->r->stats.power.resetPwr();
					t[i][j][k]->r->stats.power.resetTransientPwr();
					t[i][j][k]->r->stats.temperature = INIT_TEMP - 273.15 ;
					
					t[i][j][k]->r->stats.last_pre_temperature1 = 0; //Jimmy added on 2013.03.06
					t[i][j][k]->r->stats.predict_error1 = 0; //Jimmy added on 2013.03.06
				}
			}
		}
		if(base_cycle == -2)
		{
			for(int ridx=0; ridx < 8*8*4 ; ridx++){
				init_temp[ridx] = INIT_TEMP - 273.15;
			}
			init_temp_csv.open("init_temp.csv");
			while(init_temp_csv.peek()!=-1)
			{
				int tidx;
				init_temp_csv>>tidx;
				init_temp_csv.ignore();
				if(init_temp_csv.peek()!=-1)
				{
					init_temp_csv>>init_temp[tidx];
					init_temp_csv.ignore();
				}
			}
			init_temp_csv.close();
			HS_interface->setInitTemp(init_temp);
		}
		base_cycle += 1;
	}
	else{
		static double NN_in[10][10][10];
		int CurrentCycle    = getCurrentCycleNum() - base_cycle ;
		int CurrentCycleMod = (CurrentCycle % (int) (TEMP_REPORT_PERIOD));
		/*for(int j=0; j < NoximGlobalParams::mesh_dim_y; j++)	
			for(int i=0; i < NoximGlobalParams::mesh_dim_x; i++){
				t[i][j][0]->r->BTD = 0;
				t[i][j][0]->r->BTD = 0;
				t[i][j][0]->r->BTD = 0;
				t[i][j][0]->r->BTD = 0;
				t[i][j][0]->r->BTD = 0;
				
			}
		for(int j=0; j < NoximGlobalParams::mesh_dim_y; j++)	
			for(int i=0; i < NoximGlobalParams::mesh_dim_x; i++){
				double mu;
				if(throttling[i][j][0] == 1)
					mu = 0;
				else if(throttling[i][j][0] == 0)
					mu = 1;
				else if(throttling[i][j][0] == 2)
					mu = 0.5;
				t[i][j][0]->r->BTD += t[i][j][0]->r->buffer[0].Size()*mu;
				t[i][j][0]->r->BTD += t[i][j][0]->r->buffer[1].Size()*mu;
				t[i][j][0]->r->BTD += t[i][j][0]->r->buffer[2].Size()*mu;
				t[i][j][0]->r->BTD += t[i][j][0]->r->buffer[3].Size()*mu;
				t[i][j][0]->r->BTD += t[i][j][0]->r->buffer[6].Size()*mu;
				t[i][j][0]->r->BTD /= 5;
				
			}*/
		if(CurrentCycleMod == ((int) (TEMP_REPORT_PERIOD) - NoximGlobalParams::clean_stage_time) || CurrentCycle == 1){
			
			//cout << "[INFO]" << CurrentCycleMod<<endl;
			for(int j=0; j < NoximGlobalParams::mesh_dim_y; j++)	
				for(int i=0; i < NoximGlobalParams::mesh_dim_x; i++){
					t[i][j][0]->r->in[0] = t[i][j][0]->r->buffer[0].Size();
					t[i][j][0]->r->in[1] = t[i][j][0]->r->buffer[1].Size();
					t[i][j][0]->r->in[2] = t[i][j][0]->r->buffer[2].Size();
					t[i][j][0]->r->in[3] = t[i][j][0]->r->buffer[3].Size();
					t[i][j][0]->r->in[4] = t[i][j][0]->r->buffer[6].Size();
					for(int k=0;k<5;k++){
						NN_in[i][j][(k+4)%5] = t[i][j][0]->r->buffer[(k+6)%7].Size();
						NN_in[i][j][(k+4)%5] = NN_in[i][j][(k+4)%5] / NoximGlobalParams::buffer_depth;
					}
					/*NN_in[i][j][0] = (j+1)<NoximGlobalParams::mesh_dim_y ? (((double)t[i][j+1][0]->pe->t_quota)/NoximGlobalParams::static_quota) : 0;
					NN_in[i][j][1] = (i+1)<NoximGlobalParams::mesh_dim_x ? (((double)t[i+1][j][0]->pe->t_quota)/NoximGlobalParams::static_quota) : 0;
					NN_in[i][j][2] = (j-1)>=0 ? (((double)t[i][j-1][0]->pe->t_quota)/NoximGlobalParams::static_quota) : 0;
					NN_in[i][j][3] = (i-1)>=0 ? (((double)t[i-1][j][0]->pe->t_quota)/NoximGlobalParams::static_quota) : 0;*/
					NN_in[i][j][0] = 0;
					NN_in[i][j][1] = 0;
					NN_in[i][j][2] = 0;
					NN_in[i][j][3] = 0;
					//NN_in[i][j][4] = (HS_interface->tempRec(xyz2Id(i,j,0),0)-80)/20;
					NN_in[i][j][4] = HS_interface->tempRec(xyz2Id(i,j,0),0);
				}
			for(int o=0; o < NoximGlobalParams::mesh_dim_z; o++)
				for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++) 
					for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
						int idx = xyz2Id( m, n, o);
						//NN_in[m][n][5] = ((double)t[m][n][o]->pe->t_quota)/NoximGlobalParams::static_quota;//throttling[m][n][o];//(TemperatureTrace[3*idx]-80)/20;
						NN_in[m][n][5] = 0;
					}
			
		}
		if(  CurrentCycleMod == ((int) (TEMP_REPORT_PERIOD) - NoximGlobalParams::clean_stage_time)){
			
			setCleanStage();
			
			
		}
		if( CurrentCycleMod == 0 ){
			EndCleanStage();
			//accumulate steady power after warm-up time
			if( CurrentCycle > (int)( NoximGlobalParams::stats_warm_up_time ) )
				steadyPwr2PtraceFile();
			//Calculate Temperature
			if( NoximGlobalParams::cal_temp ){
				transPwr2PtraceFile();
				HS_interface->Temperature_calc(instPowerTrace, TemperatureTrace);
		
				setTemperature(NN_in);
				
			}
			//check temperature, whether set emergency mode or not (Jimmy modifed on 2016.03.24)
			if(NoximGlobalParams::sensor_allocate == 0) EmergencyDecision();
			else EmergencyDecision_sen();
			
			
			//Buffer length Adjustment (Jimmy added on 2017.05.02)
			if(NoximGlobalParams::Buffer_Allocation == BUFFER_ALLOCATION){
				for(int k=0; k < NoximGlobalParams::mesh_dim_z; k++)
				for(int j=0; j < NoximGlobalParams::mesh_dim_y; j++)	
				for(int i=0; i < NoximGlobalParams::mesh_dim_x; i++){		
					t[i][j][k]->r->TBDB();
				}
			}
		}
		if(CurrentCycleMod == 0 || CurrentCycle == 1)
		{
			cout << "\n\nCycle : "<<cycle<<" Eclipse time : " << (time(NULL) - times) << "s" << endl;
			times = time(NULL);
			
			TransientLog();
			int tqta = 0;
			for(int o=0; o < NoximGlobalParams::mesh_dim_z; o++)
				for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
					for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
						if(o==0){
							//cout<<n*NoximGlobalParams::mesh_dim_x+m<< " : ";
							//cout<<t[m][n][0]->pe->t_quota;
							//cout<<(m==NoximGlobalParams::mesh_dim_x-1?"\n":" ");
							tqta += t[m][n][o]->pe->t_quota;
							cout << setw(6) << t[m][n][o]->pe->t_quota << "      ";
						}
					}
					cout << '\n';
				}
			cout << "Total t_quota : " << tqta << "\n";
			
			for(int modular=0;modular<1;++modular){
				cout<<"\n"<<(modular==0?"router":modular==1?"mem   ":modular==2?"mac   ":"      ")<<" :\n\n";
				for(int o=0; o < NoximGlobalParams::mesh_dim_z; o++){
					int paddings = (NoximGlobalParams::mesh_dim_x-1)*4;
					cout<<string(paddings,' ')<<"Layer";
					printf("%3d",o);
					cout<<string(paddings,' ')<<(o==NoximGlobalParams::mesh_dim_z-1?"\n":"        ");
				}
				for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
					for(int o=0; o < NoximGlobalParams::mesh_dim_z; o++){
						for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
							int ridx = o*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_x+n*NoximGlobalParams::mesh_dim_x+m;
							printf("%7.3lf ",HS_interface->tempRec(ridx,modular));
							cout<<(m==NoximGlobalParams::mesh_dim_x-1?(o==NoximGlobalParams::mesh_dim_z-1?"\n":"        "):"");
						}
					}
					/*for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++)
						cout << "      " << setw(6) << NN_in[m][n][0] << "      " << "      ";
					cout << "\n";
					for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++)
						cout << setw(6) << NN_in[m][n][1] << setw(6) << NN_in[m][n][4] << setw(6) << NN_in[m][n][3] << "      ";
					cout << "\n";
					for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++)
						cout << "      " << setw(6) << NN_in[m][n][0] << "      " << "      ";
					cout << "\n";*/
				}
			}	
			cout<<"\n\n";
			for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
				for(int o=0; o < NoximGlobalParams::mesh_dim_z; o++){
					for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
						printf("%7.3lf ",t[m][n][o]->r->stats.pre_temperature1);
						cout<<(m==NoximGlobalParams::mesh_dim_x-1?(o==NoximGlobalParams::mesh_dim_z-1?"\n":"        "):"");
					}
				}
			}
			cout<<"\n\n";
			temp_log_csv.open("../temp.csv",ios::app);
			for(int modular=0;modular<3;++modular){
				for(int o=0; o < NoximGlobalParams::mesh_dim_z; o++){
					for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
						for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
							int ridx = o*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_x+n*NoximGlobalParams::mesh_dim_x+m;
							temp_log_csv<<HS_interface->tempRec(ridx,modular)<<(ridx==NoximGlobalParams::mesh_dim_z*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_x-1?"\n":",");
						}
					}
				}
			}
			temp_log_csv.close();
		}
		if( CurrentCycle == NoximGlobalParams::simulation_time && NoximGlobalParams::cal_temp){ //Calculate steady state temp.
			cout<<"Calculate SteadyTemp at "<<CurrentCycle<<endl;
			HS_interface->steadyTmp(t);
		}  
	}      
}

NoximTile *NoximNoC::searchNode(const int id) const{
	int i,j,k;
	NoximCoord node = id2Coord(id);
	return t[node.x][node.y][node.z];
}

//----------Modified by CMH
void NoximNoC::transPwr2PtraceFile()
{
    int idx = 0;	
	int m, n, o;
	/*================================Begin of collecting POWER TRACE ======================================*/
	for(o=0; o < NoximGlobalParams::mesh_dim_z; o++)
	for(n=0; n < NoximGlobalParams::mesh_dim_y; n++)
	for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
		idx = xyz2Id( m, n, o);
		
		double a = t[m][n][o]->r->stats.power.getTransientRouterPower();
		//router : offset = 0
		//instPowerTrace[3*idx] = t[m][n][o]->r->stats.power.getTransientRouterPower()/(TEMP_REPORT_PERIOD *1e-9);
		//overallPowerTrace[3*idx] += instPowerTrace[3*idx];
		instPowerTrace[3*idx] = t[m][n][o]->r->stats.power.getTransientRouterPower();
		results_log_pwr << instPowerTrace[3*idx]<<"\t";	
				
        //uP_mem : offset = 1
		//instPowerTrace[3*idx+1] = t[m][n][o]->r->stats.power.getTransientMEM()/(TEMP_REPORT_PERIOD *1e-9);
		//overallPowerTrace[3*idx+1] += instPowerTrace[3*idx+1];
		instPowerTrace[3*idx+1] = t[m][n][o]->r->stats.power.getTransientMEMPower();
		results_log_pwr << instPowerTrace[3*idx+1]<<"\t";	

		//uP_mac : offset = 2
		//instPowerTrace[3*idx+2] = t[m][n][o]->r->stats.power.getTransientFPMACPower()/(TEMP_REPORT_PERIOD *1e-9);
		//overallPowerTrace[3*idx+2] += instPowerTrace[3*idx+2];
		instPowerTrace[3*idx+2] = t[m][n][o]->r->stats.power.getTransientFPMACPower();
		results_log_pwr << instPowerTrace[3*idx+2]<<"\t";	

    	t[m][n][o]->r->stats.power.resetTransientPwr();
	}
	/*================================End of COLLECTING Power TRACE=================================================*/
	results_log_pwr<<"\n";
}

void NoximNoC::steadyPwr2PtraceFile()
{
    int idx = 0;	
	int m, n, o;
	/*================================Begin of collecting POWER TRACE ======================================*/
	for(o=0; o < NoximGlobalParams::mesh_dim_z; o++)
	for(n=0; n < NoximGlobalParams::mesh_dim_y; n++)
	for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
        idx = xyz2Id( m, n, o);
		//router : offset = 0
		overallPowerTrace[3*idx  ] += t[m][n][o]->r->stats.power.getSteadyStateRouterPower();					
        //uP_mem : offset = 1
		overallPowerTrace[3*idx+1] += t[m][n][o]->r->stats.power.getSteadyStateMEMPower   ();
		//uP_mac : offset = 2
		overallPowerTrace[3*idx+2] += t[m][n][o]->r->stats.power.getSteadyStateFPMACPower ();
	}
	/*================================End of COLLECTING Power TRACE=================================================*/
}

/* Recover the full chip temperature (Jimmy added on 2016.03.21)*/
double NoximNoC::TemperatureRecover(int o, int n, int m){
	double recover_temp = 0;
	int idx = xyz2Id( m, n, o);
	
	switch(NoximGlobalParams::temp_recover)
	{
		case 0: // linear combinational temperature reconstruction
		{
			assert(gltable.load(NoximGlobalParams::rector_linear_coef_filename)); //check the table availability
			
			for(int i = (idx * (gstable.sensor_table.size()-1)); i < ((idx * (gstable.sensor_table.size()-1)) + (gstable.sensor_table.size()-1)); i++){ 
				recover_temp += TemperatureTrace[3*(gstable.sensor_table[i%(gstable.sensor_table.size()-1)].location-1)] * gltable.coef_table[i].rector_line_coef;
			}			
			return recover_temp;
			break;
		}
		case 1: // linear regression temperature reconstruction      //Andy added on 2017
		{
			assert(gltable.load(NoximGlobalParams::rector_regression_coef_filename)); //check the table availability
			assert(gmtable.load(NoximGlobalParams::rector_regression_coef1_filename)); //check the table availability
			assert(gntable.load(NoximGlobalParams::rector_regression_ref_filename)); //check the table availability
			
			recover_temp = TemperatureTrace[3*(gntable.sensor_table[idx].location-1)]*gltable.coef_table[idx].rector_line_coef+gmtable.coef_table[idx].rector_line_coef;//y=ax+b
			
			return recover_temp;
			break;
		}
		default: break;
	}
}

void NoximNoC::setTemperature(double in[10][10][10]){
	int m, n, o;
    int idx = 0;
	double current_temp; //current temperature (Jimmy added on 2011.11.10)
	double current_delta_temp; //current delta temperature (Jimmy added on 2011.11.10)
	double pre_delta_temp; //the delta temp. between the current predictive temp. and current prediction temp. (Jimmy added on 2012.06.06)
	double pre_current_temp; //the predictive temperature (Jimmy added on 2012.06.06)
	double adjustment; //the delta temp. between the current predictive temp. and current temp.(Jimmy added on 2012.06.06)
	double after_TBDB_temperture;//Jack add on 2019.05.21
	double model_in[18] = {0};
	double avg_throughput = 0.0;
	double ncomms;
	double total_comms = 0.0;
	double error = 0;//Jack add on 2019.06.29
	
	
	idx = 0;
	for(o=0; o < NoximGlobalParams::mesh_dim_z; o++)
		for(n=0; n < NoximGlobalParams::mesh_dim_y; n++) 
			for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				idx = xyz2Id( m, n, o);
				t[m][n][o]->r->stats.temperature      = TemperatureTrace[3*idx];
			}
	for(o=0; o < NoximGlobalParams::mesh_dim_z; o++)
	for(n=0; n < NoximGlobalParams::mesh_dim_y; n++) 
	for(m=0; m < NoximGlobalParams::mesh_dim_x; m++) {
		idx = xyz2Id( m, n, o);
		
		//set tile REAL temperature
		t[m][n][o]->r->stats.last_temperature = t[m][n][o]->r->stats.temperature;
		//t[m][n][o]->r->stats.temperature      = TemperatureTrace[3*idx];
        t[m][n][o]->r->stats.temperature_s    = -1; //initialize the sensing result table (Jimmy added on 2016.03.19)
		t[m][n][o]->r->stats.temperature_s_r  = -1; //initialize the temperature reconstruction table (Jimmy added on 2016.03.22)
		
		
		/***** Sensing temperature grabing based on different sensor allocation methods (Jimmy added on 2015.05.27) *****/
		if (NoximGlobalParams::sensor_allocate == SENSOR_ALLOCATE_NORMAL) {        //Every NoC node has its own thermal sensor
			t[m][n][o]->r->stats.temperature_s = t[m][n][o]->r->stats.temperature;
		}
		else if (NoximGlobalParams::sensor_allocate == SENSOR_ALLOCATE_EVEN) { //Even NoC nodes hse thermal sensor
			t[m][n][o]->r->stats.temperature_s = ((idx%2)==0)? t[m][n][o]->r->stats.temperature : -1;
		}
		else if (NoximGlobalParams::sensor_allocate == SENSOR_ALLOCATE_ODD){ //Odd NoC nodes hse thermal sensor
			t[m][n][o]->r->stats.temperature_s = ((idx%2)!=0)? t[m][n][o]->r->stats.temperature : -1;
		}
		else if (NoximGlobalParams::sensor_allocate == SENSOR_ALLOCATE_TABLE){ //The locations for sensor placement are defined by user (Jimmy added on 2016.03.19)
			for(int i=0; i<gstable.sensor_table.size()-1; i++ )
			{
				if(gstable.sensor_table[i].location == (idx+1))
				{
					t[m][n][o]->r->stats.temperature_s = t[m][n][o]->r->stats.temperature;
				}
			}
		}
		/*****************************************************************/
		model_in[0] = t[m][n][o]->r->stats.temperature;
		model_in[1] = (m-1>=0 && n-1>=0)?t[m-1][n-1][o]->r->stats.temperature:0;
		model_in[2] = (m-1>=0)?t[m-1][n][o]->r->stats.temperature:0;
		model_in[3] = (m-1>=0 && n+1<NoximGlobalParams::mesh_dim_y)?t[m-1][n+1][o]->r->stats.temperature:0;
		model_in[4] = (n-1>=0)?t[m][n-1][o]->r->stats.temperature:0;
		model_in[5] = (n+1<NoximGlobalParams::mesh_dim_y)?t[m][n+1][o]->r->stats.temperature:0;
		model_in[6] = (m+1<NoximGlobalParams::mesh_dim_x && n-1>=0)?t[m+1][n-1][o]->r->stats.temperature:0;
		model_in[7] = (m+1<NoximGlobalParams::mesh_dim_x)?t[m+1][n][o]->r->stats.temperature:0;
		model_in[8] = (m+1<NoximGlobalParams::mesh_dim_x && n+1<NoximGlobalParams::mesh_dim_y)?t[m+1][n+1][o]->r->stats.temperature:0;
		model_in[9] = throttling[m][n][o];
		model_in[10] = (m-1>=0 && n-1>=0)?throttling[m-1][n-1][o]:0;
		model_in[11] = (m-1>=0)?throttling[m-1][n][o]:0;
		model_in[12] = (m-1>=0 && n+1<NoximGlobalParams::mesh_dim_y)?throttling[m-1][n+1][o]:0;
		model_in[13] = (n-1>=0)?throttling[m][n-1][o]:0;
		model_in[14] = (n+1<NoximGlobalParams::mesh_dim_y)?throttling[m][n+1][o]:0;
		model_in[15] = (m+1<NoximGlobalParams::mesh_dim_x && n-1>=0)?throttling[m+1][n-1][o]:0;
		model_in[16] = (m+1<NoximGlobalParams::mesh_dim_x)?throttling[m+1][n][o]:0;
		model_in[17] = (m+1<NoximGlobalParams::mesh_dim_x && n+1<NoximGlobalParams::mesh_dim_y)?throttling[m+1][n+1][o]:0;
		
		for(int i=9;i<18;i++)
			if(model_in[i] != 0)
				model_in[i] = 1;
		/*if(m==4 && n==4 && o==0){
			for(int i=0;i<18;i++)
				results_log_throttling << model_in[i] << "\t";
			results_log_throttling << endl;
		}
		if(m==4 && n==4 && o==0){
			for(int i=0;i<19;i++)
				results_log_weight << weight[m*8+n][i] << "\t";
			results_log_weight << endl;
		}*/
		/******************* Temperature Prediction (Jimmy modified on 2012.04.16) ***********************/
		double predicted_temp=0;
		if(t[m][n][o]->r->stats.temperature > (INIT_TEMP - 273.15))
		{
			
			current_temp = t[m][n][o]->r->stats.temperature;
			
			current_delta_temp = t[m][n][o]->r->stats.temperature - t[m][n][o]->r->stats.last_temperature;
			/*** Caculate the prediciton error (Jimmy added on 2013.03.06) ***/
			if(t[m][n][o]->r->stats.last_pre_temperature1 == 0) t[m][n][o]->r->stats.predict_error1 = 0;
			else t[m][n][o]->r->stats.predict_error1 = current_temp - t[m][n][o]->r->stats.last_pre_temperature1;
			/*****************************************************************///jack added on 2019.06.05
			double b = 1.98;
			double del_t = 0.01;
			double T;
			if(current_delta_temp < 0){
			/*	t[m][n][o]->r->stats.pre_temperature1 =  t[m][n][o]->r->stats.last_pre_temperature1 + current_delta_temp;
				t[m][n][o]->r->stats.pre_temperature2 =  t[m][n][o]->r->stats.last_pre_temperature2 + current_delta_temp;
				t[m][n][o]->r->stats.pre_temperature3 =  t[m][n][o]->r->stats.last_pre_temperature3 + current_delta_temp;
				t[m][n][o]->r->stats.pre_temperature4 =  t[m][n][o]->r->stats.last_pre_temperature4 + current_delta_temp;
				t[m][n][o]->r->stats.pre_temperature5 =  t[m][n][o]->r->stats.last_pre_temperature5 + current_delta_temp;*/
				
				pre_delta_temp = t[m][n][o]->r->stats.last_pre_temperature1 - t[m][n][o]->r->stats.last_temperature;
				pre_current_temp = t[m][n][o]->r->stats.last_pre_temperature1;
				adjustment = t[m][n][o]->r->stats.last_pre_temperature1 - current_temp;
				
				/*ANN-based temperature prediction Jack add on 2019.07.03*/
				
				//for(int i=0;i<18;i++)
					//cout << model_in[i] << " ";
				for(int index=0;index<18;index++)
					predicted_temp += weight[n*8+m][index]*model_in[index];
				
				t[m][n][o]->r->stats.pre_temperature1 = predicted_temp + weight[n*8+m][18];
										
				//cout << "Decrease" <<"( " << m << "," << n << " )" << "predicted temperature is " << t[m][n][o]->r->stats.last_pre_temperature1 << "Real temperature is " << current_temp << endl;
				//cout << "error = " << (current_temp - t[m][n][o]->r->stats.last_pre_temperature1) << endl << endl;
				
				//T = 0.01*(cycle-1);
				//double cof = ((exp(-b*del_t) - 2*exp(-2*b*del_t) + exp(-3*b*del_t))/(pow(1 - exp(-b*del_t),2)))*del_t - 1/b;
				//double Pc;
				//int idx = xyz2Id( m, n, o);
				//Pc = instPowerTrace[3*idx];
				//t[m][n][o]->r->stats.pre_temperature1 = t[m][n][o]->r->stats.temperature + (cof*(pow(b,2)*((INIT_TEMP - 273.15) - Pc/b)*exp(-b*T))*del_t);
				//t[m][n][o]->r->stats.pre_temperature1 = t[m][n][o]->r->stats.temperature*0.9852081 + t[m][n][o]->r->stats.temperature*(-0.01428994) + Pc*(-0.61) + 3.346132;
				double e_t = (current_temp - t[m][n][o]->r->stats.last_pre_temperature1);
				if (e_t < 0)
					e_t*=-1;
				error += e_t;										
				//t[m][n][o]->r->stats.pre_temperature1 =  pre_current_temp + pre_delta_temp* exp(-1.98*0.01) - adjustment;
				t[m][n][o]->r->stats.pre_temperature2 =  pre_current_temp + pre_delta_temp* exp(-1.98*0.01) + pre_delta_temp* exp(-1.98*0.02) - adjustment;
				t[m][n][o]->r->stats.pre_temperature3 =  pre_current_temp + pre_delta_temp* exp(-1.98*0.01) + pre_delta_temp* exp(-1.98*0.02) +
														 pre_delta_temp* exp(-1.98*0.03) - adjustment;
				t[m][n][o]->r->stats.pre_temperature4 =  pre_current_temp + pre_delta_temp* exp(-1.98*0.01) + pre_delta_temp* exp(-1.98*0.02) +
														 pre_delta_temp* exp(-1.98*0.03) + pre_delta_temp* exp(-1.98*0.04) - adjustment;
				t[m][n][o]->r->stats.pre_temperature5 =  pre_current_temp + pre_delta_temp* exp(-1.98*0.01) + pre_delta_temp* exp(-1.98*0.02) +
														 pre_delta_temp* exp(-1.98*0.03) + pre_delta_temp* exp(-1.98*0.04) + 
														 pre_delta_temp* exp(-1.98*0.05) - adjustment;
				t[m][n][o]->r->stats.pre_temperature6 =  pre_current_temp + pre_delta_temp* exp(-1.98*0.01) + pre_delta_temp* exp(-1.98*0.02) +
														 pre_delta_temp* exp(-1.98*0.03) + pre_delta_temp* exp(-1.98*0.04) + 
														 pre_delta_temp* exp(-1.98*0.05) + pre_delta_temp* exp(-1.98*0.06) - adjustment;														 
			}
			//else t[m][n][o]->r->stats.pre_temperature =  current_temp + current_delta_temp* exp(-2.95*0.01) + current_delta_temp* exp(-2.95*0.02) + current_delta_temp* exp(-2.95*0.03) + current_delta_temp*exp(-2.95*0.04) + current_delta_temp* exp(-2.95*0.05);
			else{
				/*ANN-based temperature prediction Jack add on 2019.07.03*/
				
				if(cycle > 1){//backpropagation LMS
					for(int i=0;i<18;i++){
						weight[n*8+m][i] = weight[n*8+m][i] + 0.000001*(current_temp - t[m][n][o]->r->stats.last_pre_temperature1)*model_in[i];
					}
				}
				
				for(int index=0;index<18;index++)
					predicted_temp += weight[n*8+m][index]*model_in[index];
				t[m][n][o]->r->stats.pre_temperature1 = predicted_temp + weight[n*8+m][18];
				//cout << "Increase" << "( " << m << "," << n << " )" << "predicted temperature is " << t[m][n][o]->r->stats.last_pre_temperature1 << " Real temperature is " << current_temp << endl;
				//cout << "error = " << (current_temp - t[m][n][o]->r->stats.last_pre_temperature1) << endl << endl;
				//T = 0.01*(cycle-1);
				//double cof = ((exp(-b*del_t) - 2*exp(-2*b*del_t) + exp(-3*b*del_t))/(pow(1 - exp(-b*del_t),2)))*del_t - 1/b;
				//double Pc;
				//int idx = xyz2Id( m, n, o);
				//Pc = instPowerTrace[3*idx];
				//t[m][n][o]->r->stats.pre_temperature1 = t[m][n][o]->r->stats.temperature + (cof*(pow(b,2)*((INIT_TEMP - 273.15) - Pc/b)*exp(-b*T))*del_t);
				//t[m][n][o]->r->stats.pre_temperature1 = t[m][n][o]->r->stats.temperature*0.9852081 + t[m][n][o]->r->stats.temperature*(-0.01428994) + Pc*(-0.61) + 3.346132;
				double e_t = (current_temp - t[m][n][o]->r->stats.last_pre_temperature1);
				if (e_t < 0)
					e_t*=-1;
				error += e_t;
			
															
				//t[m][n][o]->r->stats.pre_temperature1 =  current_temp + current_delta_temp* exp(-1.98*0.01);
				t[m][n][o]->r->stats.pre_temperature2 =  current_temp + current_delta_temp* exp(-1.98*0.01) + current_delta_temp* exp(-1.98*0.02);
				t[m][n][o]->r->stats.pre_temperature3 =  current_temp + current_delta_temp* exp(-1.98*0.01) + current_delta_temp* exp(-1.98*0.02) +
														 current_delta_temp* exp(-1.98*0.03);
				t[m][n][o]->r->stats.pre_temperature4 =  current_temp + current_delta_temp* exp(-1.98*0.01) + current_delta_temp* exp(-1.98*0.02) +
														 current_delta_temp* exp(-1.98*0.03) + current_delta_temp* exp(-1.98*0.04);
				t[m][n][o]->r->stats.pre_temperature5 =  current_temp + current_delta_temp* exp(-1.98*0.01) + current_delta_temp* exp(-1.98*0.02) +
														 current_delta_temp* exp(-1.98*0.03) + current_delta_temp* exp(-1.98*0.04) + 
														 current_delta_temp* exp(-1.98*0.05);
				t[m][n][o]->r->stats.pre_temperature6 =  current_temp + current_delta_temp* exp(-1.98*0.01) + current_delta_temp* exp(-1.98*0.02) +
														 current_delta_temp* exp(-1.98*0.03) + current_delta_temp* exp(-1.98*0.04) + 
														 current_delta_temp* exp(-1.98*0.05) + current_delta_temp* exp(-1.98*0.06);														 
			}
			
			
			t[m][n][o]->r->stats.last_pre_temperature1 = t[m][n][o]->r->stats.pre_temperature1;
			t[m][n][o]->r->stats.last_pre_temperature2 = t[m][n][o]->r->stats.pre_temperature2;
			t[m][n][o]->r->stats.last_pre_temperature3 = t[m][n][o]->r->stats.pre_temperature3;
			t[m][n][o]->r->stats.last_pre_temperature4 = t[m][n][o]->r->stats.pre_temperature4;
			t[m][n][o]->r->stats.last_pre_temperature5 = t[m][n][o]->r->stats.pre_temperature5;
			t[m][n][o]->r->stats.last_pre_temperature6 = t[m][n][o]->r->stats.pre_temperature6;			
		}
		else
		{
			t[m][n][o]->r->stats.pre_temperature1 = t[m][n][o]->r->stats.temperature;
			t[m][n][o]->r->stats.pre_temperature2 = t[m][n][o]->r->stats.temperature;
			t[m][n][o]->r->stats.pre_temperature3 = t[m][n][o]->r->stats.temperature;
			t[m][n][o]->r->stats.pre_temperature4 = t[m][n][o]->r->stats.temperature;
			t[m][n][o]->r->stats.pre_temperature5 = t[m][n][o]->r->stats.temperature;
			t[m][n][o]->r->stats.pre_temperature6 = t[m][n][o]->r->stats.temperature;			
		}
		
		results_log_temp0 << t[m][n][o]->r->stats.temperature     <<"\t";
		results_log_temp1 << t[m][n][o]->r->stats.pre_temperature1<<"\t";
		results_log_temp2 << t[m][n][o]->r->stats.pre_temperature2<<"\t";
		results_log_temp3 << t[m][n][o]->r->stats.pre_temperature3<<"\t";
		results_log_temp4 << t[m][n][o]->r->stats.pre_temperature4<<"\t";
		results_log_temp5 << t[m][n][o]->r->stats.pre_temperature5<<"\t";
		results_log_temp6 << t[m][n][o]->r->stats.pre_temperature6<<"\t";
		
		results_log_prederr0 << 0     <<"\t";
		results_log_prederr1 << t[m][n][o]->r->stats.predict_error1<<"\t";
		//results_log_predpara2 << t[m][n][o]->r->stats.pre_temperature2<<"\t";
		//results_log_predpara3 << t[m][n][o]->r->stats.pre_temperature3<<"\t";
		//results_log_predpara4 << t[m][n][o]->r->stats.pre_temperature4<<"\t";
		//results_log_predpara5 << t[m][n][o]->r->stats.pre_temperature5<<"\t";
		//results_log_predpara6 << t[m][n][o]->r->stats.pre_temperature6<<"\t";
		/*************************************************************************************************/	
	}
	/**Q-learning**/
	for(o=0; o < NoximGlobalParams::mesh_dim_z; o++)
	for(n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
		for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
			//idx = xyz2Id( m, n, o);
			//t[m][n][o]->r->stats.temperature = TemperatureTrace[3*idx];
			if(o == 0)
				in[m][n][4] = t[m][n][o]->r->stats.pre_temperature1;
		}
	}
	
	
	if(terminal == 1){
		wo_eva.open("NN_weight_eva.txt",ios::out);
		wo_tar.open("NN_weight_tar.txt",ios::out);
		for(n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
			for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				for(int i=0;i<3;i++)
					for(int j=0;j<7;j++){
						wo_eva << eva_L1_w[m][n][j][i] << " ";
						wo_tar << tar_L1_w[m][n][j][i] << " ";
					}
				wo_eva << "\n";
				wo_tar << "\n";
			}
		}
		wo_eva.close();
		wo_tar.close();
		/*dout.open("DQN_D.txt",ios::out);
		dout.setf(ios::fixed);
		dout<<setprecision(6);
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				D[m][n][count-1][22] *= -1;
				for(int i=0;i<D_MAX;i++){
					for(int idx=0;idx<6;++idx)//S_t
						dout << ' ' << setw(16) << D[m][n][i][idx];
					for(int idx=0;idx<6;++idx)//S_t+1
						dout << ' ' << setw(16) << D[m][n][i][idx+9];
					dout << ' ' << setw(16) << D[m][n][i][20];//r_t
					dout << ' ' << setw(16) << D[m][n][i][21];//a_t
					dout << ' ' << setw(16) << D[m][n][i][22];//cycle
					dout << "\n";
				}
				dout << "\n";
			}
		}
		dout << count;
		dout.close();*/
		dout.open("QN_D.txt",ios::out);
		dout.setf(ios::fixed);
		dout<<setprecision(6);
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				D[m][n][count-1][22] *= -1;
				for(int i=0;i<D_MAX;i++){
					for(int idx=0;idx<6;++idx)//S_t
						dout << ' ' << setw(16) << D[m][n][i][idx];
					for(int idx=0;idx<6;++idx)//S_t+1
						dout << ' ' << setw(16) << D[m][n][i][idx+9];
					dout << ' ' << setw(16) << D[m][n][i][20];//r_t
					dout << ' ' << setw(16) << D[m][n][i][21];//a_t
					dout << ' ' << setw(16) << D[m][n][i][22];//cycle
					dout << "\n";
				}
				dout << "\n";
				
			}
		}
		dout << count;
		dout.close();
		if(NoximGlobalParams::DQN_init)
			rew_out.open("../reward.txt",ios::out);
		else{
			rew_out.open("../reward.txt",ios::app);
			reward_log_csv.open("../reward.csv",ios::app);
		}
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				//cout << "Average reward : " << reward[m][n] << endl;
				rew_out << setw(12) << reward[m][n] << " ";
				reward_log_csv << reward[m][n];
			}
			reward_log_csv << "\n";
		}
		rew_out << "\n";
		rew_out.close();
		qt_out.open("Q_table.txt",ios::out);
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				double temp = 0.0;
				for(int i=0;i<5;i++){
					if(i == 4)
						qt_out<< setprecision(3) << 100 <<setw(12);
					else
						qt_out<< setprecision(3) << temp+NoximGlobalParams::temp_trigger <<setw(12);
					 temp += 0.3;
					for(int j=0;j<3;j++)
						qt_out << setw(12) << Q_table[m][n][i][j];
					qt_out<<endl;
				}
			}
		}
		qt_out.close();
		number += 1;
		number_out.open("number.txt",ios::out);
		number_out << number;
		number_out.close();
		sc_stop();
	}
	
	int tquota = 0;
	for(o=0; o < NoximGlobalParams::mesh_dim_z; o++){
		for(n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
			for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				tquota += t[m][n][0]->pe->t_quota;
			}
		}
	}
	
	/*if(total_transmit == 0&& tquota == 0 && cycle>1){
		terminal = 1;
	}*/
	
	cout << "Current cycle : " << cycle << endl;
	for(n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
		for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
			D[m][n][count][22] = cycle;
			int ridx = n*NoximGlobalParams::mesh_dim_x+m;
			double now_temp = HS_interface->tempRec(ridx,0);
			double last_quota = D[m][n][(count-1)%D_MAX][5];
			double now_quota = ((double)t[m][n][0]->pe->t_quota)/NoximGlobalParams::static_quota;
		
			if(now_temp >= 100){
				terminal = 1;
				D[m][n][count%D_MAX][20] = -1.0;
				R[10][10] = -1.0;
			}
			/*else if(last_quota>0 && now_quota==0){
				D[m][n][count%D_MAX][20] = 1.0;
				R[10][10] = 1.0;
			}*/
			else if(cycle == (NoximGlobalParams::simulation_time/TEMP_REPORT_PERIOD) - 1){
				ncomms = t[m][n][0]->r->stats.getTotalCommunications();
				D[m][n][count%D_MAX][20] = ncomms * t[m][n][0]->r->stats.getAverageThroughput()/ncomms;
				R[10][10] = ncomms * t[m][n][0]->r->stats.getAverageThroughput()/ncomms;
				if(D[m][n][count%D_MAX][20]!=0)
					reward[m][n] = D[m][n][count%D_MAX][20];
				cout << "Average reward : " << reward[m][n] << endl;
			}
			else{
				D[m][n][count%D_MAX][20] = 0;
				R[10][10] = 0;
			}
			if(D[m][n][count%D_MAX][20]!=0)
				reward[m][n] = D[m][n][count%D_MAX][20];
			cout << "Average reward : " << reward[m][n] << endl;
		}
	}
	if(count>0)
		update_table(in);
	choose_act(in);
	int count_flag = 0;
	for(n=0; n < NoximGlobalParams::mesh_dim_y; n++)
		for(m=0; m < NoximGlobalParams::mesh_dim_x; m++)
			if(in[m][n][4] > NoximGlobalParams::temp_trigger)
				count_flag = 1;
	if(count_flag == 1)
		count ++;
	cycle+=1;
	
	if(count%200 == 0)
	{
		for(n=0; n < NoximGlobalParams::mesh_dim_y; n++){ 
			for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				for(int i=0;i<3;i++)
					for(int j=0;j<7;j++){
						tar_L1_w[m][n][j][i] = eva_L1_w[m][n][j][i];
					}
			}
		}
	}
	if(cycle == (NoximGlobalParams::simulation_time/TEMP_REPORT_PERIOD) ){
		terminal = 1;
		
	}
	/** Recontruct the full-chip temperature (Jimmy added on 2016.03.24) **/
	for(o=0; o < NoximGlobalParams::mesh_dim_z; o++)
	for(n=0; n < NoximGlobalParams::mesh_dim_y; n++) 
	for(m=0; m < NoximGlobalParams::mesh_dim_x; m++){
		if(NoximGlobalParams::sensor_allocate == SENSOR_ALLOCATE_NORMAL){ // in normal sensor placement case, we don't need to do temp reconstruction
			t[m][n][o]->r->stats.temperature_s_r = t[m][n][o]->r->stats.temperature_s;
			results_log_temp0_s_r << t[m][n][o]->r->stats.temperature_s_r     <<"\t";
		}
		else{
			if(o == 0){ // %%%%%%%%%%% Recently, we onlt focus on the Layer 0's temperature to simulate a 2D NoC (will be deleted as exten to 3D NoC!!!!!!)
				t[m][n][o]->r->stats.temperature_s_r = TemperatureRecover(o, n, m);
				results_log_temp0_s_r << t[m][n][o]->r->stats.temperature_s_r     <<"\t"; 
			}
			else{
				t[m][n][o]->r->stats.temperature_s_r = -1;
				results_log_temp0_s_r << t[m][n][o]->r->stats.temperature_s_r    <<"\t"; 
			} 
		}	
	}
	
	results_log_temp0 << endl;
	results_log_temp1 << endl;
	results_log_temp2 << endl;
	results_log_temp3 << endl;
	results_log_temp4 << endl;
	results_log_temp5 << endl;
	results_log_temp6 << endl;
	
	results_log_prederr0 << endl;
	results_log_prederr1 << endl;
	results_log_prederr2 << endl;
	results_log_prederr3 << endl;
	results_log_prederr4 << endl;
	results_log_prederr5 << endl;
	results_log_prederr6 << endl;
	
	results_log_temp0_s   << endl;
	results_log_temp0_s_r << endl;
}

bool NoximNoC::EmergencyDecision()
{	
	bool isEmergency = false;
	int emergency_dist = -1; //the distance of thermal emregency
	

	//for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
	//for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
	//for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++)
				//throttling[x][y][z] = 0;     

	if(NoximGlobalParams::throt_type == THROT_GLOBAL){ //Jimmy modified the proactive DTM_GT on 2012.11.3
		for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
		for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
		for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
			if(t[x][y][z]->r->stats.temperature > NoximGlobalParams::temp_trigger){ // each temperature of routers exceed temperature threshould
				isEmergency = true;
				emergency_dist = 0;
				break;
			}
			else if(t[x][y][z]->r->stats.pre_temperature1 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 1){
				if(!isEmergency) isEmergency = false;
				//if(emergency_dist==-1 || emergency_dist >=1) emergency_dist = 1;
				/** Jimmy added on 2013.03.06 **/
				emergency_dist = (NoximGlobalParams::dfs_level > 1) ? 1 : (NoximGlobalParams::dfs_level - 1);
			}
			else if(t[x][y][z]->r->stats.pre_temperature2 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 2){
				if(!isEmergency) isEmergency = false;
				//if(emergency_dist==-1 || emergency_dist >=2) emergency_dist = 2;
				//emergency_dist = 2;
				/** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 2) ? 2 : (NoximGlobalParams::dfs_level - 1);				
			}
			else if(t[x][y][z]->r->stats.pre_temperature3 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 3){
				if(!isEmergency) isEmergency = false;
				//if(emergency_dist==-1 || emergency_dist >=3) emergency_dist = 3; 
				//emergency_dist = 3;
                /** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 3) ? 3 : (NoximGlobalParams::dfs_level - 1);				
			}
			else if(t[x][y][z]->r->stats.pre_temperature4 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 4){
				if(!isEmergency) isEmergency = false;
				//if(emergency_dist==-1 || emergency_dist >=4) emergency_dist = 4;
				//emergency_dist = 4;
				/** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 4) ? 4 : (NoximGlobalParams::dfs_level - 1);
			}
			else if(t[x][y][z]->r->stats.pre_temperature5 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 5){
				if(!isEmergency) isEmergency = false;
				//if(emergency_dist==-1 || emergency_dist >=5) emergency_dist = 5;
				//emergency_dist = 5;
                /** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 5) ? 5 : (NoximGlobalParams::dfs_level - 1);				
			}
		}
		
		if(isEmergency ){
		    for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
		    for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
		    for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
					t[x][y][z]->pe->IntoEmergency();
					t[x][y][z]->r ->IntoEmergency();
					throttling[x][y][z] = 1;
		    }
		}
		else if(!isEmergency && emergency_dist!=-1 ){
			for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
		    for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
		    for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
				throttling[x][y][z] = emergency_dist + 1;
				t[x][y][z]->pe->IntoSemiEmergency(throttling[x][y][z]);
				t[x][y][z]->r ->IntoSemiEmergency(throttling[x][y][z]);
			}
		}
		else{
			for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
		    for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
		    for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
		    	t[x][y][z]->pe->OutOfEmergency();
		    	t[x][y][z]->r ->OutOfEmergency();
		    	throttling[x][y][z] = 0; 
		    }
		}
	}
	else if(NoximGlobalParams::throt_type == THROT_DISTRIBUTED){ //Jimmy add the proactive DTM_GT on 2012.07.15
		//for(int z=0; z < NoximGlobalParams::mesh_dim_z - 1 ; z++)
		for(int z=0; z < NoximGlobalParams::mesh_dim_z     ; z++)
		for(int y=0; y < NoximGlobalParams::mesh_dim_y     ; y++) 
		for(int x=0; x < NoximGlobalParams::mesh_dim_x     ; x++){
			if(t[x][y][z]->r->stats.temperature > NoximGlobalParams::temp_trigger){	
				isEmergency = true;
				t[x][y][z]->pe->IntoEmergency();
				t[x][y][z]->r ->IntoEmergency();
				throttling[x][y][z] = 1; 
			}
			else if(t[x][y][z]->r->stats.pre_temperature1 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 1){ //Jimmy added on 2012.04.12
				isEmergency = false;
				/** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 1) ? 1 : (NoximGlobalParams::dfs_level - 1);
				//throttling[x][y][z] = 2;
				throttling[x][y][z] = emergency_dist + 1;
				/******************************/
				t[x][y][z]->pe->IntoSemiEmergency(throttling[x][y][z]);
				t[x][y][z]->r ->IntoSemiEmergency(throttling[x][y][z]);
			}
			else if(t[x][y][z]->r->stats.pre_temperature2 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 2){ //Jimmy added on 2012.04.12
				isEmergency = false;
				/** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 2) ? 2 : (NoximGlobalParams::dfs_level - 1);
				//throttling[x][y][z] = 3;
				throttling[x][y][z] = emergency_dist + 1;
				/******************************/
				t[x][y][z]->pe->IntoSemiEmergency(throttling[x][y][z]);
				t[x][y][z]->r ->IntoSemiEmergency(throttling[x][y][z]);
			}
			else if(t[x][y][z]->r->stats.pre_temperature3 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 3){ //Jimmy added on 2012.04.12
				isEmergency = false;
				/** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 3) ? 3 : (NoximGlobalParams::dfs_level - 1);
				//throttling[x][y][z] = 4;
				throttling[x][y][z] = emergency_dist + 1;
				/******************************/
				t[x][y][z]->pe->IntoSemiEmergency(throttling[x][y][z]);
				t[x][y][z]->r ->IntoSemiEmergency(throttling[x][y][z]);
			}
			else if(t[x][y][z]->r->stats.pre_temperature4 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 4){ //Jimmy added on 2012.04.12
				isEmergency = false;
				/** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 4) ? 4 : (NoximGlobalParams::dfs_level - 1);
				//throttling[x][y][z] = 5;
				throttling[x][y][z] = emergency_dist + 1;
				/******************************/
				t[x][y][z]->pe->IntoSemiEmergency(throttling[x][y][z]);
				t[x][y][z]->r ->IntoSemiEmergency(throttling[x][y][z]);
			}
			else if(t[x][y][z]->r->stats.pre_temperature5 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 5){ //Jimmy added on 2012.04.12
				isEmergency = false;
				/** Jimmy added on 2013.03.06 **/
                emergency_dist = (NoximGlobalParams::dfs_level > 5) ? 5 : (NoximGlobalParams::dfs_level - 1);
				//throttling[x][y][z] = 6;
				throttling[x][y][z] = emergency_dist + 1;
				/******************************/
				t[x][y][z]->pe->IntoSemiEmergency(throttling[x][y][z]);
				t[x][y][z]->r ->IntoSemiEmergency(throttling[x][y][z]);
			}
			else{	
				t[x][y][z]->pe->OutOfEmergency();	
				t[x][y][z]->r ->OutOfEmergency();
				throttling[x][y][z] = 0; 
			}
		}
	}
	else if(NoximGlobalParams::throt_type == THROT_VERTICAL){
		
		for(int z=0; z < NoximGlobalParams::mesh_dim_z ; z++ )//from top to down, and bottom layer never be throttled.
		//for(int z=0; z < NoximGlobalParams::mesh_dim_z     ; z++ )
			for(int y=0; y < NoximGlobalParams::mesh_dim_y     ; y++ ) 
				for(int x=0; x < NoximGlobalParams::mesh_dim_x     ; x++ ) {
					isEmergency = false;
					//for( int zz = 0 ; zz < z ; zz++)
					//cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.1 = "<<t[x][y][z]->r->stats.pre_temperature1<<" )"<<endl;
					if(NoximGlobalParams::DQN){
						if(t[x][y][z]->r->node_no !=0){
							//cout<<"t["<<x<<"]["<<y<<"]["<<z << "]"<< endl ;
							for( int zz = z ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
								//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
													//t[x][y][zz]->pe->IntoEmergency();
													//t[x][y][zz]->r ->IntoEmergency();
													//throttling[x][y][zz]       = 1;
								if(zz == z || zz <NoximGlobalParams::mesh_dim_z-1)
									if (throttling[x][y][zz]==0){
										/** Jimmy added on 2013.03.06 **/
										//emergency_dist = (NoximGlobalParams::dfs_level > 1) ? 1 : (NoximGlobalParams::dfs_level - 1);
										if(t[x][y][0]->r->node_no == 0)
											emergency_dist = 0;
										else if(t[x][y][0]->r->node_no == 1)
											emergency_dist = 2;
										else if(t[x][y][0]->r->node_no == 2)
											emergency_dist = 1;
										//throttling[x][y][zz] = 3;
										throttling[x][y][zz] = emergency_dist;
										//cout << throttling[x][y][zz] << endl;
										/******************************/
										if(throttling[x][y][zz] == 1){
											t[x][y][zz]->pe->IntoEmergency();
											t[x][y][zz]->r ->IntoEmergency();
											//cout<<"Enter Overheat"<<endl;
										}
										else if(throttling[x][y][zz] == 2){
											t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
											t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
											//cout<<"Enter Partial Throttle 1/2"<<endl;
										}
									}
							}
						}
					}
					else
					if(t[x][y][z]->r->stats.temperature > NoximGlobalParams::temp_trigger){ // >TEMP_THRESHOLD
						isEmergency = true;
						//for( int zz = 0 ; zz < z + 1 ; zz++)
						//cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" )"<<endl;
						double p = rand() / (RAND_MAX + 1.0);
						
						for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
							if(zz < NoximGlobalParams::mesh_dim_z){	//Bottom chip layer won't be throttle
								if(p<0.5)
								{
									throttling[x][y][zz]       = 2;
									t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
									t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
								}
								else{
									t[x][y][zz]->pe->IntoEmergency();
									t[x][y][zz]->r ->IntoEmergency();
									throttling[x][y][zz]       = 1;
								}
								
									/** Jimmy added on 2013.03.06 **/
								/*	t[x][y][zz]->pe->IntoEmergency();
									t[x][y][zz]->r ->IntoEmergency();
									throttling[x][y][zz]       = 1;
								*/
								
							}
						}
						//cout<<"Enter Overheat"<<endl;
										//NoximGlobalParams::temp_trigger = NoximGlobalParams::temp_trigger + 3;
					}
					else if(t[x][y][z]->r->stats.pre_temperature1 > NoximGlobalParams::temp_trigger &&
							NoximGlobalParams::predict_vol >= 1){ //Jimmy added on 2012.04.12
						isEmergency = false;
						//for( int zz = 0 ; zz < z ; zz++)
						cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.1 = "<<t[x][y][z]->r->stats.pre_temperature1<<" )"<<endl;
						for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
							//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
												//t[x][y][zz]->pe->IntoEmergency();
												//t[x][y][zz]->r ->IntoEmergency();
												//throttling[x][y][zz]       = 1;
							if (throttling[x][y][zz]==0){
								/** Jimmy added on 2013.03.06 **/
								emergency_dist = (NoximGlobalParams::dfs_level > 1) ? 1 : (NoximGlobalParams::dfs_level - 1);
								//emergency_dist = t[x][y][zz]->r->node_no;
								//throttling[x][y][zz] = 3;
								throttling[x][y][zz] = emergency_dist + 1;
								/******************************/
								t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
								t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
							}
						}
						cout<<"Enter Partial Throttle 1/2"<<endl;
					}
					else if(t[x][y][z]->r->stats.pre_temperature2 > NoximGlobalParams::temp_trigger &&
							NoximGlobalParams::predict_vol >= 2){ //Jimmy added on 2012.04.12
						isEmergency = false;
						//for( int zz = 0 ; zz < z ; zz++)
						cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.2 = "<<t[x][y][z]->r->stats.pre_temperature2<<" )"<<endl;
						for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
							//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
							if (throttling[x][y][zz]==0){
								/** Jimmy added on 2013.03.06 **/
								emergency_dist = (NoximGlobalParams::dfs_level > 2) ? 2 : (NoximGlobalParams::dfs_level - 1);
								//throttling[x][y][zz] = 3;
								throttling[x][y][zz] = emergency_dist + 1;
								/******************************/
								t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
								t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
							}
												//t[x][y][zz]->pe->IntoEmergency();
												//t[x][y][zz]->r ->IntoEmergency();
												//throttling[x][y][zz]       = 1;
						}
						cout<<"Enter Partial Throttle 1/3"<<endl;
					}
					else if(t[x][y][z]->r->stats.pre_temperature3 > NoximGlobalParams::temp_trigger &&
							NoximGlobalParams::predict_vol >= 3){ //Jimmy added on 2012.04.12
						isEmergency = false;
						//for( int zz = 0 ; zz < z ; zz++)
						cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.3 = "<<t[x][y][z]->r->stats.pre_temperature3<<" )"<<endl;
						for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
							//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
							if (throttling[x][y][zz]==0){
								throttling[x][y][zz]       = 4;
								/** Jimmy added on 2013.03.06 **/
								emergency_dist = (NoximGlobalParams::dfs_level > 3) ? 3 : (NoximGlobalParams::dfs_level - 1);
								//throttling[x][y][zz] = 4;
								throttling[x][y][zz] = emergency_dist + 1;
								/******************************/
								t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
								t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
							}
												//t[x][y][zz]->pe->IntoEmergency();
												//t[x][y][zz]->r ->IntoEmergency();
												//throttling[x][y][zz]       = 1;
						}
						cout<<"Enter Partial Throttle 1/4"<<endl;
					}
					else if(t[x][y][z]->r->stats.pre_temperature4 > NoximGlobalParams::temp_trigger &&
							NoximGlobalParams::predict_vol >= 4){ //Jimmy added on 2012.04.12
						isEmergency = false;
						//for( int zz = 0 ; zz < z ; zz++)
						cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.4 = "<<t[x][y][z]->r->stats.pre_temperature4<<" )"<<endl;
						for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
							//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
							if (throttling[x][y][zz]==0){
								/** Jimmy added on 2013.03.06 **/
								emergency_dist = (NoximGlobalParams::dfs_level > 4) ? 1 : (NoximGlobalParams::dfs_level - 1);
								//throttling[x][y][zz] = 5;
								throttling[x][y][zz] = emergency_dist + 1;
								/******************************/
								t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
								t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
							}
												//t[x][y][zz]->pe->IntoEmergency();
												//t[x][y][zz]->r ->IntoEmergency();
												//throttling[x][y][zz]       = 1;
						}
						cout<<"Enter Partial Throttle 1/5"<<endl;
					}
					else if(t[x][y][z]->r->stats.pre_temperature5 > NoximGlobalParams::temp_trigger &&
							NoximGlobalParams::predict_vol >= 5){ //Jimmy added on 2012.04.12
						isEmergency = false;
						//for( int zz = 0 ; zz < z ; zz++)
						cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.5 = "<<t[x][y][z]->r->stats.pre_temperature5<<" )"<<endl;
						for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
							//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
							if (throttling[x][y][zz]==0){
								throttling[x][y][zz]       = 6;
								/** Jimmy added on 2013.03.06 **/
								emergency_dist = (NoximGlobalParams::dfs_level > 5) ? 5 : (NoximGlobalParams::dfs_level - 1);
								//throttling[x][y][zz] = 6;
								throttling[x][y][zz] = emergency_dist + 1;
								/******************************/
								t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
								t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
							}
												//t[x][y][zz]->pe->IntoEmergency();
												//t[x][y][zz]->r ->IntoEmergency();
												//throttling[x][y][zz]       = 1;
						}
						cout<<"Enter Partial Throttle 1/6"<<endl;
					}
					else{
						
						if (throttling[x][y][z] == 0) {	
						
							//set OutOfEmergency
							t[x][y][z]->pe->OutOfEmergency();		
							t[x][y][z]->r ->OutOfEmergency();
							
							//set buffersize
							/*
							if ( t[x][y][z]->r->_emergency_trigger_flag ) {
								for (int i = 0; i < DIRECTIONS + 1; i++) {
									printf("Router id: %d ---- >buffer[%d]'s previous-buffer size: %d, ", t[x][y][z]->r->local_id, i, t[x][y][z]->r ->buffer[i].GetMaxBufferSize());
									t[x][y][z]->r ->buffer[i].SetMaxBufferSize(t[x][y][z]->r ->_man_set_max_buffer_size);
									printf("now: %d\n",t[x][y][z]->r ->buffer[i].GetMaxBufferSize());
								}
								
								t[x][y][z]->r->_emergency_trigger_flag = false;
							}*/
							
							
							throttling[x][y][z]             = 0;
						}
						//cout<<"Enter Non-Overheat"<<endl;
					}
			//int non_beltway_layer,non_throt_layer;
			//int col_max,col_min,row_max,row_min;
			//findNonXLayer(non_throt_layer,non_beltway_layer);
			//calROC(col_max,col_min,row_max,row_min,non_beltway_layer);
			//for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++ )
			//for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++ ) 
			//for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++ )
			//	t[x][y][z]->pe->RTM_set_var(non_throt_layer, non_beltway_layer, col_max, col_min, row_max, row_min);
			//cout<<"This is for testing, need to be remove after verify"<<endl;
			//cout<<"non_beltway_layer,non_throt_layer = "<<non_beltway_layer<<","<<non_throt_layer<<endl;
			//cout<<"col_max,col_min,row_max,row_min = "<<col_max<<","<<col_min<<","<<row_max<<","<<row_min<<endl;
			
		}
	}
	else if(NoximGlobalParams::throt_type == THROT_TAVT){
		//for(int z=0; z < NoximGlobalParams::mesh_dim_z - 1 ; z++ )//from top to down, and bottom layer never be throttled.
		for(int z=0; z < NoximGlobalParams::mesh_dim_z     ; z++ )
		for(int y=0; y < NoximGlobalParams::mesh_dim_y     ; y++ ) 
		for(int x=0; x < NoximGlobalParams::mesh_dim_x     ; x++ ){
			// if (t[x][y][z]->r->stats.temperature < TEMP_THRESHOLD && 
			    // t[x][y][z]->r->stats.pre_temperature < TEMP_THRESHOLD &&
				// throttling[x][y][z] == 0){	
				// t[x][y][z]->pe->OutOfEmergency();		
				// t[x][y][z]->r ->OutOfEmergency();
				// throttling[x][y][z]             = 0; 
				// /*if( t[x][y][z]->r->stats.temperature > NoximGlobalParams::beltway_trigger && NoximGlobalParams::beltway )beltway[x][y][z]       = true;
				// else                                                                					beltway[x][y][z]       = false;*/
			// }
			if(t[x][y][z]->r->stats.temperature > NoximGlobalParams::temp_trigger){ // >TEMP_THRESHOLD
				isEmergency = true;
				for( int zz = 0 ; zz < z + 1 ; zz++)
				//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
				if(zz < NoximGlobalParams::mesh_dim_z){
					t[x][y][zz]->pe->IntoEmergency();
					t[x][y][zz]->r ->IntoEmergency();
					throttling[x][y][zz]       = 1;
				}
			}
			else if(t[x][y][z]->r->stats.pre_temperature1 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 1){ //Jimmy added on 2012.04.14
				isEmergency = false;
				for( int zz = 0 ; zz < z + 1 ; zz++)
				//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
				if(zz < NoximGlobalParams::mesh_dim_z && throttling[x][y][zz]!=1){
					throttling[x][y][zz]       = 2;
					t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
					t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
				}
			}
			else if(t[x][y][z]->r->stats.pre_temperature2 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 2){ //Jimmy added on 2012.04.14
				isEmergency = false;
				for( int zz = 0 ; zz < z + 1 ; zz++)
				//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
				if(zz < NoximGlobalParams::mesh_dim_z && throttling[x][y][zz]!=1 && throttling[x][y][zz]!=2){
					throttling[x][y][zz]       = 3;
					t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
					t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
				}
			}
			else if(t[x][y][z]->r->stats.pre_temperature3 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 3){ //Jimmy added on 2012.04.12
				isEmergency = false;
				//for( int zz = 0 ; zz < z ; zz++)
				cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.3 = "<<t[x][y][z]->r->stats.pre_temperature3<<" )"<<endl;
				//for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
				for( int zz = 0 ; zz < z + 1 ; zz++){
					//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
					if (throttling[x][y][zz]==0){
						throttling[x][y][zz]       = 4;
						t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
						t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
					}
				}
				cout<<"Enter Partial Throttle 1/4"<<endl;
			}
			else if(t[x][y][z]->r->stats.pre_temperature4 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 4){ //Jimmy added on 2012.04.12
				isEmergency = false;
				//for( int zz = 0 ; zz < z ; zz++)
				cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.4 = "<<t[x][y][z]->r->stats.pre_temperature4<<" )"<<endl;
				//for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
				for( int zz = 0 ; zz < z + 1 ; zz++){
					//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
					if (throttling[x][y][zz]==0){
						throttling[x][y][zz]       = 5;
						t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
						t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
					}
				}
				cout<<"Enter Partial Throttle 1/5"<<endl;
			}
			else if(t[x][y][z]->r->stats.pre_temperature5 > NoximGlobalParams::temp_trigger &&
			        NoximGlobalParams::predict_vol >= 5){ //Jimmy added on 2012.04.12
				isEmergency = false;
				//for( int zz = 0 ; zz < z ; zz++)
				cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Semi-Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" Prediction Temp.5 = "<<t[x][y][z]->r->stats.pre_temperature5<<" )"<<endl;
				//for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
				for( int zz = 0 ; zz < z + 1 ; zz++){
					//if(zz < NoximGlobalParams::mesh_dim_z-1){	//Bottom chip layer won't be throttle
					if (throttling[x][y][zz]==0){
						throttling[x][y][zz]       = 6;
						t[x][y][zz]->pe->IntoSemiEmergency(throttling[x][y][zz]);
						t[x][y][zz]->r ->IntoSemiEmergency(throttling[x][y][zz]);
					}
				}
				cout<<"Enter Partial Throttle 1/6"<<endl;
			}
			else{
				if (throttling[x][y][z] == 0){	
					t[x][y][z]->pe->OutOfEmergency();		
					t[x][y][z]->r ->OutOfEmergency();
					throttling[x][y][z]             = 0;
				}
			}
			/*
			int non_beltway_layer,non_throt_layer;
			int col_max,col_min,row_max,row_min;
			findNonXLayer(non_throt_layer,non_beltway_layer);
			calROC(col_max,col_min,row_max,row_min,non_beltway_layer);
			for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++ )
			for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++ ) 
			for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++ )
				t[x][y][z]->pe->RTM_set_var(non_throt_layer, non_beltway_layer, col_max, col_min, row_max, row_min);
			cout<<"This is for testing, need to be remove after verify"<<endl;
			cout<<"non_beltway_layer,non_throt_layer = "<<non_beltway_layer<<","<<non_throt_layer<<endl;
			cout<<"col_max,col_min,row_max,row_min = "<<col_max<<","<<col_min<<","<<row_max<<","<<row_min<<endl;
			*/
		}
	}
	else return isEmergency;//THROT_NORMAL,THROT_TEST do nothing, because the topology won't change
	return isEmergency;
}

bool NoximNoC::EmergencyDecision_sen() // the decision is made by considering the reconstruction temperature information (Jimmy added on 2016.03.24)
{	
	bool isEmergency = false;
	//int emergency_dist = -1; //the distance of thermal emregency

	for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
	for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
	for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++)
				throttling[x][y][z] = 0;     

	if(NoximGlobalParams::throt_type == THROT_GLOBAL){ 
		for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
		for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
		for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
			if(t[x][y][z]->r->stats.temperature_s_r > NoximGlobalParams::temp_trigger){ // each temperature of routers exceed temperature threshould
				isEmergency = true;
				//emergency_dist = 0;
				break;
			}
		
			if(isEmergency){
				for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
				for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
				for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
						t[x][y][z]->pe->IntoEmergency();
						t[x][y][z]->r ->IntoEmergency();
						throttling[x][y][z] = 1;
				}
			}
			else{
				for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++) 
				for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++) 
				for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
					t[x][y][z]->pe->OutOfEmergency();
					t[x][y][z]->r ->OutOfEmergency();
					throttling[x][y][z] = 0; 
				}
			}
		}
	}
	else if(NoximGlobalParams::throt_type == THROT_DISTRIBUTED){ 
		//for(int z=0; z < NoximGlobalParams::mesh_dim_z - 1 ; z++)
		for(int z=0; z < NoximGlobalParams::mesh_dim_z     ; z++)
		for(int y=0; y < NoximGlobalParams::mesh_dim_y     ; y++) 
		for(int x=0; x < NoximGlobalParams::mesh_dim_x     ; x++){
			if(t[x][y][z]->r->stats.temperature_s_r > NoximGlobalParams::temp_trigger){	
				isEmergency = true;
				t[x][y][z]->pe->IntoEmergency();
				t[x][y][z]->r ->IntoEmergency();
				throttling[x][y][z] = 1; 
			}
			else{	
				t[x][y][z]->pe->OutOfEmergency();	
				t[x][y][z]->r ->OutOfEmergency();
				throttling[x][y][z] = 0; 
			}
		}
	}
	else if(NoximGlobalParams::throt_type == THROT_VERTICAL){
		//for(int z=0; z < NoximGlobalParams::mesh_dim_z - 1 ; z++ )//from top to down, and bottom layer never be throttled.
		for(int z=0; z < NoximGlobalParams::mesh_dim_z     ; z++ )
		for(int y=0; y < NoximGlobalParams::mesh_dim_y     ; y++ ) 
		for(int x=0; x < NoximGlobalParams::mesh_dim_x     ; x++ ){
			if(t[x][y][z]->r->stats.temperature_s_r > NoximGlobalParams::temp_trigger){ // >TEMP_THRESHOLD
				isEmergency = true;
				cout<<"t["<<x<<"]["<<y<<"]["<<z<<"] ""Go into Emergency (Current Temp. ="<<t[x][y][z]->r->stats.temperature<<" )"<<endl;
				for( int zz = 0 ; zz < NoximGlobalParams::mesh_dim_z ; zz++){
						t[x][y][zz]->pe->IntoEmergency();
						t[x][y][zz]->r ->IntoEmergency();
						throttling[x][y][zz]       = 1;
				}
				cout<<"Enter Overheat"<<endl;
			}
			else{
				if (throttling[x][y][z] == 0){	
					t[x][y][z]->pe->OutOfEmergency();		
					t[x][y][z]->r ->OutOfEmergency();
					throttling[x][y][z]             = 0;
				}
			}
			
			//int non_beltway_layer,non_throt_layer;
			//int col_max,col_min,row_max,row_min;
			//findNonXLayer(non_throt_layer,non_beltway_layer);
			//calROC(col_max,col_min,row_max,row_min,non_beltway_layer);
			//for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++ )
			//for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++ ) 
			//for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++ )
			//	t[x][y][z]->pe->RTM_set_var(non_throt_layer, non_beltway_layer, col_max, col_min, row_max, row_min);
			//cout<<"This is for testing, need to be remove after verify"<<endl;
			//cout<<"non_beltway_layer,non_throt_layer = "<<non_beltway_layer<<","<<non_throt_layer<<endl;
			//cout<<"col_max,col_min,row_max,row_min = "<<col_max<<","<<col_min<<","<<row_max<<","<<row_min<<endl;
			
		}
	}
	else if(NoximGlobalParams::throt_type == THROT_TAVT){
		//for(int z=0; z < NoximGlobalParams::mesh_dim_z - 1 ; z++ )//from top to down, and bottom layer never be throttled.
		for(int z=0; z < NoximGlobalParams::mesh_dim_z     ; z++ )
		for(int y=0; y < NoximGlobalParams::mesh_dim_y     ; y++ ) 
		for(int x=0; x < NoximGlobalParams::mesh_dim_x     ; x++ ){
			if(t[x][y][z]->r->stats.temperature_s_r > NoximGlobalParams::temp_trigger){ // >TEMP_THRESHOLD
				isEmergency = true;
				for( int zz = 0 ; zz < z + 1 ; zz++)
				if(zz < NoximGlobalParams::mesh_dim_z){
					t[x][y][zz]->pe->IntoEmergency();
					t[x][y][zz]->r ->IntoEmergency();
					throttling[x][y][zz]       = 1;
				}
			}
			else{
				if (throttling[x][y][z] == 0){	
					t[x][y][z]->pe->OutOfEmergency();		
					t[x][y][z]->r ->OutOfEmergency();
					throttling[x][y][z]             = 0;
				}
			}
			/*
			int non_beltway_layer,non_throt_layer;
			int col_max,col_min,row_max,row_min;
			findNonXLayer(non_throt_layer,non_beltway_layer);
			calROC(col_max,col_min,row_max,row_min,non_beltway_layer);
			for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++ )
			for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++ ) 
			for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++ )
				t[x][y][z]->pe->RTM_set_var(non_throt_layer, non_beltway_layer, col_max, col_min, row_max, row_min);
			cout<<"This is for testing, need to be remove after verify"<<endl;
			cout<<"non_beltway_layer,non_throt_layer = "<<non_beltway_layer<<","<<non_throt_layer<<endl;
			cout<<"col_max,col_min,row_max,row_min = "<<col_max<<","<<col_min<<","<<row_max<<","<<row_min<<endl;
			*/
		}
	}
	else return isEmergency;//THROT_NORMAL,THROT_TEST do nothing, because the topology won't change
	return isEmergency;
}

void NoximNoC::calROC(int &col_max, int &col_min, int &row_max, int &row_min,int non_beltway_layer){
	int X_min = 0;
	int X_max = NoximGlobalParams::mesh_dim_x - 1;
	int Y_min = 0;
	int Y_max = NoximGlobalParams::mesh_dim_x - 1;
	int Z     = non_beltway_layer;
	
	int m, n, layer;
	bool index = false;
	for( n = 0 ; n < NoximGlobalParams::mesh_dim_y ; n++ ){
		for(layer=0; layer < Z; layer++) 
		for(m=0; m < NoximGlobalParams::mesh_dim_x; m++)
			index |= beltway[m][n][layer];
		if (index){
			Y_min = n ;
			break;
		}
	}
	index = 0;
	for( n = NoximGlobalParams::mesh_dim_y - 1 ; n > -1  ; n-- ){
		for(layer=0; layer < Z; layer++) 
		for(m=0; m < NoximGlobalParams::mesh_dim_x; m++)
			index |= beltway[m][n][layer];
		if (index){
			Y_max = n ;
			break;
		}
	}
	index = 0;
	for( m = 0 ; m < NoximGlobalParams::mesh_dim_x ; m++ ){
		for(layer=0; layer < Z; layer++) 
		for(n = Y_min ; n < Y_max + 1 ; n++)
			index |= beltway[m][n][layer];
		if (index){
			X_min = m ;
			break;
		}
	}
	index = 0;
	for( m = NoximGlobalParams::mesh_dim_x - 1 ; m > -1  ; m-- ){
		for(layer = 0; layer < Z; layer++) 
		for(n=Y_min ; n < Y_max + 1; n++){
			index |= beltway[m][n][layer];
			}
		if (index){
			X_max = m ;
			break;
		}
	}
	col_min = X_min;
	col_max = X_max;
	row_min = Y_min;
	row_max = Y_max;
}
void NoximNoC::setCleanStage(){
	cout<<":Into Clean Stage"<<endl;
	for(int z=0; z < NoximGlobalParams::mesh_dim_z ; z++ )//from top to down
	for(int y=0; y < NoximGlobalParams::mesh_dim_y ; y++ ) 
	for(int x=0; x < NoximGlobalParams::mesh_dim_x ; x++ ){
		//for (int i = 0; i < DIRECTIONS + 1; i++) {
			//t[x][y][z]->r->buffer[i].SetMaxBufferSize(_man_set_max_buffer_size);
		//}
		t[x][y][z]->pe->IntoCleanStage();
		t[x][y][z]->pe->OutOfEmergency();
		t[x][y][z]->r->IntoCleanStage(); //Jimmy added on 2017.05.02 - reset the buffer length in clean stage
		t[x][y][z]->r ->OutOfEmergency();
		//if(x != 2 || y != 2)
		throttling[x][y][z]             = 0;
		
	}
}

void NoximNoC::EndCleanStage(){
	cout<<":Out of Clean Stage"<<endl;
	for(int z=0; z < NoximGlobalParams::mesh_dim_z ; z++ )//from top to down
	for(int y=0; y < NoximGlobalParams::mesh_dim_y ; y++ ) 
	for(int x=0; x < NoximGlobalParams::mesh_dim_x ; x++ ){
		t[x][y][z]->pe->OutOfCleanStage();
		t[x][y][z]->r->OutOfCleanStage(); //Jimmy added on 2017.05.02 - reset the buffer length in clean stage
		//throttling[x][y][z]             = 1;
		
	}
}

void NoximNoC::findNonXLayer(int &non_throt_layer, int &non_beltway_layer){
	int m, n, layer;
	bool index_throt = false,index_beltway = false;
	non_throt_layer = 0,non_beltway_layer = 0;
	for(layer=NoximGlobalParams::mesh_dim_z - 1 ; layer > -1 ; layer-- ){
		for(n=0; n < NoximGlobalParams::mesh_dim_y; n++) 
		for(m=0; m < NoximGlobalParams::mesh_dim_x; m++)
			index_beltway |= beltway[m][n][layer];
		if (index_beltway){
			non_beltway_layer = layer + 1;
			break;
		}
	}
	for(layer = NoximGlobalParams::mesh_dim_z - 1 ; layer > -1 ; layer-- ){
		for(n=0; n < NoximGlobalParams::mesh_dim_y; n++) 
		for(m=0; m < NoximGlobalParams::mesh_dim_x; m++)
			index_throt   |= throttling[m][n][layer];
		if (index_throt){
			non_throt_layer = layer + 1;
			break;
		}
	}
	assert( non_throt_layer > -1 && non_beltway_layer > -1 );
}
void NoximNoC::TransientLog(){
	//calculate the period throughtput

	int packet_in_buffer           =0;
	int packet_in_buffer_locals    =0;
	int throttle_num               =0;
	int par_throttle_1_2           =0;
	int par_throttle_1_3           =0;
	int par_throttle_1_4           =0;
	int par_throttle_1_5           =0;
	int par_throttle_1_6           =0;
	float max_temp                 =0;
	//int total_transmit             =0;
	int total_adaptive_transmit    =0;
	int total_dor_transmit         =0;
	int total_dw_transmit          =0;
	int total_mid_adaptive_transmit=0;
	int total_mid_dor_transmit     =0;
	int total_mid_dw_transmit      =0;
	int total_beltway              =0;
	int max_delay                  =0;
	int max_delay_id               =0;
	int max_delay_id_d             =0;
	int total_drop_pkt             =0; //Jimmy added on 2012.04.24
	int total_pkt_gen              =0; //JImmy added on 2012.04.26
	int total_reshot_pkt           =0; //Jimmy added on 2017.05.30
	int total_allocation_time      =0; //Calvin added 
	
		for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++)
		for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++)
		for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
		for(int d = 0; d < DIRECTIONS+1; d++){
			if ( !(t[x][y][z]->r->buffer[d].IsEmpty()) ){
				if(d <  DIRECTIONS-1) packet_in_buffer++;
				if(d >= (DIRECTIONS-1) && d < (DIRECTIONS+1)) packet_in_buffer_locals++; //Jimmy add on 2017/05/02
				cout<<"In node t["<<x<<"]["<<y<<"]["<<z<<"] direction "<<d<<" waiting time "<<getCurrentCycleNum() - t[x][y][z]->r->buffer[d].Front().waiting_cnt<<" ";
				cout<<t[x][y][z]->r->buffer[d].Front()<<" ";
				cout<<"This flit is route to "<<t[x][y][z]->r->getFlitRoute(d)<<" ack("<<t[x][y][z]->r->ack_tx[d]<<") avalible("<<t[x][y][z]->r->getDirAvailable(d)<<")"<<endl;
				/*if( ((int)(t[x][y][z]->r->buffer[d].Front().timestamp) % (int) (TEMP_REPORT_PERIOD) ) > (int) (TEMP_REPORT_PERIOD) - NoximGlobalParams::clean_stage_time){
					cout<<"flit timestame = "<<t[x][y][z]->r->buffer[d].Front().timestamp<<endl;
					assert(false);
				}*/
			}
		
			if( d == 0){
				if( throttling[x][y][0]==1 ){ throttle_num++; full_throttling[x][y]++;}
				if( throttling[x][y][0]!=1 &&  throttling[x][y][0]==2 ){ par_throttle_1_2++; par_throttling[x][y]++;}
				if( throttling[x][y][z]!=1 &&  throttling[x][y][z]==3 )par_throttle_1_3++;
				if( throttling[x][y][z]!=1 &&  throttling[x][y][z]==4 )par_throttle_1_4++;
				if( throttling[x][y][z]!=1 &&  throttling[x][y][z]==5 )par_throttle_1_5++;
				if( throttling[x][y][z]!=1 &&  throttling[x][y][z]==6 )par_throttle_1_6++;
				max_temp = ( t[x][y][z]->r->stats.temperature > max_temp)?t[x][y][z]->r->stats.temperature:max_temp;
				// total_transmit             += t[x][y][z]->pe->getTransient_Total_Transmit();
				total_adaptive_transmit    += t[x][y][z]->pe->getTransient_Adaptive_Transmit();
				total_dor_transmit         += t[x][y][z]->pe->getTransient_DOR_Transmit();
				total_dw_transmit          += t[x][y][z]->pe->getTransient_DW_Transmit();
				total_mid_adaptive_transmit+= t[x][y][z]->pe->getTransient_Mid_Adaptive_Transmit();
				total_mid_dor_transmit     += t[x][y][z]->pe->getTransient_Mid_DOR_Transmit();
				total_mid_dw_transmit      += t[x][y][z]->pe->getTransient_Mid_DW_Transmit();
				total_beltway              += t[x][y][z]->pe->getTransient_Beltway_Transmit();
				total_drop_pkt             += t[x][y][z]->pe->drop_pkt; //Jimmy added on 2012.04.24
				total_pkt_gen              += t[x][y][z]->pe->inject_pkt; //Jimmy added on 2012.04.26
				total_reshot_pkt           += t[x][y][z]->pe->reshot_pkt; //Jimmy added on 2017.05.30
				total_allocation_time      += t[x][y][z]->r->buffer_allocation_time;//Calvin added on 2015.9.17
				
				//assert(max_temp < 100);
			}
		}
		t[x][y][z]->pe->ResetTransient_Transmit();
		}
		total_transmit = total_adaptive_transmit + 
		total_dor_transmit  +       
		total_dw_transmit   +       
		total_mid_adaptive_transmit+
		total_mid_dor_transmit     +
		total_mid_dw_transmit      +
		total_beltway              ;
		
		
		//assert(max_temp < 100);
	transient_log_throughput<<getCurrentCycleNum()<<"\t"
	                        <<total_transmit<<"\t"
							<<total_transmit/TEMP_REPORT_PERIOD<<"\t"
							<<total_pkt_gen<<"\t"         //Jimmy added
							<<total_reshot_pkt<<"\t"      //Jimmy added
							<<total_drop_pkt<<"\t"        //Jimmy added
							<<throttle_num<<"\t"          //Jimmy added
							<<par_throttle_1_2<<"\t"      //Jimmy added
							<<par_throttle_1_3<<"\t"      //Jimmy added
							<<par_throttle_1_4<<"\t"      //Jimmy added
							<<par_throttle_1_5<<"\t"      //Jimmy added
							<<par_throttle_1_6<<"\t"      //Jimmy added
							<<max_temp<<"\t"
							<<total_allocation_time<<"\t" //Calvin added
							//<<x<<"\t"
							/*<<total_adaptive_transmit    <<"\t"
							<<total_dor_transmit         <<"\t"
							<<total_dw_transmit          <<"\t"
							<<total_mid_adaptive_transmit<<"\t"
							<<total_mid_dor_transmit     <<"\t"
							<<total_mid_dw_transmit      <<"\t"*/
							//<<total_beltway              <<"\t"
							<<endl;
	
	if(NoximGlobalParams::throt_type != THROT_NORMAL); //assert(max_temp < 100);/////大於100度中???��?mark2018.03.31
	
	printf("Throttling Table @ %12.0lf\n",(getCurrentCycleNum()-base_cycle));
	cout<<throttle_num<<" nodes are throttled, "<<packet_in_buffer<<" Non-Empty Buffers,"<<packet_in_buffer_locals<<
          " Non-Empty local Buffers, Throughput "<<total_transmit/TEMP_REPORT_PERIOD<<endl;
				for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++)
					cout<<"Layer "<<z<<"\t";
				cout<<"\n";
				for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++){
					for(int z=0; z < NoximGlobalParams::mesh_dim_z; z++){
						for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
							if(throttling[x][y][z]==1)cout<<"X";
							else if(throttling[x][y][z]==2)cout<<"o";
							else if( beltway[x][y][z] )cout<<"*";
							else cout<<".";
						}
						cout<<"\t";	
					}
					cout<<"\n";			
				}
	/****************///Jack added on 2019.02.24
	
	//results_log_pre_temp << t[4][4][0]->r->stats.pre_temperature1;
	if(cycle>0){
		for(int y=0; y < NoximGlobalParams::mesh_dim_y; y++)
			for(int x=0; x < NoximGlobalParams::mesh_dim_x; x++){
				results_log_throttling << throttling[x][y][0] << " ";
			}
		results_log_throttling << endl;
	}
	node += throttle_num;
	partial_throttled_node += par_throttle_1_2;
	cout << "Total throttled node is " << node << endl;
	cout << "Total partial throttled node is " << partial_throttled_node << endl;
	results_log_pre_temp << endl;
	
	
}



void NoximNoC::choose_act(double in[10][10][10]){
	double max;
	int max_i;
	double delt = 1000000.0/(count*count+1000000.0);
	double e_greedy = 0.99 *(1 - delt);
	double lim_0 = 1.0/3.0 + 2.0/3.0 * delt;
	double lim_1 = 1.0/3.0 - 1.0/3.0 * delt + lim_0;
	
	for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
		for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
			if(in[m][n][4]>100 || in[m][n][4] == 100)
				t[m][n][0]->r->node_no = 1;
			if(in[m][n][4]>=NoximGlobalParams::temp_trigger){
				if(in[m][n][4]>100 || in[m][n][4] == 100)
					in[m][n][4] = 100.0;
				for(int j=0;j<6;j++) //store St
						D[m][n][count%D_MAX][j] = in[m][n][j];
				
				for(int i=0;i<3;i++){    //store Qt
					int index = (in[m][n][4]-NoximGlobalParams::temp_trigger)/0.3;
					if(in[m][n][4] == 100.0)
						index = 4;
					D[m][n][count%D_MAX][i+6] = Q_table[m][n][index][i];
				}
				double p = rand() / (RAND_MAX + 1.0);
				double p1 = rand() / (RAND_MAX + 1.0);
				if(p>e_greedy){		//Random choose
					int index = (in[m][n][4]-NoximGlobalParams::temp_trigger)/0.3;
					if(p<lim_0)
						max_i=0;
					else if(p<lim_1)
						max_i=2;
					else
						max_i=1;
					max = Q_table[m][n][index][max_i];
					cout<<'*';
				}
				else{
					//Maximun_Q_value
					int index = (in[m][n][4]-NoximGlobalParams::temp_trigger)/0.3;
					if(in[m][n][4] == 100.0)
						index = 4;
					max = Q_table[m][n][index][0];
					max_i = 0;
					for(int i=1;i<3;i++){
						if(Q_table[m][n][index][i] > max){
							max = Q_table[m][n][index][i];
							max_i = i;
						}
					}
					cout<<'_';
				}
				//Maximun_Q_value
				/*	int index = (in[m][n][4]-NoximGlobalParams::temp_trigger)/0.3;
					if(in[m][n][4] == 100.0)
						index = 4;
					max = Q_table[m][n][index][0];
					max_i = 0;
					for(int i=1;i<3;i++){
						if(Q_table[m][n][index][i] > max){
							max = Q_table[m][n][index][i];
							max_i = i;
						}
					}
					cout<<'_';*/
				t[m][n][0]->r->node_no = max_i;
				D[m][n][count%D_MAX][21] = max_i;//store at 
				D[m][n][count%D_MAX][18] = max;  //store maxQt
				
				
			}
			else{
				t[m][n][0]->r->node_no = 0;
				cout<< "x";
			}
		}
		cout<<"\n";
	}
	

}

void NoximNoC::update_table(double in[10][10][10]){
	double max;
	int max_i;
	double gamma = 0.9;
	double lr = 1.0 / ((count>D_MAX?D_MAX:count)/50) / 16.0;
	int index;
	
	double delt = 1000000.0/(count*count+1000000.0);
	double e_greedy = 0.99 *(1 - delt);
	//double e_greedy = 0.99;
	double lim_0 = 1.0/3.0 + 2.0/3.0 * delt;
	double lim_1 = 1.0/3.0 - 1.0/3.0 * delt + lim_0;
	

	
	if(count>100){
		//cout<< "INFO " << (count>D_MAX?D_MAX:count)/50 << endl;
		for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
			for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
				
				for(int num=0;num < (count>D_MAX?D_MAX:count)/50;num++){
					
					index = (count-1+num*(50+1))%(count>D_MAX?D_MAX:count);
					//cout<<index<<endl;
					int chosen_i = D[m][n][index][21];
					//re-calculte tar_net w/ current weight
					double Q_max = 0;
					if(D[m][n][index][22]>=0)//no gamma * Qmax for terminate step
					{
						int index1 = (D[m][n][index][13]-NoximGlobalParams::temp_trigger)/0.3;
						if(D[m][n][index][13] == 100.0)
							index1 = 4;
						Q_max = Q_table[m][n][index1][0];
						for(int i=1;i<3;i++)
							Q_max =  Q_table[m][n][index1][i] > Q_max ? Q_table[m][n][index1][i] : Q_max;
					}
					int index2 = (D[m][n][index][4]-NoximGlobalParams::temp_trigger)/0.3;
					if(D[m][n][index][4] == 100.0)
						index2 = 4;
					if(m == NoximGlobalParams::DQN_x && n == NoximGlobalParams::DQN_y){
						//cout<< Q_table[NoximGlobalParams::DQN_x][NoximGlobalParams::DQN_y][index2][chosen_i];
						//cout<<endl;
					}
					Q_table[m][n][index2][chosen_i] += lr*(D[m][n][index][20] + 0.9*Q_max - Q_table[m][n][index2][chosen_i]);
					if(m == NoximGlobalParams::DQN_x && n == NoximGlobalParams::DQN_y){
						//cout<< "lr: " << lr << " Reward: " << D[m][n][index][20] << " Qmax: " << max << endl << endl;
						//cout<< "After update: " << Q_table[NoximGlobalParams::DQN_x][NoximGlobalParams::DQN_y][index2][chosen_i];
					}
					
				}
				
			}
			
		}
		
	}
	for(int n=0; n < NoximGlobalParams::mesh_dim_y; n++){
		for(int m=0; m < NoximGlobalParams::mesh_dim_x; m++){
			if(in[m][n][4] >= NoximGlobalParams::temp_trigger){
				if(in[m][n][4]>100 || in[m][n][4] == 100)
					in[m][n][4] = 100.0000;
				for(int j=0;j<6;j++) //store St
					D[m][n][(count-1)%D_MAX][j+9] = in[m][n][j];
				
				int index = (in[m][n][4]-NoximGlobalParams::temp_trigger)/0.3;
				if(in[m][n][4] == 100.0)
					index = 4;
				for(int i=0;i<3;i++){    //store Qt
					D[m][n][(count-1)%D_MAX][i+15] = Q_table[m][n][index][i];
				}
				
				double p = rand() / (RAND_MAX + 1.0);
				if(p>e_greedy){		//Random choose
					int index = (in[m][n][4]-NoximGlobalParams::temp_trigger)/0.3;
					if(p<lim_0)
						max_i=0;
					else if(p<lim_1)
						max_i=2;
					else
						max_i=1;
					
					max = Q_table[m][n][index][max_i];
					
				}
				else{
					//Maximun_Q_value
					int index = (in[m][n][4]-NoximGlobalParams::temp_trigger)/0.3;
					if(in[m][n][4] == 100.0)
						index = 4;
					max = Q_table[m][n][index][0];
					max_i = 0;
					for(int i=1;i<3;i++){
						if(Q_table[m][n][index][i] > max){
							max = Q_table[m][n][index][i];
							max_i = i;
						}
					}
					
				}
				//t[m][n][0]->r->node_no = max_i;
				D[m][n][(count-1)%D_MAX][19] = max;  //store maxQt+1
			}
		}
		
	}	
	
	
}





