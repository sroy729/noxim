/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file represents the top-level testbench
 */

#ifndef __NOXIMNOC_H__
#define __NOXIMNOC_H__
#include <time.h>
#include <systemc.h>
#include <iostream>
#include "NoximTile.h"
#include "NoximVLink.h"
#include "NoximGlobalRoutingTable.h"
#include "NoximGlobalTrafficTable.h"
#include "NoximGlobalSensorTable.h" //Jimmy added on 2016.03.21
#include "thermal_IF.h"
#include "NoximMain.h"
#include "NoximRouter.h"//Jack added
using namespace std;

extern ofstream results_log_pwr;

SC_MODULE(NoximNoC)
{
    // I/O Ports
    sc_in_clk clock;		// The input clock for the NoC
    sc_in < bool > reset;	// The reset signal for the NoC

    // Signals
	/****************MODIFY BY HUI-SHUN********************/
    //Hui-shun add the third dim in the end
	//and add up/down signals
	sc_signal <bool> req_to_east                   [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <bool> req_to_west                   [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <bool> req_to_south                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <bool> req_to_north                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	//To Tile and To Vertical Link
	sc_signal <bool> req_toT_up                    [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1]; 
	sc_signal <bool> req_toT_down                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal <bool> req_toV_up                    [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1]; 
	sc_signal <bool> req_toV_down                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	
    sc_signal <bool> ack_to_east                   [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <bool> ack_to_west                   [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <bool> ack_to_south                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <bool> ack_to_north                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	
	sc_signal <bool> ack_toT_up                    [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1]; 
	sc_signal <bool> ack_toT_down                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal <bool> ack_toV_up                    [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1]; 
	sc_signal <bool> ack_toV_down                  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	
    sc_signal <NoximFlit> flit_to_east             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximFlit> flit_to_west             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximFlit> flit_to_south            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximFlit> flit_to_north            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	//To Tile and To Vertical Link
	sc_signal <NoximFlit> flit_toT_up              [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximFlit> flit_toT_down            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal <NoximFlit> flit_toV_up              [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximFlit> flit_toV_down            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

    sc_signal <int> free_slots_to_east             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <int> free_slots_to_west             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <int> free_slots_to_south            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <int> free_slots_to_north            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal <int> free_slots_to_up               [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal <int> free_slots_to_down             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    // NoP 
    sc_signal <NoximNoP_data> NoP_data_to_east     [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximNoP_data> NoP_data_to_west     [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximNoP_data> NoP_data_to_south    [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximNoP_data> NoP_data_to_north    [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal <NoximNoP_data> NoP_data_to_up       [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal <NoximNoP_data> NoP_data_to_down     [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	// RCA Monitor Network
	/*** (Jimmy added on 2012.06.27) ***/
	sc_signal<int>    RCA_data_to_east0[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	sc_signal<int>    RCA_data_to_east1[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	sc_signal<int>    RCA_data_to_west0[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	sc_signal<int>    RCA_data_to_west1[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	sc_signal<int>    RCA_data_to_south0[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	sc_signal<int>    RCA_data_to_south1[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	sc_signal<int>    RCA_data_to_north0[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	sc_signal<int>    RCA_data_to_north1[MAX_STATIC_DIM+1][MAX_STATIC_DIM+1][MAX_STATIC_DIM+1];
	/********************************************/
	
	sc_signal<double>    RCA_to_east               [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<double>    RCA_to_west               [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<double>    RCA_to_south              [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<double>    RCA_to_north              [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<double>    RCA_to_up                 [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<double>    RCA_to_down               [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	/*******THROTTLING******/
	sc_signal<bool>	on_off_to_east                 [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<bool>	on_off_to_west                 [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<bool>	on_off_to_south                [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	sc_signal<bool>	on_off_to_north                [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

	/*************** Jimmy added on 2014.12.16 *******************/
	//+++ the signal of temperature prediction results for each neighbor +++//
	sc_signal<float>        PDT_to_east            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        PDT_to_west            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        PDT_to_south           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        PDT_to_north           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        PDT_to_up              [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        PDT_to_down            [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

	//+++ the signal of current buffer depth for each neighbor +++//
	// the information of North Buffer Depth fo each neighbor
    sc_signal<float>        buf0_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf0_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf0_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf0_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf0_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf0_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

	// the information of East Buffer Depth fo each neighbor
	sc_signal<float>        buf1_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf1_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf1_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf1_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf1_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf1_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

	// the information of South Buffer Depth fo each neighbor
	sc_signal<float>        buf2_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf2_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf2_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf2_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf2_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf2_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

	// the information of West Buffer Depth fo each neighbor
	sc_signal<float>        buf3_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf3_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf3_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf3_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf3_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf3_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	
	// the information of Up Buffer Depth fo each neighbor
	sc_signal<float>        buf4_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf4_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf4_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf4_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf4_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf4_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

	// the information of Down Buffer Depth fo each neighbor
	sc_signal<float>        buf5_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf5_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf5_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf5_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf5_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf5_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

	// the information of Local Buffer Depth fo each neighbor
	sc_signal<float>        buf6_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf6_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf6_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf6_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf6_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf6_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];

	// the information of Semi_Local Buffer Depth fo each neighbor
	sc_signal<float>        buf7_to_east           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf7_to_west           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf7_to_south          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf7_to_north          [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf7_to_up             [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
    sc_signal<float>        buf7_to_down           [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
	/*************************************************************/
	
	sc_signal< NoximNoP_data > vertical_free_slot  [MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1][MAX_STATIC_DIM + 1];
	// Matrix of tiles
    NoximTile  *t[MAX_STATIC_DIM][MAX_STATIC_DIM][MAX_STATIC_DIM];
	NoximVLink *v[MAX_STATIC_DIM][MAX_STATIC_DIM];
    // Global tables
    NoximGlobalRoutingTable grtable;
    NoximGlobalTrafficTable gttable;
   	NoximGlobalSensorTable       gstable; //Jimmy added on 2016.03.19
	 NoximGlobalRecoverCoefTable  gltable; //Jimmy added on 2016.03.21
    NoximGlobalRecoverCoefTable  gmtable; //Andy added on 2017
	  NoximGlobalSensorTable       gntable; //Andy added on 2017
    // Constructor
    SC_CTOR(NoximNoC) {
		// Build the Mesh
		buildMesh();
		times = time(NULL);
		//---------- Hot spot interface BY CMH <start>
		//HS_initial();
		HS_interface = new Thermal_IF(NoximGlobalParams::mesh_dim_x,NoximGlobalParams::mesh_dim_y, NoximGlobalParams::mesh_dim_z);
		instPowerTrace   .resize(3*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_z, 0);
		overallPowerTrace.resize(3*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_z, 0);
		TemperatureTrace .resize(3*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_z, 0);
		SC_METHOD(entry);
		sensitive << reset;
		sensitive << clock.pos();
		//---------- Hot spot interface BY CMH <end>
    }
    ~NoximNoC() 
	{ 
	  delete HS_interface; 
	};
	// Support methods
    NoximTile *searchNode(const int id) const;
	 bool EmergencyDecision();
   bool EmergencyDecision_sen(); // the decision is made by considering the reconstruction temperature information (Jimmy added on 2016.03.24)

  private:

	    void buildMesh();
		void entry();
	    Thermal_IF* HS_interface;
        //Power trace	
		vector<double> instPowerTrace;
		vector<double> overallPowerTrace;
		//get transient power in one sample period
		void transPwr2PtraceFile();
		void steadyPwr2PtraceFile();
		int times;
		//Temperature trace 
		vector<double> TemperatureTrace;
        void setTemperature(double in[10][10][10]);
		void calROC(int &col_max, int &col_min, int &row_max, int &row_min,int non_beltway_layer);
		void setCleanStage();
		void EndCleanStage();
		void findNonXLayer(int &non_throt_layer, int &non_beltway_layer);
		void TransientLog();
		int cycle;
		int node;
		int partial_throttled_node;
		int full_throttling[10][10];
		int par_throttling[10][10];
		double avg_error;
		int terminal;
		double reward[10][10];
		double TemperatureRecover(int o, int n, int m); //Jimmy added on 2016.03.21
		void NN_forward(int pre_ly, int ly,int act,int ly_no,double in[10][10][10]);
		void NN_backward(int pre_ly, int ly,int act,int ly_no,double in[10][10][10]);
		void eva_NN(double in[10][10][10]);
		void tar_NN(double in[10][10][10]);
		void update_table(double in[10][10][10]);
		void choose_act(double in[10][10][10]);
		int total_transmit;
		unsigned long long count;
		int number;
		/**/
		ifstream win;
		ifstream win_tar;
		ifstream win_eva;
		ifstream din;
		ifstream init_temp_csv;
		ofstream wo;
		ofstream wo_tar;
		ofstream wo_eva;
		ofstream dout;
		ofstream rew_out;
		ofstream temp_log_csv;
		ofstream reward_log_csv;
		ofstream qt_out;
		ofstream results_log_temp0;
		ofstream results_log_temp1;
		ofstream results_log_temp2;
		ofstream results_log_temp3;
		ofstream results_log_temp4;
		ofstream results_log_temp5;
		ofstream results_log_temp6;
		
		ofstream results_log_prederr0;
		ofstream results_log_prederr1;
		ofstream results_log_prederr2;
		ofstream results_log_prederr3;
		ofstream results_log_prederr4;
		ofstream results_log_prederr5;
		ofstream results_log_prederr6;
		ofstream results_log_pre_temp;//jack added on 2019.02.19
		ofstream results_log_throttling;//jack added on 2019.02.19
		ofstream results_log_weight;//jack added on 2019.02.19
		ifstream w;
		ifstream qt_in;
		ifstream number_in;
		ofstream number_out;
		ofstream results_log_temp0_s;   //recorde the real sensor's sensing results
		ofstream results_log_temp0_s_r; //record the full-chip temperature distribution after temperature recovery (Jimmy added on 2016.03.21) 
};

#endif
