//Design by CMH, Access Lab, NTU

#ifndef __THERMAL_IF_H__
#define __THERMAL_IF_H__

#include "NoximMain.h"
#include "co-sim.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>

//HotSpot source code
extern "C"{ //Foster:HotSpot��C�g���ANoxim�OC++�A�[����ޥ�C�y���{��
#include "temperature.h"
#include "temperature_grid.h"	/* for dump_steady_temp_grid	*/
#include "temperature_block.h"	/* for dump_steady_temp_grid	*/
#include "flp.h"
#include "util.h"
#include "npe.h"
#include "shape.h"
}
#include <stdio.h>
#include "NoximTile.h"
using namespace std;



class Thermal_IF{
public:
    Thermal_IF();
	Thermal_IF(int x,int y, int z) :mesh_dim_x(x), mesh_dim_y(y), mesh_dim_z(z) 
	{ fpGenerator(); hs_init(); logFile_init(); PredictErrorLog();}
	~Thermal_IF() 
	{ if(results_log_temp.is_open()) results_log_temp.close(); }
	
	void initial() { fpGenerator(); }
	void setInitTemp(double * init_t)		//from temperature_grid.c::set_temp_grid()
	{
		grid_model_t *model = hs_model->grid;
		double val = hs_model->config->init_temp;
		int i;
		
		double total_nodes;
		if (model->config.model_secondary)
			total_nodes = model->total_n_blocks + EXTRA + EXTRA_SEC;
		else
			total_nodes = model->total_n_blocks + EXTRA;
			
		if (model->total_n_blocks <= 0)
			fatal("total_n_blocks is not greater than zero\n");
		for(i=0; i < total_nodes; i++)
		{
			bool insize = (i<NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y*NoximGlobalParams::mesh_dim_z*3);
			hs_inst_temp[i] = (insize && i%3==0)?(init_t[i/3]+273.15):val;
		}
	}
	double tempRec(int idx, int modu)
	{
		return hs_inst_temp[idx*3+modu] - 273.15;
	}
	void Temperature_calc(vector<double>& pwr, vector<double>&temp)
	{ 
	  hs_temperature(pwr); 
	  Tmp2TtraceFile(temp); 
	}
	void fpGenerator();  //floorplan Generator
	void steadyTmp( NoximTile* t[MAX_STATIC_DIM][MAX_STATIC_DIM][MAX_STATIC_DIM]){ hs_steady(t); }
	void finish() { hs_finish(); }
	
private:
    int mesh_dim_x, mesh_dim_y, mesh_dim_z;

    /*=================================Variable for HotSpot======================*/
	/* floorplan	*/
	flp_t *hs_flp;
	/* hotspot temperature model	*/
	RC_model_t *hs_model;
	/* instantaneous temperature and power values	*/
	double *hs_inst_temp, *hs_inst_power;
	/* steady state temperature and power values	*/
	double *hs_overall_power, *hs_steady_temp;
	int hs_first_call;

    //Thermal model function of HotSpot
    void hs_init();
	//void hs_steady();
	void hs_steady( NoximTile *t[MAX_STATIC_DIM][MAX_STATIC_DIM][MAX_STATIC_DIM]);
	void hs_finish();
	void hs_temperature(vector<double>& pwr);
	void Tmp2TtraceFile(vector<double>& temp);
	
	//logFile
	ofstream results_log_temp;
	ofstream results_log_router;
	void PredictErrorLog(); //Jimmy added on 2013.03.07

	void logFile_init();
};
#endif
