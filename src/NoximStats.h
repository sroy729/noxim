/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the declaration of the statistics
 */

#ifndef __NOXIMSTATS_H__
#define __NOXIMSTATS_H__

#include <iostream>
#include <iomanip>
#include <vector>
#include "NoximMain.h"
#include "NoximPower.h"
using namespace std;

struct CommHistory {
    int src_id;
     vector < double >delays;
	 vector < double >ni_delays;
	 vector < double >nw_delays;
    unsigned int total_received_flits;
    double last_received_flit_time;
};

class NoximStats {

  public:

    NoximStats() {
    } void configure(const int node_id, const double _warm_up_time);

    // Access point for stats update
    void receivedFlit(const double arrival_time, const NoximFlit & flit);

    // Returns the average delay (cycles) for the current node as
    // regards to the communication whose source is src_id
    double getAverageDelay(const int src_id);

    // Returns the average delay (cycles) for the current node
    double getAverageDelay();

	double getNiAverageDelay(const int src_id);
	double getNiAverageDelay();
	double getNwAverageDelay(const int src_id);
	double getNwAverageDelay();
    // Returns the max delay for the current node as regards the
    // communication whose source node is src_id
    double getMaxDelay(const int src_id);
	
	// Returns the delay histogram
	double getDelayHist(NoximCoord dst,double interval, double *DistHist, double *DelayHist);
	
    // Returns the max delay (cycles) for the current node
    double getMaxDelay();

    // Returns the average throughput (flits/cycle) for the current node
    // and for the communication whose source is src_id
    double getAverageThroughput(const int src_id);

    // Returns the average throughput (flits/cycle) for the current node
    double getAverageThroughput();

    // Returns the number of received packets from current node
    unsigned int getReceivedPackets();

    // Returns the number of received flits from current node
    unsigned int getReceivedFlits();

    // Returns the number of communications whose destination is the
    // current node
    unsigned int getTotalCommunications();

    // Returns the energy consumed for communication src_id-->dst_id
    // under the following assumptions: (i) Minimal routing is
    // considered, (ii) constant packet size is considered (as the
    // average between the minimum and the maximum packet size).
    double getCommunicationEnergy(int src_id, int dst_id);

    // Shows statistics for the current node
    void showStats(int curr_node, std::ostream & out =
		   std::cout, bool header = false);
	
  public:
	
    NoximPower power;
	double	   temperature;           //the real temperature
	double	   last_temperature;	  //record the last temperature for comparison to decision throttled quota
	double     last_pre_temperature1; //the last prediciton result (Jimmy modified on 2012.04.12)
	double     last_pre_temperature2; //the last prediciton result (Jimmy modified on 2012.04.12)
	double     last_pre_temperature3; //the last prediciton result (Jimmy modified on 2012.04.12)
	double     last_pre_temperature4; //the last prediciton result (Jimmy modified on 2012.04.12)
	double     last_pre_temperature5; //the last prediciton result (Jimmy modified on 2012.04.12)
	double     last_pre_temperature6; //the last prediciton result (Jimmy modified on 2012.04.12)	
	double     pre_temperature1;      //the prediciton result after 1 delta t(Jimmy modified on 2012.04.12)
	double     pre_temperature2;      //the prediciton result after 2 delta t(Jimmy modified on 2012.04.12)
	double     pre_temperature3;      //the prediciton result after 3 delta t(Jimmy modified on 2012.04.12)
	double     pre_temperature4;      //the prediciton result after 4 delta t(Jimmy modified on 2012.04.12)
	double     pre_temperature5;      //the prediciton result after 5 delta t(Jimmy modified on 2012.04.12)
	double     pre_temperature6;      //the prediciton result after 6 delta t(Jimmy modified on 2012.04.12)	
	double     predict_error1;        //the prediciton error between the temp_PD1 and temp_PD0 (Jimmy added on 2013.03.06)
	
	double	   temperature_s;           //the sensing temperature (Jimmy added on 2015.05.27)
  double     temperature_s_r;         //the reconstruction temperature after temperature reconstruction (Jimmy added on 2016.03.22)
	
	unsigned int received[ DIRECTIONS + 1]; // for counting 7 port arrival rate
	
  private:

    int id;
    vector < CommHistory > chist;
    double warm_up_time;

    int searchCommHistory(int src_id);
};

#endif
