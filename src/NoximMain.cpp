 /*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the top-level of Noxim
 */
#include "NoximMain.h"
#include "NoximNoC.h"
#include "NoximGlobalStats.h"
#include "NoximCmdLineParser.h"
#include "NoximLog.h"
#include <sys/stat.h>
using namespace std;
    
// need to be globally visible to allow "-volume" simulation stop
unsigned int drained_volume;
// Log file stream
ofstream results_log_pwr;           //Initial in NoximLog, record in thermal_IF.cpp
ofstream transient_log_throughput;  //Initial in NoximLog, record in NoximNoC.cpp

// Initialize global configuration parameters (can be overridden with command-line arguments)
bool                         NoximGlobalParams::DQN_mon                       = false;
int                          NoximGlobalParams::DQN_x                         = 0;
int                          NoximGlobalParams::DQN_y                         = 0;
bool                         NoximGlobalParams::DQN                           = false;
bool                         NoximGlobalParams::DQN_init                      = false;
bool						 NoximGlobalParams::start                         = true;
bool                         NoximGlobalParams::init_throttle                 = false;
int                          NoximGlobalParams::verbose_mode                  = DEFAULT_VERBOSE_MODE;
int                          NoximGlobalParams::trace_mode                    = DEFAULT_TRACE_MODE;
char                         NoximGlobalParams::trace_filename[128]           = DEFAULT_TRACE_FILENAME;
int                          NoximGlobalParams::mesh_dim_x                    = DEFAULT_MESH_DIM_X;
int                          NoximGlobalParams::mesh_dim_y                    = DEFAULT_MESH_DIM_Y;
int                          NoximGlobalParams::mesh_dim_z                    = DEFAULT_MESH_DIM_Z;
int                          NoximGlobalParams::buffer_depth                  = DEFAULT_BUFFER_DEPTH;
int                          NoximGlobalParams::min_packet_size               = DEFAULT_MIN_PACKET_SIZE;
int                          NoximGlobalParams::max_packet_size               = DEFAULT_MAX_PACKET_SIZE;
int                          NoximGlobalParams::routing_algorithm             = DEFAULT_ROUTING_ALGORITHM;
char                         NoximGlobalParams::routing_table_filename[128]   = DEFAULT_ROUTING_TABLE_FILENAME;
int                          NoximGlobalParams::selection_strategy            = DEFAULT_SELECTION_STRATEGY;
float                        NoximGlobalParams::packet_injection_rate         = DEFAULT_PACKET_INJECTION_RATE;
float                        NoximGlobalParams::probability_of_retransmission = DEFAULT_PROBABILITY_OF_RETRANSMISSION;
int                          NoximGlobalParams::traffic_distribution          = DEFAULT_TRAFFIC_DISTRIBUTION;
char                         NoximGlobalParams::traffic_table_filename[128]   = DEFAULT_TRAFFIC_TABLE_FILENAME;
int                          NoximGlobalParams::simulation_time               = DEFAULT_SIMULATION_TIME;
int                          NoximGlobalParams::stats_warm_up_time            = DEFAULT_STATS_WARM_UP_TIME;
int                          NoximGlobalParams::rnd_generator_seed            = 0;//time(NULL);
bool                         NoximGlobalParams::detailed                      = DEFAULT_DETAILED;
float                        NoximGlobalParams::dyad_threshold                = DEFAULT_DYAD_THRESHOLD;
unsigned int                 NoximGlobalParams::max_volume_to_be_drained      = DEFAULT_MAX_VOLUME_TO_BE_DRAINED;
vector <pair <int, double> > NoximGlobalParams::hotspots;
int                          NoximGlobalParams::dw_layer_sel                  = DEFAULT_DW_LAYER_SEL;
int                          NoximGlobalParams::throt_type                    = DEFAULT_THROTTLING_TYPE;
int                          NoximGlobalParams::down_level                    = DEFAULT_DOWN_LEVEL;
float	                     NoximGlobalParams::throt_ratio	                  = DEFAULT_THROTTLING_RATIO;
bool                         NoximGlobalParams::buffer_alloc                  = DEFAULT_BUFFER_ALLOC;
int                          NoximGlobalParams::vertical_link                 = DEFAULT_VERTICAL_LINK;
bool                         NoximGlobalParams::cascade_node                  = DEFAULT_CASCADE_NODE;
bool                         NoximGlobalParams::Mcascade                      = DEFAULT_MCASCADE;
int                          NoximGlobalParams::Mcascade_step                 = 0;
bool                         NoximGlobalParams::beltway                       = DEFAULT_BELTWAY;
float                        NoximGlobalParams::beltway_trigger               = TEMP_THRESHOLD; 
bool                         NoximGlobalParams::Mbeltway                      = DEFAULT_MBELTWAY;
bool                         NoximGlobalParams::Sbeltway                      = DEFAULT_SBELTWAY;
int                          NoximGlobalParams::Sbeltway_ring                 = 0;
int                          NoximGlobalParams::ROC_UP                        = 4;
int                          NoximGlobalParams::ROC_DOWN                      = 3;
float                        NoximGlobalParams::beltway_ratio                 = 0.25;
bool                         NoximGlobalParams::Log_all_Temp                  = false;
int                          NoximGlobalParams::clean_stage_time              = 10000;
bool                         NoximGlobalParams::cal_temp                      = DEFAULT_CAL_TEMP;
int                          NoximGlobalParams::predict_vol                   = DEFAULT_PREDICT_VOL;
int                          NoximGlobalParams::pkt_queue_depth               = DEFAULT_PACKET_QUEUE_DEPTH;
int                          NoximGlobalParams::static_quota                  = DEFAULT_STATIC_TQUOTA;
bool                         NoximGlobalParams::selection_info                = DEFAULT_SELECTION_INFO;
bool                         NoximGlobalParams::message_level                 = DEFAULT_MESSAGE_LEVEL;
bool                         NoximGlobalParams::tla_support                   = DEFAULT_TLA;
float                        NoximGlobalParams::temp_trigger                  = TEMP_THRESHOLD; //Jimmy added on 2012.11.06
int                          NoximGlobalParams::dfs_level                     = DEFAULT_DFS_LEVEL; //Jimmy added on 2013.03.06
int                          NoximGlobalParams::sensor_allocate               = DEFAULT_SENSOR_ALLOCATE; //Jimmy added on 2015.05.27
char                         NoximGlobalParams::sensor_table_filename[128]        = DEFAULT_SENSOR_TABLE_FILENAME; //Jimmy added on 2016.03.19
int                          NoximGlobalParams::temp_recover                      = DEFAULT_TEMPERATURE_RECOVER; //Jimmy added on 2016.03.20
char                         NoximGlobalParams::rector_linear_coef_filename[128]  = DEFAULT_RECTOR_LINEAR_FILENAME; //Jimmy added on 2016.03.20
char                         NoximGlobalParams::rector_regression_coef_filename[128]=DEFAULT_RECTOR_REGRESSION_FILENAME;//Andy added on 2017
char                         NoximGlobalParams::rector_regression_coef1_filename[128]=DEFAULT_RECTOR_REGRESSION1_FILENAME;//Andy added on 2017
char                         NoximGlobalParams::rector_regression_ref_filename[128]=DEFAULT_RECTOR_REF_FILENAME;//Andy added on 2017
int                          NoximGlobalParams::Buffer_Allocation             = DEFAULT_BUFFER_ALLOCATION; //Calvin add on 2015.07.27
int                          NoximGlobalParams::Throt_Reset_Buffer            = DEFAULT_Throt_Reset_Buffer; //Calvin add on 2015.07.27
int                          NoximGlobalParams::TBDB_Type                     = DEFAULT_TBDB_Type; //Calvin add on 2015.07.27
int                          NoximGlobalParams::TD_Type                       = DEFAULT_TD_Type; //Dora add on 2017.04.19
bool                         NoximGlobalParams::Drop_or_not                   = DEFAULT_Drop_or_not;
//---------------------------------------------------------------------------

int sc_main(int arg_num, char *arg_vet[])
{
    // TEMP
    drained_volume = 0;

    // Handle command-line arguments
    cout << endl << "\t\tNoxim - the NoC Simulator" << endl;
    cout << "\t\t(C) University of Catania" << endl << endl;

    parseCmdLine(arg_num, arg_vet);

    // Signals
    sc_clock clock("clock", 1, SC_NS);
    sc_signal <bool> reset;

    // NoC instance
	cout<<"Building Mesh..."<<endl;
    NoximNoC *n = new NoximNoC("NoC");
    n->clock(clock);
    n->reset(reset);
	//Log files
	NoximLog log;
	cout<<"Initial log files..."<<endl;
	if (NoximGlobalParams::trace_mode)log.TraceSignal(n);
	log.PowerLog();//Transient power tracefile 
	log.Throughput();
	
	

    // Reset the chip and run the simulation
    reset.write(1);
    cout << "Reset...\n";
    //srand(NoximGlobalParams::rnd_generator_seed);	// time(NULL));
	srand(time(NULL));//modified by Jack 2018.02.19
	//DEFAULT_RESET_TIME = # of simulation cycle
    
	sc_start(DEFAULT_RESET_TIME * CYCLE_PERIOD + 1, SC_NS);

    reset.write(0);
    cout << " done! Now running for " << NoximGlobalParams::
	simulation_time << " cycles..." << endl;
	// sc_start( TEMP_REPORT_PERIOD - DEFAULT_RESET_TIME * CYCLE_PERIOD  , SC_NS);
	// int i;
	// for ( i = TEMP_REPORT_PERIOD ; i < NoximGlobalParams::simulation_time ; i += TEMP_REPORT_PERIOD ){
		// reset.write(1);
		// sc_start( DEFAULT_RESET_TIME * CYCLE_PERIOD , SC_NS);
		// reset.write(0);
		// cout<<getCurrentCycleNum()<<": Reset done."<<endl;
		// sc_start( TEMP_REPORT_PERIOD - DEFAULT_RESET_TIME * CYCLE_PERIOD  , SC_NS);
		// cout<<getCurrentCycleNum()<<" cycles run"<<endl;
		
		
	// }
	// sc_start(NoximGlobalParams::simulation_time * CYCLE_PERIOD - getCurrentCycleNum() , SC_NS);
	
	sc_start(NoximGlobalParams::simulation_time * CYCLE_PERIOD + TEMP_REPORT_PERIOD , SC_NS);
	
    // Close the simulation
	cout << "Noxim simulation completed."                        << endl;
    cout << " ( " << getCurrentCycleNum() << " cycles executed)" << endl;
	// Show statistics
    NoximGlobalStats gs(n);
    gs.showStats(std::cout, NoximGlobalParams::detailed);
	
	//
	transient_log_throughput << endl; 
	transient_log_throughput << "% Total received packets: "                  << gs.getReceivedPackets     () << endl;
    transient_log_throughput << "% Total received flits: "                    << gs.getReceivedFlits       () << endl;
    transient_log_throughput << "% Global average delay (cycles): "           << gs.getAverageDelay        () << endl;
	transient_log_throughput << "% Global average NI delay (cycles): "        << gs.getNiAverageDelay      () << endl;
	transient_log_throughput << "% Global average NW delay (cycles): "        << gs.getNwAverageDelay      () << endl;
	transient_log_throughput << "% Global Total Nw  avg delay (cycles): "     << gs.getNwTotalAverageDelay () << endl;
	transient_log_throughput << "% Global Total Ni  avg delay (cycles): "     << gs.getNiTotalAverageDelay () << endl;
	transient_log_throughput << "% Global Total Msg avg delay (cycles): "     << gs.getMsgTotalAverageDelay() << endl;
    transient_log_throughput << "% Global average throughput (flits/cycle): " << gs.getAverageThroughput   () << endl;
    transient_log_throughput << "% Throughput (flits/cycle/IP): "             << gs.getThroughput          () << endl;
    transient_log_throughput << "% Max delay (cycles): "                      << gs.getMaxDelay            () << endl;
	 
	   
	if (NoximGlobalParams::trace_mode)log.TraceEnd();
	log.BufferLog(n);
	log.TrafficLog(n);
	log.PowerLogEnd();
	log.ThroughputEnd();
	log.WeightsMatrix(n);
	
    
    if ((NoximGlobalParams::max_volume_to_be_drained > 0) && getCurrentCycleNum() >= NoximGlobalParams::simulation_time ) {
	cout << "\nWARNING! the number of flits specified with -volume option" << endl;
	cout << "has not been reached."                                        << endl;
	cout << "You might want to try an higher value of simulation cycles"   << endl;
	cout << "using -sim option."                                           << endl;
	cout << "\n Effective drained volume: "   << drained_volume            << endl;
    }
    return 0;
}
