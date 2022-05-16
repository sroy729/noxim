/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the command line parser
 */

#include "NoximCmdLineParser.h"

void showHelp(char selfname[])
{
    cout << "Usage: " << selfname <<
	" [options]\nwhere [options] is one or more of the following ones:"
	<< endl;
    cout << "\t-help\t\tShow this help and exit" << endl;
    cout <<
	"\t-verbose N\tVerbosity level (1=low, 2=medium, 3=high, default off)"
	<< endl;
    cout <<
	"\t-trace FILENAME\tTrace signals to a VCD file named 'FILENAME.vcd' (default off)"
	<< endl;
    cout <<
	"\t-dimx N\t\tSet the mesh X dimension to the specified integer value (default "
	<< DEFAULT_MESH_DIM_X << ")" << endl;
    cout <<
	"\t-dimy N\t\tSet the mesh Y dimension to the specified integer value (default "
	<< DEFAULT_MESH_DIM_Y << ")" << endl;
    cout <<
	"\t-buffer N\tSet the buffer depth of each channel of the router to the specified integer value [flits] (default "
	<< DEFAULT_BUFFER_DEPTH << ")" << endl;
    cout <<
	"\t-size Nmin Nmax\tSet the minimum and maximum packet size to the specified integer values [flits] (default min="
	<< DEFAULT_MIN_PACKET_SIZE << ", max=" << DEFAULT_MAX_PACKET_SIZE
	<< ")" << endl;
    cout <<
	"\t-routing TYPE\tSet the routing algorithm to TYPE where TYPE is one of the following (default "
	<< DEFAULT_ROUTING_ALGORITHM << "):" << endl;
    cout << "\t\txy\t\tXY routing algorithm" << endl;
    cout << "\t\twestfirst\tWest-First routing algorithm" << endl;
    cout << "\t\tnorthlast\tNorth-Last routing algorithm" << endl;
    cout << "\t\tnegativefirst\tNegative-First routing algorithm" << endl;
    cout << "\t\toddeven\t\tOdd-Even routing algorithm" << endl;
    cout << "\t\tdyad T\t\tDyAD routing algorithm with threshold T" <<
	endl;
    cout << "\t\tfullyadaptive\tFully-Adaptive routing algorithm" << endl;
    cout <<
	"\t\ttable FILENAME\tRouting Table Based routing algorithm with table in the specified file"
	<< endl;
    cout <<
	"\t-sel TYPE\tSet the selection strategy to TYPE where TYPE is one of the following (default "
	<< DEFAULT_SELECTION_STRATEGY << "):" << endl;
    cout << "\t\trandom\t\tRandom selection strategy" << endl;
    cout << "\t\tbufferlevel\tBuffer-Level Based selection strategy" <<
	endl;
    cout << "\t\tnop\t\tNeighbors-on-Path selection strategy" << endl;
    cout <<
	"\t-pir R TYPE\t\tSet the packet injection rate to the specified real value [0..1] (default "
	<< DEFAULT_PACKET_INJECTION_RATE <<
	") and the time distribution of traffic to TYPE where TYPE is one of the following:"
	<< endl;
    cout << "\t\tpoisson\t\tMemory-less Poisson distribution (default)" <<
	endl;
    cout << "\t\tburst R\t\tBurst distribution with given real burstness"
	<< endl;
    cout <<
	"\t\tpareto on off r\tSelf-similar Pareto distribution with given real parameters (alfa-on alfa-off r)"
	<< endl;
    cout <<
	"\t\tcustom R\tCustom distribution with given real probability of retransmission"
	<< endl;
    cout <<
	"\t-traffic TYPE\tSet the spatial distribution of traffic to TYPE where TYPE is one of the following (default "
	<< DEFAULT_TRAFFIC_DISTRIBUTION << "'):" << endl;
    cout << "\t\trandom\t\tRandom traffic distribution" << endl;
    cout << "\t\ttranspose1\tTranspose matrix 1 traffic distribution" <<
	endl;
    cout << "\t\ttranspose2\tTranspose matrix 2 traffic distribution" <<
	endl;
    cout << "\t\tbitreversal\tBit-reversal traffic distribution" << endl;
    cout << "\t\tbutterfly\tButterfly traffic distribution" << endl;
    cout << "\t\tshuffle\t\tShuffle traffic distribution" << endl;
    cout <<
	"\t\ttable FILENAME\tTraffic Table Based traffic distribution with table in the specified file"
	<< endl;
    cout <<
	"\t-hs ID P\tAdd node ID to hotspot nodes, with percentage P (0..1) (Only for 'random' traffic)"
	<< endl;
    cout <<
	"\t-warmup N\tStart to collect statistics after N cycles (default "
	<< DEFAULT_STATS_WARM_UP_TIME << ")" << endl;
    cout <<
	"\t-seed N\t\tSet the seed of the random generator (default time())"
	<< endl;
    cout << "\t-detailed\tShow detailed statistics" << endl;
    cout <<
	"\t-volume N\tStop the simulation when either the maximum number of cycles has been reached or N flits have been delivered"
	<< endl;
    cout <<
	"\t-sim N\t\tRun for the specified simulation time [cycles] (default "
	<< DEFAULT_SIMULATION_TIME << ")" << endl << endl;
    cout <<
	"If you find this program useful please don't forget to mention in your paper Maurizio Palesi <mpalesi@diit.unict.it>"
	<< endl;
    cout <<
	"If you find this program useless please feel free to complain with Davide Patti <dpatti@diit.unict.it>"
	<< endl;
    cout <<
	"And if you want to send money please feel free to PayPal to Fabrizio Fazzino <fabrizio@fazzino.it>"
	<< endl;
}

void showConfig()
{
    cout << "Using the following configuration: " << endl;
    cout << "- verbose_mode = " << NoximGlobalParams::verbose_mode << endl;
    cout << "- trace_mode = " << NoximGlobalParams::trace_mode << endl;
    //  cout << "- trace_filename = " << NoximGlobalParams::trace_filename << endl;
    cout << "- mesh_dim_x = " << NoximGlobalParams::mesh_dim_x << endl;
    cout << "- mesh_dim_y = " << NoximGlobalParams::mesh_dim_y << endl;
    /***3D***/
	cout << "- mesh_dim_z = " << NoximGlobalParams::mesh_dim_z << endl; ////
	cout << "- buffer_depth = " << NoximGlobalParams::buffer_depth << endl;
    cout << "- max_packet_size = " << NoximGlobalParams::
	max_packet_size << endl;
    cout << "- routing_algorithm = " << NoximGlobalParams::
	routing_algorithm << endl;
    //  cout << "- routing_table_filename = " << NoximGlobalParams::routing_table_filename << endl;
	cout << "- Buffer_Allocation = " 	<< NoximGlobalParams::Buffer_Allocation 	<< endl; //Calvin add on 2015/07/27
	cout << "- Throt_Reset_Buffer = " 	<< NoximGlobalParams::Throt_Reset_Buffer 	<< endl;
	cout << "- TBDB_Type = " 			<< NoximGlobalParams::TBDB_Type 			<< endl;
    cout << "- selection_strategy = " 	<< NoximGlobalParams::selection_strategy 	<< endl;
    cout << "- packet_injection_rate = "<< NoximGlobalParams::packet_injection_rate << endl;
	cout << "- probability_of_retransmission = " << NoximGlobalParams::probability_of_retransmission << endl;
    cout << "- traffic_distribution = " << NoximGlobalParams::traffic_distribution 	<< endl;
    cout << "- simulation_time = "    	<< NoximGlobalParams::simulation_time    	<< endl;
    cout << "- stats_warm_up_time = " 	<< NoximGlobalParams::stats_warm_up_time 	<< endl;
    cout << "- rnd_generator_seed = " 	<< NoximGlobalParams::rnd_generator_seed 	<< endl;	
	cout << "- throttling_type = "    	<< NoximGlobalParams::throt_type         	<< endl;
	cout << "- dw_layer_sel = "       	<< NoximGlobalParams::dw_layer_sel       	<< endl;
	cout << "- prediciton_vol ="      	<< NoximGlobalParams::predict_vol        	<< endl;
	cout << "- TD_Type = " 			  	<< NoximGlobalParams::TD_Type 		   		<< endl;	//Dora added on 2017/4/19
}

void checkInputParameters()
{
    if (NoximGlobalParams::mesh_dim_x <= 1) {
	cerr << "Error: dimx must be greater than 1" << endl;
	exit(1);
    }

    if (NoximGlobalParams::mesh_dim_y <= 1) {
	cerr << "Error: dimy must be greater than 1" << endl;
	exit(1);
    }

    if (NoximGlobalParams::buffer_depth < 1) {
	cerr << "Error: buffer must be >= 1" << endl;
	exit(1);
    }

    if (NoximGlobalParams::min_packet_size < 2 ||
	NoximGlobalParams::max_packet_size < 2) {
	cerr << "Error: packet size must be >= 2" << endl;
	exit(1);
    }

    if (NoximGlobalParams::min_packet_size >
	NoximGlobalParams::max_packet_size) {
	cerr << "Error: min packet size must be less than max packet size"
	    << endl;
	exit(1);
    }

    if (NoximGlobalParams::routing_algorithm == INVALID_ROUTING) {
	cerr << "Error: invalid routing algorithm" << endl;
	exit(1);
    }

    if (NoximGlobalParams::selection_strategy == INVALID_SELECTION) {
	cerr << "Error: invalid selection policy" << endl;
	exit(1);
    }
	if (NoximGlobalParams::Throt_Reset_Buffer == INVALID_Throt_Reset_Buffer) {
	cerr << "Error: invalid Throt_Reset_Buffer" << endl;
	exit(1);
    }
	if (NoximGlobalParams::Buffer_Allocation == INVALID_BUFFER_ALLOCATION) {
	cerr << "Error: invalid Buffer_Allocation" << endl;
	exit(1);
    }
	if (NoximGlobalParams::TBDB_Type == INVALID_TBDB_Type) {
	cerr << "Error: invalid TBDB_Type" << endl;
	exit(1);
    }
    if (NoximGlobalParams::packet_injection_rate <= 0.0 ||
	NoximGlobalParams::packet_injection_rate > 1.0) {
	cerr <<
	    "Error: packet injection rate mmust be in the interval ]0,1]"
	    << endl;
	exit(1);
    }

    if (NoximGlobalParams::traffic_distribution == INVALID_TRAFFIC) {
	cerr << "Error: invalid traffic" << endl;
	exit(1);
    }

    for (unsigned int i = 0; i < NoximGlobalParams::hotspots.size(); i++) {
	if (NoximGlobalParams::hotspots[i].first > MAX_ID) {
	    cerr << "Error: hotspot node " << NoximGlobalParams::
		hotspots[i].first << " is invalid (out of range)" << endl;
	    exit(1);
	}
	if (NoximGlobalParams::hotspots[i].second < 0.0
	    && NoximGlobalParams::hotspots[i].second > 1.0) {
	    cerr <<
		"Error: hotspot percentage must be in the interval [0,1]"
		<< endl;
	    exit(1);
	}
    }

    if (NoximGlobalParams::stats_warm_up_time < 0) {
	cerr << "Error: warm-up time must be positive" << endl;
	exit(1);
    }

    if (NoximGlobalParams::simulation_time < 0) {
	cerr << "Error: simulation time must be positive" << endl;
	exit(1);
    }

    if (NoximGlobalParams::stats_warm_up_time >
	NoximGlobalParams::simulation_time) {
	cerr << "Error: warmup time must be less than simulation time" <<
	    endl;
	exit(1);
    }
	
	if (NoximGlobalParams::TD_Type == INVALID_TD_Type) { // Dora added on 2017/4/19
	cerr << "Error: invalid TD_Type" << endl;
	exit(1);
    }
}

//---------------------------------------------------------------------------

void parseCmdLine(int arg_num, char *arg_vet[])
{
    if (arg_num == 1)
	cout <<
	    "Running with default parameters (use '-help' option to see how to override them)"
	    << endl;
    else {
	for (int i = 1; i < arg_num; i++) {
	    if (!strcmp(arg_vet[i], "-help")) {
		showHelp(arg_vet[0]);
		exit(0);
	    } else if (!strcmp(arg_vet[i], "-init_throttle"))
		NoximGlobalParams::init_throttle = true;
		else if (!strcmp(arg_vet[i], "-DQN_init"))
		NoximGlobalParams::DQN_init = true;
		else if (!strcmp(arg_vet[i], "-DQN"))
		NoximGlobalParams::DQN = true;
	    else if (!strcmp(arg_vet[i], "-DQN_mon")) {
		NoximGlobalParams::DQN_mon = true;
		NoximGlobalParams::DQN_x = atoi(arg_vet[++i]);
		NoximGlobalParams::DQN_y = atoi(arg_vet[++i]);
		}
		else if(!strcmp(arg_vet[i], "-start"))
		NoximGlobalParams::start = true;
	    else if (!strcmp(arg_vet[i], "-verbose"))
		NoximGlobalParams::verbose_mode = atoi(arg_vet[++i]);
	    else if (!strcmp(arg_vet[i], "-trace")) {
		NoximGlobalParams::trace_mode = true;
		strcpy(NoximGlobalParams::trace_filename, arg_vet[++i]);
	    } else if (!strcmp(arg_vet[i], "-dimx"))
		NoximGlobalParams::mesh_dim_x = atoi(arg_vet[++i]);
	    else if (!strcmp(arg_vet[i], "-dimy"))
		NoximGlobalParams::mesh_dim_y = atoi(arg_vet[++i]);
		/***3D***/
		else if (!strcmp(arg_vet[i], "-dimz"))
		NoximGlobalParams::mesh_dim_z = atoi(arg_vet[++i]);
		/***3D***/
	    else if (!strcmp(arg_vet[i], "-buffer"))
		NoximGlobalParams::buffer_depth = atoi(arg_vet[++i]);
	    else if (!strcmp(arg_vet[i], "-size")) {
		NoximGlobalParams::min_packet_size = atoi(arg_vet[++i]);
		NoximGlobalParams::max_packet_size = atoi(arg_vet[++i]);
	    } else if (!strcmp(arg_vet[i], "-routing")) {
		char *routing = arg_vet[++i];
		if      (!strcmp(routing, "xyz"          ))
		    NoximGlobalParams::routing_algorithm = ROUTING_XYZ;
		else if (!strcmp(routing, "zxy"          ))
		    NoximGlobalParams::routing_algorithm = ROUTING_ZXY;
		else if (!strcmp(routing,"downward"      )) 
			NoximGlobalParams::routing_algorithm = ROUTING_DOWNWARD;
		else if (!strcmp(routing,"oe_downward"   ))
			NoximGlobalParams::routing_algorithm = ROUTING_ODD_EVEN_DOWNWARD;
		else if (!strcmp(routing,"oe_3d"         ))
			NoximGlobalParams::routing_algorithm = ROUTING_ODD_EVEN_3D;
		else if (!strcmp(routing,"oe_z"          )) 
			NoximGlobalParams::routing_algorithm = ROUTING_ODD_EVEN_Z;
		else if (!strcmp(routing, "westfirst"    ))
		    NoximGlobalParams::routing_algorithm = ROUTING_WEST_FIRST;
		else if (!strcmp(routing, "northlast"    ))
		    NoximGlobalParams::routing_algorithm = ROUTING_NORTH_LAST;
		else if (!strcmp(routing, "negativefirst"))
		    NoximGlobalParams::routing_algorithm = ROUTING_NEGATIVE_FIRST;
		else if (!strcmp(routing, "oddeven"      ))
		    NoximGlobalParams::routing_algorithm = ROUTING_ODD_EVEN;
		else if (!strcmp(routing, "fullyadaptive"))
		    NoximGlobalParams::routing_algorithm = ROUTING_FULLY_ADAPTIVE;
		else if (!strcmp(routing, "DLADR"        ))
		    NoximGlobalParams::routing_algorithm = ROUTING_DLADR;	
		else if (!strcmp(routing, "DLAR"         ))
		    NoximGlobalParams::routing_algorithm = ROUTING_DLAR;	
		else if (!strcmp(routing, "DLDR"         ))
		    NoximGlobalParams::routing_algorithm = ROUTING_DLDR;
		else if (!strcmp(routing, "VTBR"         )){
		    NoximGlobalParams::routing_algorithm = ROUTING_DLADR;
			NoximGlobalParams::dw_layer_sel = DW_ODWL_IPD;
			}
		else if (!strcmp(routing, "TTMRA"         )){
		    NoximGlobalParams::routing_algorithm = ROUTING_DLADR;
			NoximGlobalParams::dw_layer_sel = DW_BL;
			NoximGlobalParams::cascade_node = true;
			}
		else if (!strcmp(routing, "TAAR"         )){
		    NoximGlobalParams::routing_algorithm = ROUTING_DLADR;
			NoximGlobalParams::dw_layer_sel = DW_ODWL_IPD;
			NoximGlobalParams::cascade_node = true;
		}
		else if (!strcmp(routing, "TTABR"         )){
		    NoximGlobalParams::routing_algorithm = ROUTING_DLADR;
			NoximGlobalParams::dw_layer_sel = DW_ODWL_IPD;
			NoximGlobalParams::cascade_node = true;
			NoximGlobalParams::beltway = true;
			//NoximGlobalParams::br_sel = SEL_RCA;
		}
		else if (!strcmp(routing, "PTDBA"         )){ //Jimmy added on 2014.12.17
		    NoximGlobalParams::routing_algorithm = ROUTING_DLADR;
			NoximGlobalParams::dw_layer_sel = DW_ODWL_IPD;
			NoximGlobalParams::cascade_node = true;
			NoximGlobalParams::beltway = true;
			//NoximGlobalParams::br_sel = SEL_RCA;
		}
		else if (!strcmp(routing, "dyad"         )) {
		    NoximGlobalParams::routing_algorithm = ROUTING_DYAD;
		    NoximGlobalParams::dyad_threshold = atof(arg_vet[++i]);
		}
		else if (!strcmp(routing, "table")) {
		    NoximGlobalParams::routing_algorithm =
			ROUTING_TABLE_BASED;
		    strcpy(NoximGlobalParams::routing_table_filename,
			   arg_vet[++i]);
		    NoximGlobalParams::packet_injection_rate = 0;	
		} else
		    NoximGlobalParams::routing_algorithm = INVALID_ROUTING;
	    } else if (!strcmp(arg_vet[i], "-sel")) {
		char *selection = arg_vet[++i];
		if      (!strcmp(selection,"random"     ))
		    NoximGlobalParams::selection_strategy = SEL_RANDOM;
		else if (!strcmp(selection,"bufferlevel"))
		    NoximGlobalParams::selection_strategy = SEL_BUFFER_LEVEL;
		else if (!strcmp(selection,"hs"         )) 
			NoximGlobalParams::selection_strategy = SEL_HS;
		else if (!strcmp(selection,"nop"        ))
		    NoximGlobalParams::selection_strategy = SEL_NOP;
		else if (!strcmp(selection,"rca"        )) 
			NoximGlobalParams::selection_strategy = SEL_RCA;
		else if (!strcmp(selection,"proposed"        )) 
			NoximGlobalParams::selection_strategy = SEL_PROPOSED;
		else if(!strcmp(selection,"thermal"))  //Jimmy added on 2014.12.17
			NoximGlobalParams::selection_strategy = SEL_THERMAL;
		else
		    NoximGlobalParams::selection_strategy =
			INVALID_SELECTION;
	    }else if (!strcmp(arg_vet[i], "-dw_sel")) {
		char *dw_selection = arg_vet[++i];
		if (!strcmp(dw_selection, "bl"))
		    NoximGlobalParams::dw_layer_sel = DW_BL;
		else if (!strcmp(dw_selection, "odwl"))
		    NoximGlobalParams::dw_layer_sel = DW_ODWL;
		else if (!strcmp(dw_selection, "odwl_ipd"))
		    NoximGlobalParams::dw_layer_sel = DW_ODWL_IPD;	
		else if(!strcmp(dw_selection,"adwl")) 
			NoximGlobalParams::dw_layer_sel = DW_ADWL;
		else if (!strcmp(dw_selection, "ipd"))
		    NoximGlobalParams::dw_layer_sel = DW_IPD;
		else if(!strcmp(dw_selection,"vbdr")) 
			NoximGlobalParams::dw_layer_sel = DW_VBDR;
		else
		    NoximGlobalParams::dw_layer_sel = INVALID_SELECTION;
		} else if (!strcmp(arg_vet[i], "-vertical")) {
		char *selection = arg_vet[++i];
		if      (!strcmp(selection,"mesh"     ))
		    NoximGlobalParams::vertical_link = VERTICAL_MESH;
		else if (!strcmp(selection,"crossbar"))
		    NoximGlobalParams::vertical_link = VERTICAL_CROSSBAR;
		else
		    NoximGlobalParams::vertical_link = INVALID_SELECTION;
		} else if (!strcmp(arg_vet[i], "-cascade")) {
		    NoximGlobalParams::cascade_node = true;
		} else if (!strcmp(arg_vet[i], "-Mcascade")) {
		    NoximGlobalParams::cascade_node = true;
			NoximGlobalParams::Mcascade     = true;
			NoximGlobalParams::Mcascade_step= atoi(arg_vet[++i]);
		} else if (!strcmp(arg_vet[i], "-beltway")) {
		    NoximGlobalParams::beltway = true;
		} else if (!strcmp(arg_vet[i], "-multibeltway")) {
		    NoximGlobalParams::beltway = true;
			NoximGlobalParams::Mbeltway = true;
		} else if (!strcmp(arg_vet[i], "-ROCup")) {
		    NoximGlobalParams::ROC_UP = atoi(arg_vet[++i]);
		} else if (!strcmp(arg_vet[i], "-ROCdown")) {
		    NoximGlobalParams::ROC_DOWN = atoi(arg_vet[++i]);
		} else if (!strcmp(arg_vet[i], "-beltway_ratio")) {
		    NoximGlobalParams::beltway_ratio = atof(arg_vet[++i]);
		} else if (!strcmp(arg_vet[i], "-beltway_trigger")) {
		    NoximGlobalParams::beltway_trigger = atof(arg_vet[++i]);
		} else if (!strcmp(arg_vet[i], "-singlebeltway")) {
			NoximGlobalParams::beltway  = true;
			NoximGlobalParams::Sbeltway = true;
		    NoximGlobalParams::Sbeltway_ring = atoi(arg_vet[++i]);
		} else if (!strcmp(arg_vet[i], "-temp_off")) {
		    NoximGlobalParams::cal_temp = false;
		} else if (!strcmp(arg_vet[i], "-pir")) {
		NoximGlobalParams::packet_injection_rate =
		    atof(arg_vet[++i]);
		char *distribution = arg_vet[++i];
		if (!strcmp(distribution, "poisson"))
		    NoximGlobalParams::probability_of_retransmission =
			NoximGlobalParams::packet_injection_rate;
		else if (!strcmp(distribution, "burst")) {
		    float burstness = atof(arg_vet[++i]);
		    NoximGlobalParams::probability_of_retransmission =
			NoximGlobalParams::packet_injection_rate / (1 -
								    burstness);
		} else if (!strcmp(distribution, "pareto")) {
		    float Aon = atof(arg_vet[++i]);
		    float Aoff = atof(arg_vet[++i]);
		    float r = atof(arg_vet[++i]);
		    NoximGlobalParams::probability_of_retransmission =
			NoximGlobalParams::packet_injection_rate *
			pow((1 - r), (1 / Aoff - 1 / Aon));
		} else if (!strcmp(distribution, "custom"))
		    NoximGlobalParams::probability_of_retransmission =
			atof(arg_vet[++i]);
	    } else if (!strcmp(arg_vet[i], "-traffic")) {
		char *traffic = arg_vet[++i];
		if (!strcmp(traffic, "random"))//0
		    NoximGlobalParams::traffic_distribution =
			TRAFFIC_RANDOM;		
		else if (!strcmp(traffic, "transpose1"))//1
		    NoximGlobalParams::traffic_distribution =
			TRAFFIC_TRANSPOSE1;
		else if (!strcmp(traffic, "transpose2"))//2
		    NoximGlobalParams::traffic_distribution =
			TRAFFIC_TRANSPOSE2;
		else if (!strcmp(traffic, "bitreversal"))//5
		    NoximGlobalParams::traffic_distribution =
			TRAFFIC_BIT_REVERSAL;
		else if (!strcmp(traffic, "butterfly"))//7
		    NoximGlobalParams::traffic_distribution =
			TRAFFIC_BUTTERFLY;
		else if (!strcmp(traffic, "shuffle"))//6
		    NoximGlobalParams::traffic_distribution =
			TRAFFIC_SHUFFLE;
		else if (!strcmp(traffic, "table")) {//4
		    NoximGlobalParams::traffic_distribution =
			TRAFFIC_TABLE_BASED;
		    strcpy(NoximGlobalParams::traffic_table_filename,
			   arg_vet[++i]);
		} else
		    NoximGlobalParams::traffic_distribution =
			INVALID_TRAFFIC;
	    } else if (!strcmp(arg_vet[i], "-hs")) {
		int node = atoi(arg_vet[++i]);
		double percentage = atof(arg_vet[++i]);
		pair < int, double >t(node, percentage);
		NoximGlobalParams::hotspots.push_back(t);
	    } else if (!strcmp(arg_vet[i], "-warmup"))
		NoximGlobalParams::stats_warm_up_time = atoi(arg_vet[++i]);
	    else if (!strcmp(arg_vet[i], "-seed"))
		NoximGlobalParams::rnd_generator_seed = atoi(arg_vet[++i]);
		else if (!strcmp(arg_vet[i], "-clean"))
		NoximGlobalParams::clean_stage_time   = atoi(arg_vet[++i]);
	    else if (!strcmp(arg_vet[i], "-detailed"))
		NoximGlobalParams::detailed = true;
	    else if (!strcmp(arg_vet[i], "-volume"))
		NoximGlobalParams::max_volume_to_be_drained =
		    atoi(arg_vet[++i]);
		else if (!strcmp(arg_vet[i], "-buffer_alloc"))
		NoximGlobalParams::buffer_alloc = true;
	    else if (!strcmp(arg_vet[i], "-sim"))
		NoximGlobalParams::simulation_time = atoi(arg_vet[++i]);
		else if(!strcmp(arg_vet[i],"-predict"))
		NoximGlobalParams::predict_vol = atoi(arg_vet[++i]);
		else if (!strcmp(arg_vet[i], "-nibuffer"))
		NoximGlobalParams::buffer_depth = atoi(arg_vet[++i]);
		else if (!strcmp(arg_vet[i], "-tquota"))
		NoximGlobalParams::static_quota = atoi(arg_vet[++i]);
		else if (!strcmp(arg_vet[i], "-tla"))
		NoximGlobalParams::tla_support = atoi(arg_vet[++i]);
		else if(!strcmp(arg_vet[i],"-selinfo"))// traffic throttling 
		{
			char *selinfo = arg_vet[++i];
			if(!strcmp(selinfo,"buf")) NoximGlobalParams::selection_info = BUF;
  			//if(!strcmp(arg_vet[i],"buf")) NoximGlobalParams::selection_info = BUF;
			else NoximGlobalParams::selection_info = BUF_SERVER;
		}
		else if (!strcmp(arg_vet[i], "-tsh")) //Jimmy added on 2012.11.06
		NoximGlobalParams::temp_trigger = atof(arg_vet[++i]);
		else if (!strcmp(arg_vet[i], "-open_TBDB")) 
		{
		char *open_TBDB = arg_vet[++i];
		if (!strcmp(open_TBDB, "open"))
		    NoximGlobalParams::Buffer_Allocation =   //Calvin add on 2015/07/27
		BUFFER_ALLOCATION;	
		else if (!strcmp(open_TBDB, "close"))
		    NoximGlobalParams::Buffer_Allocation =   //Calvin add on 2015/07/27
		NO_BUFFER_ALLOCATION;
		else NoximGlobalParams::Buffer_Allocation =   //Calvin add on 2015/07/27 
		INVALID_BUFFER_ALLOCATION;
		}
		else if (!strcmp(arg_vet[i], "-buf_reset")) 
		{
		char *TRB = arg_vet[++i];
		if (!strcmp(TRB, "on"))
		    NoximGlobalParams::Throt_Reset_Buffer =   
		DO_Throt_Reset_Buffer;	
		else if (!strcmp(TRB, "off"))
		    NoximGlobalParams::Throt_Reset_Buffer =   
		NO_Throt_Reset_Buffer;
		else NoximGlobalParams::Throt_Reset_Buffer =   
		INVALID_Throt_Reset_Buffer;
		}
		else if (!strcmp(arg_vet[i], "-choose")) 
		{
		char *TBDBT = arg_vet[++i];
		if (!strcmp(TBDBT, "RTI"))
		    NoximGlobalParams::TBDB_Type =   
		TBDB_Type_Original;	
		else if (!strcmp(TBDBT, "PRE"))
		    NoximGlobalParams::TBDB_Type =   
		TBDB_Type_PRE;
		else if (!strcmp(TBDBT, "GAME"))
		    NoximGlobalParams::TBDB_Type =   
		TBDB_Type_GAME;
		else if (!strcmp(TBDBT, "GPRE"))
		    NoximGlobalParams::TBDB_Type =   
		TBDB_Type_GPRE;
		else NoximGlobalParams::TBDB_Type =    
		INVALID_TBDB_Type;
		}
		else if(!strcmp(arg_vet[i], "-dfs"))
		NoximGlobalParams::dfs_level = atof(arg_vet[++i]); //Jimmy added on 2013.03.06
	  else if (!strcmp(arg_vet[i], "-sen_alloc")) /*Jimmy added on 2016.03.19 */
		{
			char *sen_alloc = arg_vet[++i];
			if  (!strcmp(sen_alloc, "normal" )) NoximGlobalParams::sensor_allocate = SENSOR_ALLOCATE_NORMAL;
			else if (!strcmp(sen_alloc, "even")) NoximGlobalParams::sensor_allocate = SENSOR_ALLOCATE_EVEN;
			else if (!strcmp(sen_alloc, "odd")) NoximGlobalParams::sensor_allocate = SENSOR_ALLOCATE_ODD;
			else if ((!strcmp(sen_alloc, "table"))){
				NoximGlobalParams::sensor_allocate = SENSOR_ALLOCATE_TABLE;
				strcpy(NoximGlobalParams::sensor_table_filename, arg_vet[++i]);
			}
			else NoximGlobalParams::sensor_allocate = INVALID_SENSOR_ALLOCATE;
		}
		else if (!strcmp(arg_vet[i], "-temp_recover")) /*Jimmy added on 2016.03.20 */
		{
			char *sen_alloc = arg_vet[++i];
			if  (!strcmp(sen_alloc, "linear" )){
				NoximGlobalParams::temp_recover = TEMP_RECTOR_LINEAR;
				strcpy(NoximGlobalParams::rector_linear_coef_filename, arg_vet[++i]);
			}
			else if (!strcmp(sen_alloc, "regression" )){                                      //Andy added
				NoximGlobalParams::temp_recover = TEMP_RECTOR_REGRESSION;
				strcpy(NoximGlobalParams::rector_regression_coef_filename, arg_vet[++i]);
				strcpy(NoximGlobalParams::rector_regression_coef1_filename, arg_vet[++i]);
				strcpy(NoximGlobalParams::rector_regression_ref_filename, arg_vet[++i]);
			}
			else NoximGlobalParams::temp_recover = INVALID_TEMP_RECTOR;
		}	
   
	    else if (!strcmp(arg_vet[i], "-TD")) // Dora Dselction Thermal Delay type added on 2017/4/19 
		{
			char *TD = arg_vet[++i];
			if (!strcmp(TD, "Original"))
				NoximGlobalParams::TD_Type = TD_Type_Original;	
			else if (!strcmp(TD, "GTDAR"))
				NoximGlobalParams::TD_Type = TD_Type_GTDAR;		
			else NoximGlobalParams::TD_Type = INVALID_TD_Type;
		}
		else if(!strcmp(arg_vet[i++],"-throt"))// traffic throttling 
		{
  			if(!strcmp(arg_vet[i],"normal")) NoximGlobalParams::throt_type = THROT_NORMAL;
			else if(!strcmp(arg_vet[i],"test")) NoximGlobalParams::throt_type = THROT_TEST;
  			else if(!strcmp(arg_vet[i],"global")) NoximGlobalParams::throt_type = THROT_GLOBAL;
 			else if(!strcmp(arg_vet[i],"distributed")) NoximGlobalParams::throt_type = THROT_DISTRIBUTED;
 			else if(!strcmp(arg_vet[i],"vertical")) NoximGlobalParams::throt_type = THROT_VERTICAL;
			else if(!strcmp(arg_vet[i],"tavt")) NoximGlobalParams::throt_type = THROT_TAVT;
  			else NoximGlobalParams::throt_type = NOT_VALID;
      	}// end traffic throttling
		
		
		else {
		cerr << "Error: Invalid option: " << arg_vet[i] << endl;
		exit(1);
	    }
	}
    }

    checkInputParameters();

    // Show configuration
    if (NoximGlobalParams::verbose_mode > VERBOSE_OFF)
	showConfig();
}
