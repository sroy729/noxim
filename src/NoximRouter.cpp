/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the router
 */

#include "NoximRouter.h"
extern int throttling[10][10][4];
extern int reShot[10][10][4]; //Jimmy added on 2017.05.29
int HT_node = 2;
int HT_node2 = 37;

/////////////////////////shubham////////////////////////////////////////////////
vector<pair <int,int>> the_list;
extern bool ht_tamering = true;
int NoximRouter::randInti(int min, int max)
{
    return min +
	(int) ((double) (max - min + 1) * rand() / (RAND_MAX + 1.0));
}

int NoximRouter::map_fun(int i)
{ 
  switch(i)
  {
    case 2: return 50;
    case 6: return 46;
  }
  return i;
}
/////////////////////////////shubham////////////////////////////////////////////

void NoximRouter::rxProcess(){
	int i,j,k;
	if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"Router[" << local_id << "]-Rx ( verbose_mode "<< NoximGlobalParams::verbose_mode <<")"<<endl;
    if (reset.read()) {
		// Clear outputs and indexes of receiving protocol
		for ( i = 0; i < DIRECTIONS + 2; i++){ 
			ack_rx[i].write(1);
			routed_flits[i] = 0;
			buffer[i].Clean();
		}
		reservation_table.clear();
		local_drained  = 0;
		routed_DWflits = 0;
		routed_packets = 0;
    }
	else if((getCurrentCycleNum() % (int) (TEMP_REPORT_PERIOD))==0){//////////////////
		for ( i = 0; i < DIRECTIONS + 2; i++){ 
			ack_rx[i].write(1);
			//routed_flits[i] = 0;
			//buffer[i].Clean();
		}
		reservation_table.clear();
	}
	else {
		// For each channel decide if a new flit can be accepted
		// This process simply sees a flow of incoming flits. All arbitration
		// and wormhole related issues are addressed in the txProcess()

		for ( i = 0; i < DIRECTIONS + 2; i++) {
			// To accept a new flit, the following conditions must match:
			// 1) there is an incoming request
			// 2) there is a free slot in the input buffer of direction i
			if ( (req_rx[i].read()==1) && ack_rx[i].read()==1 ){
				
				if ( !_emergency ){//check throttling 
					NoximFlit received_flit = flit_rx[i].read();
					if (NoximGlobalParams::verbose_mode > VERBOSE_OFF   && local_id == 17) {
						//cout << getCurrentCycleNum() << ": Router[" << local_id << "], Input[" << i
						//<< "], Received flit: " << received_flit << endl;
					}
					received_flit.waiting_cnt = getCurrentCycleNum();
					// if(received_flit.tampered == true){
					// 	cnt_tampered[i]++;
					// }					
					// Store the incoming flit in the circular buffer
					/////////////////////////////////////////////////////////////////shubham///////////////////////
					int temp, temp_dst;
					temp_dst = received_flit.dst_id;
					
					// do{
					// 	temp = randInti(0,63);
					// }while(temp== HT_node && temp==received_flit.dst_id);
					double cycle_count = getCurrentCycleNum();
					// if (cycle_count > 5000){
					// 	cout<<cycle_count<<"Tamering off"<<endl;
					// 	ht_tamering == false;
					// }
					if (cycle_count < 5000 ){
						temp = map_fun(randInti(0,63));
						if(local_id == HT_node || local_id == HT_node2 ){
							// received_flit.temp_id_ht = received_flit.dst_id;
							if(received_flit.flit_type == FLIT_TYPE_HEAD){
								the_list.push_back(make_pair(received_flit.timestamp,temp));
							}
							for(int i=0;i<the_list.size();i++){
								if(received_flit.timestamp == the_list[i].first){
									received_flit.dst_id = the_list[i].second;
									received_flit.tampered = true ;								
								}
							}
							
							// cout<<"ULTRA : port_packet_count[0] "<<port_packet_count[0]<<"  port_packet_count[1] "<<port_packet_count[1]
							// <<"  port_packet_count[2] "<<port_packet_count[2]<<"  port_packet_count[3] "<<port_packet_count[3]
							// <<"  port_packet_count[4] "<<port_packet_count[4]<<"  port_packet_count[5] "<<port_packet_count[5]
							// <<"  port_packet_count[6] "<<port_packet_count[6]<<endl;
							// cout<<"after:"<<"\t"<<received_flit.src_id<<"\t"<<temp_dst<<"\t"<<received_flit.dst_id<<"\n\n";
							cout<<cycle_count<<"after(finally):"<<received_flit<<endl;


						}
					}
					 cout<<cycle_count/*<<"after(nonono):"<<received_flit*/<<endl; 
					/////////////////////////////////////////////////////////////////shubham///////////////////////

					//////////////////////////////////////////////////////////////
					buffer[i].Push(received_flit);
					if(received_flit.tampered == true && (local_id != HT_node || local_id != HT_node2) ){
						cnt_tampered[i]++;
					}

					//if( received_flit.type == TAIL )
					//packet_num[i]++;
					routed_flits[i]++;
					
					if( received_flit.routing_f != ROUTING_WEST_FIRST )
						routed_DWflits++;
						
					if( received_flit.flit_type == FLIT_TYPE_HEAD )
						routed_packets++;
						
					// Incoming flit
					if ( i != DIRECTION_UP && i != DIRECTION_DOWN){
						stats.power.RouterRxLateral();//Msint + QueuesNDataPath + Clocking

					}
				}
			}
			ack_rx[i].write((!buffer[i].IsFull()) && (!_emergency) );
		}
    }
	 
	if ( !_emergency)//if router not throttle
		stats.power.TileLeakage();//Router + FPMAC + MEM
	if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"Router[" << local_id << "]-Rx ends"<<endl;
}

void NoximRouter::txProcess(){
	
	if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"Router[" << local_id << "]-Tx"<<endl;
    if (reset.read()) {
		// Clear outputs and indexes of transmitting protocol
		for (int i = 0; i < DIRECTIONS + 2; i++){// 0~6 DIRECTIONS + 2 = 8 
			req_tx[i].write(0);
			waiting[i]     = 0;
		}
		_total_waiting  = 0;
		tran_permit = 0;
		
    }
	else if((getCurrentCycleNum() % (int) (TEMP_REPORT_PERIOD))==0){////////////////
		for (int i = 0; i < DIRECTIONS + 2; i++){// 0~6 DIRECTIONS + 2 = 8 
			req_tx[i].write(0);
		}
	}
	else {
		/**/
		
		in[0] = buffer[0].Size();
		in[1] = buffer[1].Size();
		in[2] = buffer[2].Size();
		in[3] = buffer[3].Size();
		in[4] = buffer[6].Size();
		/*forward*/
		//eva_NN(in);
		//tar_NN(in);
		//_emergency_level = node_no+1;

		/**/
		//if(local_id == 17)
			//cout<< tran_permit<<endl;
		if(_emergency_level != 0){ //the current router is potencial overheat (Jimmy modified on 2011.11.15)...throttle 1/2
			
			if(tran_permit==0){ //the pkt will be transmitted when Trans_permit=0 
				trans_fun();
			}
			else{
				//cout << "Not transmit" <<endl;
				for(int o=0; o<DIRECTIONS+2; o++)
					if ( 1 == ack_tx[o].read())// && req_tx[o].read() == 1)
						req_tx[o].write(0);
			}
			tran_permit = (tran_permit==(_emergency_level-1)) ? 0 : (tran_permit+1);
			//if(_emergency_level==2) tran_permit = (tran_permit==1) ? 0 : (tran_permit+1);
			//else if(_emergency_level==3) tran_permit = (tran_permit==2) ? 0 : (tran_permit+1);
		}
		else{ //the current router is cool
				if ( _emergency_trigger_flag ) {
					if(NoximGlobalParams::Throt_Reset_Buffer == DO_Throt_Reset_Buffer){
						for (int i = 0; i < DIRECTIONS + 1; i++) {
							//printf("Router id: %d ---- >buffer[%d]'s previous-buffer size: %d, ", local_id, i, buffer[i].GetMaxBufferSize());
							buffer[i].SetMaxBufferSize(_man_set_max_buffer_size);
							//printf("now: %d\n",buffer[i].GetMaxBufferSize());
						}
						
						_emergency_trigger_flag = false;
					}
				}
				trans_fun();
		
		}	
    }
	
}

void NoximRouter::trans_fun()
{
	// 1st phase: Reservation
		for (int j = 0; j < DIRECTIONS + 2; j++) {
			int i = (start_from_port + j) % (DIRECTIONS + 2);
	
			if (!buffer[i].IsEmpty()) {
				NoximFlit flit = buffer[i].Front();	
/////////////////////////////////////////////////////



				//////////////////////////////////////
				if (flit.flit_type == FLIT_TYPE_HEAD) {
					// prepare data for routing
					NoximRouteData route_data;
					route_data.current_id = local_id;
					route_data.src_id     = (flit.arr_mid)?flit.mid_id:flit.src_id;
					route_data.dst_id     = (flit.arr_mid)?flit.dst_id:flit.mid_id;
					route_data.dir_in     = i;
					route_data.routing    = flit.routing_f;
					route_data.DW_layer   = flit.DW_layer;
					route_data.arr_mid    = flit.arr_mid;
					route_data.arr_mid    = flit.arr_mid;
					/*if( flit.routing_f < 0 ){
						cout<<getCurrentCycleNum()<<":"<<flit<<endl;
						cout<<"flit.current_id"<<"="<<local_id       <<id2Coord(route_data.current_id)<<endl; 
						cout<<"flit.src_id    "<<"="<<flit.src_id    <<id2Coord(flit.src_id          )<<endl;
						cout<<"flit.mid_id    "<<"="<<flit.mid_id    <<id2Coord(flit.mid_id          )<<endl; 		
						cout<<"flit.dst_id    "<<"="<<flit.dst_id    <<id2Coord(flit.dst_id          )<<endl; 
						cout<<"flit.dir_in    "<<"="<<i              <<endl; 
						cout<<"flit.routing   "<<"="<<flit.routing_f <<endl; 
						cout<<"flit.DW_layer  "<<"="<<flit.DW_layer  <<endl; 
						cout<<"flit.arr_mid   "<<"="<<flit.arr_mid   <<endl; 
						assert(false);
					}*/
					
					
					if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
						cout<<"Before route:"<<flit;
					int o = route(route_data);
					if (reservation_table.isAvailable(o) // && packet_num[i] > 0
					) {
						reservation_table.reserve(i, o);
						if (NoximGlobalParams::verbose_mode > VERBOSE_OFF && local_id == 17) {
							cout << getCurrentCycleNum()
							<< ": Router[" << local_id
							<< "], Input[" << i << "] (" << buffer[i].
							Size() << " flits)" << ", reserved Output["
							<< o << "], flit: " << flit << endl;
						}
						stats.power.ArbiterNControl();	
					}
				}
			}
		}
		start_from_port++;
	
		// 2nd phase: Forwarding
		for(int o=0; o<DIRECTIONS+2; o++)
			if ( 1 == ack_tx[o].read() )req_tx[o].write(0);
			
		for (int i = 0; i < DIRECTIONS + 2; i++) {
			if (!buffer[i].IsEmpty() && !_emergency ) {
				NoximFlit flit = buffer[i].Front();
				
				// if(local_id == 2){
				// 	flit.tampered = true;
				// }				

				int o = reservation_table.getOutputPort(i);
				if (o != NOT_RESERVED) {
					if ( 1 == ack_tx[o].read()) {
						if (NoximGlobalParams::verbose_mode > VERBOSE_OFF  && local_id == 17) {
							cout << getCurrentCycleNum()
							<< ": Router[" << local_id
							<< "], Input[" << i <<
							"] forward to Output[" << o << "], flit: "
							<< flit << endl;
						}
						// if(flit.tampered == true){
						// 	cnt_tampered[i]++;
						// }						

						flit_tx[o].write(flit);
						req_tx [o].write(!buffer[i].IsEmpty());
						buffer [i].Pop  ();
						waiting[i] = 0 ;
						
						if( o != DIRECTION_UP && o != DIRECTION_DOWN )
							stats.power.Router2Lateral(); // Crossbar() + Links()				
						
						if (flit.flit_type == FLIT_TYPE_TAIL)
							reservation_table.release(o);
							
						//if ( (flit.flit_type == FLIT_TYPE_HEAD) && ( flit.src_id == 53) && ( flit.dst_id == 25 ) )
						// if ( ( flit.src_id == 14) && ( flit.dst_id == 49 ) )
							_total_waiting += getCurrentCycleNum() - flit.waiting_cnt;
			
						// Update stats
						if (o == DIRECTION_LOCAL) {
							stats.receivedFlit(getCurrentCycleNum(), flit);
							stats.power.Router2Local();//DualFpmacs + Imem + Dmem + RF + ClockDistribution
							if (NoximGlobalParams::max_volume_to_be_drained) {
								if (drained_volume >= NoximGlobalParams::max_volume_to_be_drained)
									sc_stop();
								else {
									drained_volume++;
									local_drained++;
								}
							}
						}
					}
					else{//head of line blocking
						waiting[i]++;
					}
				}
				else{
					//count waiting time
					waiting[i]++;
				}
			}
		}
}

NoximNoP_data NoximRouter::getCurrentNoPData() const
{
    NoximNoP_data NoP_data;

    for (int j = 0; j < DIRECTIONS; j++) {
		NoP_data.channel_status_neighbor[j].free_slots =
			free_slots_neighbor[j].read();
		NoP_data.channel_status_neighbor[j].available =
			(reservation_table.isAvailable(j));
    }

    NoP_data.sender_id = local_id;

    return NoP_data;
}

void NoximRouter::bufferMonitor()
{	
	int i;
    if (reset.read()) {
		for ( i = 0; i < DIRECTIONS + 1; i++)
			free_slots[i].write(buffer[i].GetMaxBufferSize());
		//for(int i=0; i<4; i++)	
		//	free_slots_PE[i].write( free_slots_neighbor[i]);
	}

	else {
		// update current input buffers level to neighbors
		for ( i = 0; i < DIRECTIONS + 1; i++)
			free_slots[i].write(buffer[i].getCurrentFreeSlots());
	    // NoP selection: send neighbor info to each direction 'i'
			NoximNoP_data current_NoP_data = getCurrentNoPData();

	    for ( i = 0; i < DIRECTIONS; i++)
			NoP_data_out[i].write(current_NoP_data);
			vertical_free_slot_out.write(current_NoP_data);		
		
		//for(int i=0; i<4; i++)	
		//	free_slots_PE[i].write( free_slots_neighbor[i] );
		//for(int i=0; i<8; i++)
		//	RCA_PE[i].write( RCA_data_in[i] );		
		//for(int i=0; i<4; i++)		
		//	NoP_PE[i].write( NoP_data_in[i] );		
    }
}

vector < int >NoximRouter::routingFunction(const NoximRouteData & route_data)
{
    NoximCoord position  = id2Coord(route_data.current_id);
    NoximCoord src_coord = id2Coord(route_data.src_id    );
    NoximCoord dst_coord = id2Coord(route_data.dst_id    );
    int dir_in           = route_data.dir_in  ;
	int routing          = route_data.routing ;  
	int DW_layer         = route_data.DW_layer;
	int arr_mid          = route_data.arr_mid ;
	
    switch (NoximGlobalParams::routing_algorithm) {
	/***ACCESS IC LAB's Routing Algorithm***/
	case ROUTING_XYZ:
      return routingXYZ(position, dst_coord);
      
    case ROUTING_ZXY:
      return routingZXY(position, dst_coord);
           
    case ROUTING_DOWNWARD:
    	return routingDownward(position, src_coord, dst_coord);
	
	case ROUTING_ODD_EVEN_DOWNWARD:
      return routingOddEven_Downward(position, src_coord, dst_coord, route_data);

    case ROUTING_ODD_EVEN_3D:
      return routingOddEven_3D(position, src_coord, dst_coord);

	case ROUTING_ODD_EVEN_Z:
	  return routingOddEven_Z(position, src_coord, dst_coord);

	case ROUTING_WEST_FIRST:
		return routingWestFirst(position, dst_coord);

    case ROUTING_NORTH_LAST:
		return routingNorthLast(position, dst_coord);

    case ROUTING_NEGATIVE_FIRST:
		return routingNegativeFirst(position, dst_coord);

    case ROUTING_ODD_EVEN:
		return routingOddEven(position, src_coord, dst_coord);

    case ROUTING_DYAD:
		return routingDyAD(position, src_coord, dst_coord);

    case ROUTING_FULLY_ADAPTIVE:
		return routingFullyAdaptive(position, dst_coord);

    case ROUTING_TABLE_BASED:
		return routingTableBased( position, dir_in, dst_coord);
	
	case ROUTING_DLADR:
		return routingDLADR( position, src_coord , dst_coord,routing, DW_layer);
	
	case ROUTING_DLAR:
		return routingDLAR( position, src_coord , dst_coord,routing, DW_layer);
	
	case ROUTING_DLDR:
		return routingDLDR( position, src_coord , dst_coord,routing, DW_layer);
    default:
	assert(false);
    }

    // something weird happened, you shouldn't be here
    return (vector < int >) (0);
}

int NoximRouter::route(const NoximRouteData & route_data)
{
    if (route_data.dst_id == local_id && route_data.arr_mid)
		return DIRECTION_LOCAL;
	else if ( (route_data.dst_id == local_id && !route_data.arr_mid))
		return DIRECTION_SEMI_LOCAL;
	vector < int >candidate_channels = routingFunction(route_data);

	for(int i = candidate_channels.size() - 1  ; i >= 0 ; i--){
			if ( candidate_channels[i] < 4 ) //for lateral direction
				if ( on_off_neighbor[ candidate_channels[i] ].read() == 1 ){//if the direction is throttled
					candidate_channels.erase( candidate_channels.begin() + i);//then erase that way	
			}
	}
	if( candidate_channels.size() == 0 ){
		candidate_channels.push_back(DIRECTION_DOWN);
		}
    return selectionFunction(candidate_channels, route_data);
}

void NoximRouter::NoP_report() const
{
    NoximNoP_data NoP_tmp;
    cout << getCurrentCycleNum() << ": Router[" << local_id << "] NoP report: " << endl;

    for (int i = 0; i < DIRECTIONS; i++) {
	NoP_tmp = NoP_data_in[i].read();
	if (NoP_tmp.sender_id != NOT_VALID)
	    cout << NoP_tmp;
    }
}

void NoximRouter::RCA_Aggregation()
{
	int i,j;
	if(reset.read()){
		for(i=0; i < DIRECTIONS; i++)
			monitor_out[i].write(0);

		buffer_util = 0;

	}
	else{
		int RCA_data_tmp;
		int eff_service_time;
		int throt_status;
		
		NoximCoord position = id2Coord(local_id);
		throt_status = throttling[position.x][position.y][position.z];
		
		if(NoximGlobalParams::selection_info == BUF) eff_service_time = 1;
		else{
			if(throt_status == 0) eff_service_time = 1; //non-throttled node
			else if(throt_status == 1) eff_service_time = 100000000; //fully throttled node
			else eff_service_time = 1/(1-(1/throt_status));
		}
	

		if( NoximGlobalParams::routing_algorithm == ROUTING_ODD_EVEN_Z) {            //Restriction of OE routing is considered
			//N0
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[0].getCurrentFreeSlots())*2*32 + RCA_data_in[5].read() + RCA_data_in[6].read())/4;
			//cout << "\tN0: " << RCA_data_tmp;
			RCA_data_out[0].write(RCA_data_tmp);
			//N1
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[0].getCurrentFreeSlots())*2*32 + RCA_data_in[4].read() + RCA_data_in[3].read())/4;
			//cout << "\tN1: " << RCA_data_tmp;
			RCA_data_out[1].write(RCA_data_tmp);
			
			//E0
			if(position.x % 2 == 0)
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[1].getCurrentFreeSlots())*2*32 + RCA_data_in[7].read() + RCA_data_in[0].read())/4;
			else
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[1].getCurrentFreeSlots())*2*32 + RCA_data_in[7].read() + 0)/4;
			//cout << "\tE0: " << RCA_data_tmp;
			RCA_data_out[2].write(RCA_data_tmp);
			//E1
			if(position.x % 2 == 0)
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[1].getCurrentFreeSlots())*2*32 + RCA_data_in[6].read() + RCA_data_in[5].read())/4;
			else
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[1].getCurrentFreeSlots())*2*32 + RCA_data_in[6].read() + 0)/4;
			//cout << "\tE1: " << RCA_data_tmp;
			RCA_data_out[3].write(RCA_data_tmp);
			
			//S0
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[2].getCurrentFreeSlots())*2*32 + RCA_data_in[1].read() + RCA_data_in[2].read())/4;
			//cout << "\tS0: " << RCA_data_tmp;
			RCA_data_out[4].write(RCA_data_tmp);
			//S1
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[2].getCurrentFreeSlots())*2*32 + RCA_data_in[0].read() + RCA_data_in[7].read())/4;
			//cout << "\tS1: " << RCA_data_tmp;
			RCA_data_out[5].write(RCA_data_tmp);
			
			//W0
			if(position.x % 2 == 1)
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[3].getCurrentFreeSlots())*2*32 + RCA_data_in[3].read() + RCA_data_in[4].read())/4;
			else
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[3].getCurrentFreeSlots())*2*32 + RCA_data_in[3].read() + 0)/4;
			//cout << "\tW0: " << RCA_data_tmp;
			RCA_data_out[6].write(RCA_data_tmp);
			//W1
			if(position.x % 2 == 1)
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[3].getCurrentFreeSlots())*2*32 + RCA_data_in[2].read() + RCA_data_in[1].read())/4;
			else
				RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[3].getCurrentFreeSlots())*2*32 + RCA_data_in[2].read() + 0)/4;
			//cout << "\tW1: " << RCA_data_tmp;
			RCA_data_out[7].write(RCA_data_tmp);
		}
		else {                              //Restriction of OE routing is NOT considered
			//N0
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[0].getCurrentFreeSlots())*2*32 + RCA_data_in[5].read() + RCA_data_in[6].read())/4;
			//cout << "\tN0: " << RCA_data_tmp;
			RCA_data_out[0].write(RCA_data_tmp);
			//N1
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[0].getCurrentFreeSlots())*2*32 + RCA_data_in[4].read() + RCA_data_in[3].read())/4;
			//cout << "\tN1: " << RCA_data_tmp;
			RCA_data_out[1].write(RCA_data_tmp);
			//E0
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[1].getCurrentFreeSlots())*2*32 + RCA_data_in[7].read() + RCA_data_in[0].read())/4;
			//cout << "\tE0: " << RCA_data_tmp;
			RCA_data_out[2].write(RCA_data_tmp);
			//E1
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[1].getCurrentFreeSlots())*2*32 + RCA_data_in[6].read() + RCA_data_in[5].read())/4;
			//cout << "\tE1: " << RCA_data_tmp;
			RCA_data_out[3].write(RCA_data_tmp);
			//S0
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[2].getCurrentFreeSlots())*2*32 + RCA_data_in[1].read() + RCA_data_in[2].read())/4;
			//cout << "\tS0: " << RCA_data_tmp;
			RCA_data_out[4].write(RCA_data_tmp);
			//S1
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[2].getCurrentFreeSlots())*2*32 + RCA_data_in[0].read() + RCA_data_in[7].read())/4;
			//cout << "\tS1: " << RCA_data_tmp;
			RCA_data_out[5].write(RCA_data_tmp);
			//W0
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[3].getCurrentFreeSlots())*2*32 + RCA_data_in[3].read() + RCA_data_in[4].read())/4;
			//cout << "\tW0: " << RCA_data_tmp;
			RCA_data_out[6].write(RCA_data_tmp);
			//W1
			RCA_data_tmp = (eff_service_time*(NoximGlobalParams::buffer_depth - buffer[3].getCurrentFreeSlots())*2*32 + RCA_data_in[2].read() + RCA_data_in[1].read())/4;
			//cout << "\tW1: " << RCA_data_tmp;
			RCA_data_out[7].write(RCA_data_tmp);
		}
	}
}
//---------------------------------------------------------------------------

int NoximRouter::NoPScore(const NoximNoP_data & nop_data,
			  const vector < int >&nop_channels, int candidate_id) const
{
    int score = 0;
	int eff_wait_time = 0;
	int eff_service_time;
	int throt_status;
	
	NoximCoord neighbor_coord = id2Coord(candidate_id);
	
	if (NoximGlobalParams::verbose_mode==-58){
		cout << nop_data;
		cout << "      On-Path channels: " << endl;
    }
	
    for (unsigned int i = 0; i < nop_channels.size(); i++) {
		int available;
		if (nop_data.channel_status_neighbor[nop_channels[i]].available)
			available = 1;
		else
			available = 0;
		
		switch(nop_channels[i])
		{
			case 0:
				throt_status = throttling[neighbor_coord.x][neighbor_coord.y-1][neighbor_coord.z];
				break;
			case 1:
				throt_status = throttling[neighbor_coord.x+1][neighbor_coord.y][neighbor_coord.z];
				break;
			case 2:
				throt_status = throttling[neighbor_coord.x][neighbor_coord.y+1][neighbor_coord.z];
				break;
			case 3:
				throt_status = throttling[neighbor_coord.x-1][neighbor_coord.y][neighbor_coord.z];
				break;
			case 4:
				throt_status = throttling[neighbor_coord.x][neighbor_coord.y][neighbor_coord.z-1];
				break;
			case 5:
				throt_status = throttling[neighbor_coord.x][neighbor_coord.y][neighbor_coord.z+1];
				break;
			default:
				throt_status = 0;
				break;
		}
		
		if(throt_status == 0) eff_service_time = 1; //non-throttled node
		else if(throt_status == 1) eff_service_time = 100000000; //fully throttled node
		else eff_service_time = 1/(1-(1/throt_status));
		
		int    free_slots = nop_data.channel_status_neighbor[nop_channels[i]].free_slots   ;
		double temp_diff  = nop_data.channel_status_neighbor[nop_channels[i]].temperature  ;
		int    counter    = nop_data.channel_status_neighbor[nop_channels[i]].channel_count;
		double not_throttle;
		
		if (NoximGlobalParams::verbose_mode==-58){
			cout << "       channel " << nop_channels[i] << " -> score: ";
			cout << " + " << available << " * (" << free_slots << ")" << endl;
		}
		if((nop_channels[i] == DIRECTION_DOWN) || (nop_channels[i] == DIRECTION_UP)){  //Throttled = 0, Not throttled = 1
			not_throttle = 1;}
		else{
			//not_throttle = !(nop_data.channel_status_neighbor[nop_channels[i]].throttle);
			int b;
			b= (int)monitor_in[i].read();
			if( b == 1 ) not_throttle = 0;
			else         not_throttle = 1;
		}
		double    temp_decision = temp_diff;
		if(temp_diff >= 2)
			temp_decision = 2;
		else if(temp_diff <= -2)
			temp_decision = -2;
		temp_decision = temp_decision + 2;
		double tune_ratio = 0.66;
		int max_buff = buffer[nop_channels[i]].GetMaxBufferSize();
		
		if(NoximGlobalParams::selection_info == BUF)
			eff_wait_time +=  available*(NoximGlobalParams::buffer_depth - free_slots);
		else
			eff_wait_time +=  available*((NoximGlobalParams::buffer_depth - free_slots) * eff_service_time);
		//score += (int) not_throttle*available*free_slots; //traffic-&throttling-aware
		//score += available * free_slots;
    }
	
	if(NoximGlobalParams::selection_info == BUF) return (eff_wait_time);
	else return (eff_wait_time/nop_channels.size());
    //return score;
}

/*** RCA Function (Jimmy added on 2012.06.27)***/
int NoximRouter::selectionRCA(const vector<int>& directions, const NoximRouteData& route_data)
{
	vector<int>  best_dirs;
	int best_RCA_value = 0;
	
	if(directions.size()==1) return directions[0];
	 
	for(unsigned int i = 0; i < directions.size(); ++i) {
	  	
		bool available = reservation_table.isAvailable(directions[i]);
		if(available) {
			int temp_RCA_value;
			if(directions[i]==DIRECTION_NORTH)
				if(id2Coord(route_data.dst_id).x > id2Coord(route_data.current_id).x)       //NE
					temp_RCA_value = RCA_data_in[DIRECTION_NORTH*2+1].read();
				else if(id2Coord(route_data.dst_id).x < id2Coord(route_data.current_id).x)  //NW
					temp_RCA_value = RCA_data_in[DIRECTION_NORTH*2+0].read();
			else                                                                        //N
			  assert(false);
			else if(directions[i]==DIRECTION_EAST)
				if(id2Coord(route_data.dst_id).y > id2Coord(route_data.current_id).y)       //SE
					temp_RCA_value = RCA_data_in[DIRECTION_EAST*2+1].read();
				else if(id2Coord(route_data.dst_id).y < id2Coord(route_data.current_id).y)  //NE
					temp_RCA_value = RCA_data_in[DIRECTION_EAST*2+0].read();
				else                                                                        //E
					assert(false);
			else if(directions[i]==DIRECTION_SOUTH)
				if(id2Coord(route_data.dst_id).x > id2Coord(route_data.current_id).x)       //SE
					temp_RCA_value = RCA_data_in[DIRECTION_SOUTH*2+0].read();
				else if(id2Coord(route_data.dst_id).x < id2Coord(route_data.current_id).x)  //SW
					temp_RCA_value = RCA_data_in[DIRECTION_SOUTH*2+1].read();
				else                                                                        //S
					assert(false);
			else if(directions[i]==DIRECTION_WEST)
				if(id2Coord(route_data.dst_id).y > id2Coord(route_data.current_id).y)       //SW
					temp_RCA_value = RCA_data_in[DIRECTION_WEST*2+0].read();
				else if(id2Coord(route_data.dst_id).y < id2Coord(route_data.current_id).y)  //NW
					temp_RCA_value = RCA_data_in[DIRECTION_WEST*2+1].read();
				else                                                                        //W
					assert(false);
			else
				assert(false);
 
			if(temp_RCA_value < best_RCA_value){ //waiting tine comparison
				best_RCA_value = temp_RCA_value;
				best_dirs.clear();
				best_dirs.push_back(directions[i]);
			}
			else if(temp_RCA_value == best_RCA_value) {
				best_dirs.push_back(directions[i]);
			}
		}
	  }
	  
	  if(best_dirs.size()) {
		return best_dirs[rand() % best_dirs.size()];
	  }
	  else {
		return directions[rand() % directions.size()];
	  }
	  
	  assert(false);
}

/****** Selection function for PTDBA based routing, Temp. info. -trans.-> traffic delay (Jimmy added on 2014.12.17) ******/
int NoximRouter::selectionThermal(const vector<int>& directions, const NoximRouteData& route_data)
{
	NoximCoord current = id2Coord(route_data.current_id);
	NoximCoord dst     = id2Coord(route_data.dst_id);
	if (NoximGlobalParams::TD_Type == TD_Type_Original)
	{	
		if(directions.size()==1) return directions[0]; //No choice as there is one candidate of routing direction		
		//There is 2 candidates of routing direction
		else if(directions.size()==2){
			float thermal_delay0 = 0;
			float thermal_delay1 = 0;
			float BCT0 = free_slots_neighbor[directions[0]].read(); //BCT = Buffer Constant Time
			float BCT1 = free_slots_neighbor[directions[1]].read();
			
			thermal_delay0 = BCT0; //Jimmy modified on 2014.11.27 //initialize the thermal_delay0. 
			thermal_delay1 = BCT1;
			
			// Because of restristed turn model, only 5 cases are needed to considered
			// No Up-Laternal turn and West-* turn (there is only one routing path can be selected, if go west)		
			if(directions[0]==DIRECTION_NORTH && directions[1]==DIRECTION_EAST){
				/** Select the North path (Jimmy modified on 2014.11.27)**/
				thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[0]].read()) +
								(1/2)*(1/5)*2*(buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				/** Select the East path (Jimmy modified on 2014.11.27)**/
				thermal_delay1 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[1]].read()) +
								(1/2)*(1/5)*2*(buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			else if(directions[0]==DIRECTION_NORTH && directions[1]==DIRECTION_DOWN){
				/** Select the North path (Jimmy modified on 2014.11.27)**/
				thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  + 
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()) +
								(1/2)*(1/5)*2*(buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				/** Select the Down path (Jimmy modified on 2014.11.27)**/
				thermal_delay1 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()) +
								(1/2)*(1/6)*2*(buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			else if(directions[0]==DIRECTION_SOUTH && directions[1]==DIRECTION_EAST){
				/** Select the South path (Jimmy modified on 2014.11.28)**/
				thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[0]].read()) +
								(1/2)*(1/5)*2*(buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				/** Select the East path (Jimmy modified on 2014.11.28)**/
				thermal_delay1 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[1]].read()) +
								(1/2)*(1/5)*2*(buf_neighbor[DIRECTION_NORTH      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			else if(directions[0]==DIRECTION_SOUTH && directions[1]==DIRECTION_DOWN){
				/** Select the South path (Jimmy modified on 2014.11.28)**/
				thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  + 
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()) +
								(1/2)*(1/5)*2*(buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				/** Select the Down path (Jimmy modified on 2014.11.28)**/
				thermal_delay1 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()) +
								(1/2)*(1/6)*2*(buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			else if(directions[0]==DIRECTION_EAST && directions[1]==DIRECTION_DOWN){
				/** Select the East path (Jimmy modified on 2014.11.28)**/
				thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  + 
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()) +
								(1/2)*(1/5)*2*(buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				/** Select the Down path (Jimmy modified on 2014.11.28)**/
				thermal_delay1 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()) +
								(1/2)*(1/6)*2*(buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			
			//cout<<"#### thermal_delay0:"<<thermal_delay0<<" #### BCT0:"<<BCT0<<" ####"<<endl;
			//cout<<"#### thermal_delay1:"<<thermal_delay1<<" #### BCT1:"<<BCT1<<" ####"<<endl;
			if(thermal_delay1 < thermal_delay0) return directions[1];
			else return directions[0];
		}
	
	
		else if(directions.size()==3){
		float thermal_delay0 = 0;
		float thermal_delay1 = 0;
		float thermal_delay2 = 0;

		float BCT0 = free_slots_neighbor[directions[0]].read();
		float BCT1 = free_slots_neighbor[directions[1]].read();
		float BCT2 = free_slots_neighbor[directions[2]].read();
		
		/* Jimmy modified on 2014.11.28*/
		//the current thermal delay only consider the free buffer depth. we don't consider the service time of the router
		thermal_delay0 = BCT0;
		thermal_delay1 = BCT1;
		thermal_delay2 = BCT2;
		
		if(directions[0]==DIRECTION_NORTH && directions[1]==DIRECTION_EAST && directions[2]==DIRECTION_DOWN){
			/** Select the North path (Jimmy modified on 2014.11.28)**/
			thermal_delay0 += (1/2)*(1/5)*2*(buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  + 
											buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
											buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()) +
							(1/2)*(1/5)*3*(buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
											buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
											buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			/** Select the East path (Jimmy modified on 2014.11.28)**/
			thermal_delay1 += (1/2)*(1/5)*2*(buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
											buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
											buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()) +
							(1/2)*(1/5)*3*(buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
											buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
											buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			/** Select the Down path (Jimmy modified on 2014.11.28)**/
			thermal_delay2 += (1/2)*(1/6)*2*(buf_neighbor[DIRECTION_NORTH     ][directions[2]].read()  + 
											buf_neighbor[DIRECTION_EAST      ][directions[2]].read()  +
											buf_neighbor[DIRECTION_DOWN      ][directions[2]].read()) +
							(1/2)*(1/6)*3*(buf_neighbor[DIRECTION_SOUTH     ][directions[2]].read()  +
											buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +
											buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
											buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;
		}
		else if(directions[0]==DIRECTION_SOUTH && directions[1]==DIRECTION_EAST && directions[2]==DIRECTION_DOWN){
			/** Select the South path (Jimmy modified on 2014.11.28)**/
			thermal_delay0 += (1/2)*(1/5)*2*(buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  + 
											buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
											buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()) +
							(1/2)*(1/5)*3*(buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
											buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
											buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			/** Select the East path (Jimmy modified on 2014.11.28)**/
			thermal_delay1 += (1/2)*(1/5)*2*(buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  + 
											buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
											buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()) +
							(1/2)*(1/5)*3*(buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  +
											buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
											buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			/** Select the Down path (Jimmy modified on 2014.11.28)**/
			thermal_delay2 += (1/2)*(1/6)*2*(buf_neighbor[DIRECTION_SOUTH     ][directions[2]].read()  + 
											buf_neighbor[DIRECTION_EAST      ][directions[2]].read()  +
											buf_neighbor[DIRECTION_DOWN      ][directions[2]].read()) +
							(1/2)*(1/6)*3*(buf_neighbor[DIRECTION_NORTH     ][directions[2]].read()  +
											buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +
											buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
											buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;
		}
		//cout<<"#### thermal_delay0:"<<thermal_delay0<<" #### BCT0:"<<BCT0<<" ####"<<endl;
		//cout<<"#### thermal_delay1:"<<thermal_delay1<<" #### BCT1:"<<BCT1<<" ####"<<endl;
		//cout<<"#### thermal_delay2:"<<thermal_delay2<<" #### BCT2:"<<BCT2<<" ####"<<endl;
		
			if(thermal_delay2 < thermal_delay1 && thermal_delay2 < thermal_delay0) return directions[2];
			else if(thermal_delay1 < thermal_delay0 && thermal_delay1 < thermal_delay2) return directions[1];
			else return directions[0];
		}
	
		else return directions[rand() % directions.size()]; //Random select the candidates of routing dirction, if identical thermal delay 

	}
	else if (NoximGlobalParams::TD_Type == TD_Type_GTDAR)
	{
		//if(local_id == 29) cout<<"#### "<<getCurrentCycleNum()<<": Enter GTDAR ####"<<endl; //////////////////////////////
		if(directions.size()==1) return directions[0]; //No choice as there is one candidate of routing direction		
		//There is 2 candidates of routing direction
		else if(directions.size()==2){
			//if(local_id == 29) cout<<"#### Enter GTDAR - direction = 2 ####"<<endl; //////////////////////////////
			float thermal_delay0 = 0;
			float thermal_delay1 = 0;
			float thermal_delay2 = 0;
			float thermal_delay3 = 0;
			float BCT0 = free_slots_neighbor[directions[0]].read(); //BCT = Buffer Constant Time
			float BCT1 = free_slots_neighbor[directions[0]].read(); 
			float BCT2 = free_slots_neighbor[directions[1]].read();
			float BCT3 = free_slots_neighbor[directions[1]].read();
			
			
			thermal_delay0 = BCT0; //Jimmy modified on 2014.11.27 //initialize the thermal_delay0. 
			thermal_delay1 = BCT1;
			thermal_delay2 = BCT2;
			thermal_delay3 = BCT3;
			
			//Because of restristed turn model, only 5 cases are needed to considered
			//No Up-Laternal turn and West-* turn (there is only one routing path can be selected, if go west)		
			if(directions[0]==DIRECTION_NORTH && directions[1]==DIRECTION_EAST){
			/** Select the NORTH path (Dora modified on 2017.4.14)**/
			thermal_delay0 += (1/2)*(1/5)*	(buf_neighbor[DIRECTION_EAST     ][directions[0]].read()   +
										 buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			thermal_delay1 += (1/2)*(1/5)*	(buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			/** Select the EAST path (Dora modified on 2017.4.14)**/
			thermal_delay2 += (1/2)*(1/5)* 	(buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			thermal_delay3 += (1/2)*(1/5)* 	(buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			else if(directions[0]==DIRECTION_NORTH && directions[1]==DIRECTION_DOWN){
			/**Select the NORTH path (Dora modified on 2017.4.14)**/
			thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
						                 buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			thermal_delay1 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  + 			                                 
										 buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
						                 buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			/**Select the DOWN path (Dora modified on 2017.4.14)**/
			thermal_delay2 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read());
			thermal_delay3 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  + 			                                 
										 buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read());			
			}
			else if(directions[0]==DIRECTION_SOUTH && directions[1]==DIRECTION_EAST){
			/**Select the SOUTH path (Dora modified on 2017.4.14)**/
			thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			thermal_delay1 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  + 
			                                buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +											 
						                 buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			/**Select the EAST path (Dora modified on 2017.4.14)**/
			thermal_delay2 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			thermal_delay3 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 			                                 
										 buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read());								 
			}
			else if(directions[0]==DIRECTION_SOUTH && directions[1]==DIRECTION_DOWN){
			/**Select the SOUTH path (Dora modified on 2017.4.14)**/
			thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
						                 buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			thermal_delay1 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
						                 buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;								 
			/**Select the DOWN path (Dora modified on 2017.4.14)**/
			thermal_delay2 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			thermal_delay3 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +
						                 buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
										 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			else if(directions[0]==DIRECTION_EAST && directions[1]==DIRECTION_DOWN){
			/** Select the East path (Dora modified on 2017.4.14)**/
			thermal_delay0 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  +
							                 buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  +
											 buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
											 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
											 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			thermal_delay1 += (1/2)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[0]].read()  +
							                 buf_neighbor[DIRECTION_WEST   	  ][directions[0]].read()  +
											 buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  +
											 buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
											 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
			/** Select the DOWN path (Dora modified on 2017.4.14)**/
			thermal_delay2 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
			                                 buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
							                 buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
											 buf_neighbor[DIRECTION_WEST      ][directions[1]].read()  +							                 
											 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
											 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			thermal_delay3 += (1/2)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
			                                 buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
							                 buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
											 buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +							                 
											 buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
											 buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
			}
			float p = (thermal_delay3-thermal_delay2)/(thermal_delay0-thermal_delay2-thermal_delay1+thermal_delay3);
			//float p = (thermal_delay2-thermal_delay3)/(thermal_delay2+thermal_delay1-thermal_delay3-thermal_delay0);
		
			float rf;  //�H�����v
			//unsigned seed;
			//seed = (unsigned)time(NULL); // ���o�ɶ��ǦC
			//srand(seed); // �H�ɶ��ǦC���üƺؤl		
			//rf = rand()/(RAND_MAX+1.0);	//����[0~1) ��?���B�I�ƶü�
			rf = ((double)rand())/((double)RAND_MAX);
			/*cout<<"local id="<<local_id<<": p="<<p<<" ---- td0="<<thermal_delay0<<
			                                        "      td1="<<thermal_delay1<<
													"      td2="<<thermal_delay2<<
													"      td3="<<thermal_delay3<<endl; /////////////
			cout<<buf_neighbor[DIRECTION_NORTH     ][directions[0]].read<<"  "<<
			      buf_neighbor[DIRECTION_EAST     ][directions[0]].read<<"  "<<
				  buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read<<"  "<<
				  buf_neighbor[DIRECTION_WEST     ][directions[0]].read<<"  "<<
				  buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read<<"  "<<
				  buf_neighbor[DIRECTION_SEMI_LOCAL     ][directions[0]].read<<endl;
			cout<<buf_neighbor[DIRECTION_NORTH     ][directions[1]].read<<"  "<<
			      buf_neighbor[DIRECTION_EAST     ][directions[1]].read<<"  "<<
				  buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read<<"  "<<
				  buf_neighbor[DIRECTION_WEST     ][directions[1]].read<<"  "<<
				  buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read<<"  "<<
				  buf_neighbor[DIRECTION_SEMI_LOCAL     ][directions[1]].read<<endl<<endl;*/
			if(rf <= p){
				//if(local_id == 29) cout<<"#### "<<getCurrentCycleNum()<<": Out GTDAR - dirction = 2-1 ####"<<endl; //////////////////////////////
				return directions[0];
			}
			else{
				//if(local_id == 29) cout<<"#### "<<getCurrentCycleNum()<<": Out GTDAR - dirction = 2-2 ####"<<endl; //////////////////////////////
				return directions[1];
			}
		}
		
		else if(directions.size()==3){
			cout<<directions.size() << "test"<<endl;
			//if(local_id == 29) cout<<"#### "<<getCurrentCycleNum()<<": Enter GTDAR - direction = 3 ####"<<endl; //////////////////////////////
			float thermal_delay0 = 0;
			float thermal_delay1 = 0;
			float thermal_delay2 = 0;
			float thermal_delay3 = 0;
			float thermal_delay4 = 0;
			float thermal_delay5 = 0;
			float thermal_delay6 = 0;
			float thermal_delay7 = 0;
			float thermal_delay8 = 0;
			
			float BCT0 = free_slots_neighbor[directions[0]].read();
			float BCT1 = free_slots_neighbor[directions[0]].read();
			float BCT2 = free_slots_neighbor[directions[0]].read();
			float BCT3 = free_slots_neighbor[directions[1]].read();
			float BCT4 = free_slots_neighbor[directions[1]].read();
			float BCT5 = free_slots_neighbor[directions[1]].read();
			float BCT6 = free_slots_neighbor[directions[2]].read();
			float BCT7 = free_slots_neighbor[directions[2]].read();
			float BCT8 = free_slots_neighbor[directions[2]].read();
			/* Jimmy modified on 2014.11.28*/
			//the current thermal delay only consider the free buffer depth. we don't consider the service time of the router
			thermal_delay0 = BCT0;
			thermal_delay1 = BCT1;
			thermal_delay2 = BCT2;
			thermal_delay3 = BCT3;
			thermal_delay4 = BCT4;
			thermal_delay5 = BCT5;
			thermal_delay6 = BCT6;
			thermal_delay7 = BCT7;
			thermal_delay8 = BCT8;
			
			if(directions[0]==DIRECTION_NORTH && directions[1]==DIRECTION_EAST && directions[2]==DIRECTION_DOWN){
				/** Select the North path (Dora modified on 2017.4.14)**/
				thermal_delay0 += (1/3)*(1/5)*   (buf_neighbor[DIRECTION_EAST     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				thermal_delay1 += (1/3)*(1/5)*   (buf_neighbor[DIRECTION_NORTH    ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				thermal_delay2 += (1/3)*(1/5)*   (buf_neighbor[DIRECTION_NORTH    ][directions[0]].read()  +
												buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				/** Select the EAST path (Dora modified on 2017.4.14)**/
				thermal_delay3 += (1/3)*(1/5)*   (buf_neighbor[DIRECTION_NORTH    ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
				thermal_delay4 += (1/3)*(1/5)*   (buf_neighbor[DIRECTION_EAST     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
				thermal_delay5 += (1/3)*(1/5)*   (buf_neighbor[DIRECTION_NORTH    ][directions[1]].read()  +
												buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
				/** Select the DOWN path (Dora modified on 2017.4.14)**/
				thermal_delay6 += (1/3)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[2]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +										
												buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;
				
				thermal_delay7 += (1/3)*(1/6)*  (buf_neighbor[DIRECTION_EAST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[2]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;
				
				thermal_delay8 += (1/3)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[2]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;											
			}
			else if(directions[0]==DIRECTION_SOUTH && directions[1]==DIRECTION_EAST && directions[2]==DIRECTION_DOWN){
				/** Select the SOUTH path (Dora modified on 2017.4.14)**/
				thermal_delay0 += (1/3)*(1/5)*  (buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
												
				thermal_delay1 += (1/3)*(1/5)*  (buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
												
				thermal_delay2 += (1/3)*(1/5)*  (buf_neighbor[DIRECTION_EAST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[0]].read()  +
												buf_neighbor[DIRECTION_LOCAL     ][directions[0]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[0]].read()) ;
				/** Select the EAST path (Dora modified on 2017.4.14)**/
				thermal_delay3 += (1/3)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
											
				thermal_delay4 += (1/3)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[1]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
												
				thermal_delay5 += (1/3)*(1/5)*  (buf_neighbor[DIRECTION_NORTH     ][directions[1]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[1]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[1]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[1]].read()) ;
				/** Select the DOWN path (Dora modified on 2017.4.14)**/
				thermal_delay6 += (1/3)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[2]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SOUTH     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;
												
				thermal_delay7 += (1/3)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[2]].read()  + 
												buf_neighbor[DIRECTION_EAST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[2]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;
				
				thermal_delay8 += (1/3)*(1/6)*  (buf_neighbor[DIRECTION_NORTH     ][directions[2]].read()  + 
												buf_neighbor[DIRECTION_SOUTH     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_WEST      ][directions[2]].read()  +
												buf_neighbor[DIRECTION_DOWN      ][directions[2]].read()  +											 
												buf_neighbor[DIRECTION_LOCAL     ][directions[2]].read()  +
												buf_neighbor[DIRECTION_SEMI_LOCAL][directions[2]].read()) ;
			}
			
			float p1=(((thermal_delay7-thermal_delay6)*(thermal_delay4-thermal_delay5-thermal_delay7+thermal_delay8)-(thermal_delay8-thermal_delay7)*(thermal_delay3-thermal_delay4-thermal_delay6+thermal_delay7))/((thermal_delay0-thermal_delay1-thermal_delay6+thermal_delay7)*(thermal_delay4-thermal_delay5-thermal_delay7+thermal_delay8)-(thermal_delay1-thermal_delay2-thermal_delay7+thermal_delay8)*(thermal_delay3-thermal_delay4-thermal_delay6+thermal_delay7)));							
			float p2=(((thermal_delay7-thermal_delay6)*(thermal_delay1-thermal_delay2-thermal_delay7+thermal_delay8)-(thermal_delay8-thermal_delay7)*(thermal_delay0-thermal_delay1-thermal_delay6+thermal_delay7))/((thermal_delay3-thermal_delay4-thermal_delay6+thermal_delay7)*(thermal_delay1-thermal_delay2-thermal_delay7+thermal_delay8)-(thermal_delay4-thermal_delay5-thermal_delay7+thermal_delay8)*(thermal_delay0-thermal_delay1-thermal_delay6+thermal_delay7)));
			//float p1=(((thermal_delay6-thermal_delay7)*(thermal_delay5+thermal_delay7-thermal_delay8-thermal_delay4)-(thermal_delay7-thermal_delay8)*(thermal_delay4+thermal_delay6-thermal_delay7-thermal_delay3))/((thermal_delay1+thermal_delay6-thermal_delay7-thermal_delay0)*(thermal_delay5+thermal_delay7-thermal_delay8-thermal_delay4)-(thermal_delay2+thermal_delay7-thermal_delay8-thermal_delay1)*(thermal_delay4+thermal_delay6-thermal_delay7-thermal_delay3)));							
			//float p2=(((thermal_delay6-thermal_delay7)*(thermal_delay2+thermal_delay7-thermal_delay8-thermal_delay1)-(thermal_delay7-thermal_delay8)*(thermal_delay1+thermal_delay6-thermal_delay7-thermal_delay0))/((thermal_delay4+thermal_delay6-thermal_delay7-thermal_delay3)*(thermal_delay2+thermal_delay7-thermal_delay8-thermal_delay1)-(thermal_delay5+thermal_delay7-thermal_delay8-thermal_delay4)*(thermal_delay1+thermal_delay6-thermal_delay7-thermal_delay0)));
			
			/**/
			
			/**/
			
			/*
			float p3=(1-p1-p2);
			float rf;  //�H�����v		
			//rf = rand()/ (RAND_MAX+1.0);	//����[0~1) ��?���B�I�ƶü�
			rf = ((double)rand())/((double)RAND_MAX);
			//cout<<"local id="<<local_id<<": p1="<<p1<<" p2="<<p2<<endl; /////////////////////
			if(rf <= p1){
				//if(local_id == 29) cout<<"#### "<<getCurrentCycleNum()<<": Out GTDAR - direction = 3 ####"<<endl; //////////////////////////////
				return directions[0];
			}
			else if( (rf > p1) && (rf <= p3) ){
				//if(local_id == 29) cout<<"#### "<<getCurrentCycleNum()<<": Out GTDAR - direction = 3 ####"<<endl; //////////////////////////////
				return directions[1];
			}
			else{
				//if(local_id == 29) cout<<"#### "<<getCurrentCycleNum()<<": Out GTDAR - direction = 3 ####"<<endl; //////////////////////////////
				return directions[2];
			}
			*/
		} 	
		else return directions[rand() % directions.size()]; //Random select the candidates of routing dirction, if identical thermal delay
			
	}
}	
		
	
	

int NoximRouter::selectionNoP(const vector < int >&directions,
							  const NoximRouteData & route_data)
{
    vector < int >neighbors_on_path;
    vector < int >score;
    int direction_selected = NOT_VALID;
    int current_id         = route_data.current_id;
	int min_wait_time = 10000000;
	
    for ( int i = 0; i < directions.size(); i++) {
		// get id of adjacent candidate
		int candidate_id = getNeighborId(current_id, directions[i]);
	
		// apply routing function to the adjacent candidate node
		NoximRouteData tmp_route_data;
		tmp_route_data.current_id = candidate_id;
		tmp_route_data.src_id     = route_data.src_id;
		tmp_route_data.dst_id     = route_data.dst_id;
		tmp_route_data.dir_in     = reflexDirection(directions[i]);
		tmp_route_data.DW_layer   = route_data.DW_layer;
		tmp_route_data.routing    = route_data.routing;
	
		vector < int > next_candidate_channels = routingFunction(tmp_route_data);
	
		// select useful data from Neighbor-on-Path input 
		NoximNoP_data nop_tmp = NoP_data_in[directions[i]].read();
	
		// store the score of node in the direction[i]
		score.push_back(NoPScore(nop_tmp, next_candidate_channels, candidate_id)); //Jimmy added candidate_id on 2012.06.27
    }

    // check for direction with minimum effective waiting time (Jimmy modify max -> min)
    int  min_direction     = directions[0];
    int  min               = score     [0];
	int  down_score        = 0            ;
	
	for ( int i = 0; i < directions.size(); i++) { //Jimmy modify (> -> < ; max -> min)
		if (score[i] < min) {
			min_direction = directions[i];
			min           = score     [i];
		}
    }
    // if multiple direction have the same score = max, choose randomly.

    vector < int >equivalent_directions;

    for (unsigned int i = 0; i < directions.size(); i++)
	if (score[i] == min) //Jimmy modify max -> min
	    equivalent_directions.push_back(directions[i]);

    direction_selected =
	equivalent_directions[rand() % equivalent_directions.size()];

    return direction_selected;
}

int NoximRouter::selectionBufferLevel(const vector < int >&directions)
{
    vector < int >best_dirs;
    int max_free_slots  = 0;
	int eff_service_time; //effective service time (Jimmy added on 2012.06.26)
	int throt_status; //the throttling status of the neighboring node (Jimmy added on 2012.06.26)
	int eff_wait_time; //Jimmy added on 2012.06.26
	int min_wait_time = 10000000; //Jimmy added on 2012.06.26
	NoximCoord current = id2Coord(local_id); //Jimmy added on 2012.06.26
	
    for (unsigned int i = 0; i < directions.size(); i++) {
		int free_slots = free_slots_neighbor[directions[i]].read();
		bool available = reservation_table.isAvailable(directions[i]);
		
		/** Caculate the effective service time (Jimmy added on 2012.06.26) **/
		switch(directions[i])
		{
			case 0:
				throt_status = throttling[current.x][current.y-1][current.z];
				break;
			case 1:
				throt_status = throttling[current.x+1][current.y][current.z];
				break;
			case 2:
				throt_status = throttling[current.x][current.y+1][current.z];
				break;
			case 3:
				throt_status = throttling[current.x-1][current.y][current.z];
				break;
			case 4:
				throt_status = throttling[current.x][current.y][current.z-1];
				break;
			case 5:
				throt_status = throttling[current.x][current.y-1][current.z+1];
				break;
			default:
				throt_status = 0;
				break;
		}

		if(throt_status == 0) eff_service_time = 1; //the effective service time is equal to 1, if this node is a non-throttled node
		else if(throt_status == 1) eff_service_time = 10000000; //if this node is throttled, the effective service time become very large
		else eff_service_time = 1/(1-(1/throt_status)); //the effective service time of the partial throttled (i.e., hafely throttle) node is equal to 2
		
		if(NoximGlobalParams::selection_info == BUF)
			eff_wait_time = NoximGlobalParams::buffer_depth - free_slots;
		else
			eff_wait_time = (NoximGlobalParams::buffer_depth - free_slots) * eff_service_time;
		/****/
		
		if (available) {
			if (eff_wait_time  < min_wait_time) {
				min_wait_time = eff_wait_time;
				best_dirs.clear();
				best_dirs.push_back(directions[i]);
			} 
			else if (eff_wait_time == min_wait_time)
				best_dirs.push_back(directions[i]);
		}
		
		/*if (available) {
			if (free_slots > max_free_slots) {
			max_free_slots = free_slots;
			best_dirs.clear();
			best_dirs.push_back(directions[i]);
			} else if (free_slots == max_free_slots)
			best_dirs.push_back(directions[i]);
		}*/
    }
    if (best_dirs.size())
		return (best_dirs[rand() % best_dirs.size()]);
    else
		return (directions[rand() % directions.size()]);
}

int NoximRouter::selectionProposed(const vector <int> & directions, const NoximRouteData & route_data)
{
	vector < int >neighbors_on_path;
    vector < int >score;
    int direction_selected = NOT_VALID;
    int current_id         = route_data.current_id;
	
    for ( int i = 0; i < directions.size(); i++) {
	// get id of adjacent candidate
	int candidate_id = getNeighborId(current_id, directions[i]);

	// apply routing function to the adjacent candidate node
	NoximRouteData tmp_route_data;
	tmp_route_data.current_id = candidate_id;
	tmp_route_data.src_id     = route_data.src_id;
	tmp_route_data.dst_id     = route_data.dst_id;
	tmp_route_data.dir_in     = reflexDirection(directions[i]);
	tmp_route_data.DW_layer   = route_data.DW_layer;
	tmp_route_data.routing    = route_data.routing;

	vector < int > next_candidate_channels = routingFunction(tmp_route_data);

	// select useful data from Neighbor-on-Path input 
	NoximNoP_data nop_tmp = NoP_data_in[directions[i]].read();

	// store the score of node in the direction[i]
	if( current_id < 128 && i < 4 )
		score.push_back(NoPScore(nop_tmp, next_candidate_channels, candidate_id)*2); //Jimmy added candidate_id on 2012.06.27
	else 
		score.push_back(NoPScore(nop_tmp, next_candidate_channels, candidate_id)); //Jimmy added candidate_id on 2012.06.27
    }

    // check for direction with higher score
    int  max_direction     = directions[0];
    int  max               = score     [0];
	int  down_score        = 0            ;
	
	for ( int i = 0; i < directions.size(); i++) {
		if (score[i] > max) {
			max_direction = directions[i];
			max           = score     [i];
		}
    }
    // if multiple direction have the same score = max, choose randomly.

    vector < int >equivalent_directions;

    for (unsigned int i = 0; i < directions.size(); i++)
	if (score[i] == max)
	    equivalent_directions.push_back(directions[i]);

    direction_selected =
	equivalent_directions[rand() % equivalent_directions.size()];

    return direction_selected;
}

int NoximRouter::selectionRandom(const vector < int >&directions)
{
    return directions[rand() % directions.size()];
}
vector<int> NoximRouter::DW_layerSelFunction    (const int select_routing, const NoximCoord& current, const NoximCoord& destination, const NoximCoord& source, int dw_layer){
	switch ( NoximGlobalParams::dw_layer_sel ){
	case DW_BL      :return routingTLAR_DW         (current, source, destination          );break;
	case DW_ODWL    :return routingTLAR_DW_ODWL    (current, source, destination, dw_layer);break;
	case DW_ADWL    :return routingTLAR_DW_ADWL    (current, source, destination          );break;
	case DW_IPD     :return routingTLAR_DW_IPD     (current, source, destination          );break;
	case DW_VBDR    :return routingTLAR_DW_VBDR    (current, source, destination          );break;
	case DW_ODWL_IPD:return routingTLAR_DW_ODWL_IPD(current, source, destination, dw_layer);break;
	default         :return routingTLAR_DW         (current, source, destination          );break;
	}
}
int NoximRouter::selectionFunction(const vector < int >&directions,
				   const NoximRouteData & route_data)
{
	
    // not so elegant but fast escape ;)
    //if (directions.size() == 1)
	//return directions[0];
    //cout<<"@@@@@@@@@@@"<<endl;
	//cout <<directions.size()<<endl;
    switch (NoximGlobalParams::selection_strategy) {
    case SEL_RANDOM      :return selectionRandom     (directions);
    case SEL_BUFFER_LEVEL:return selectionBufferLevel(directions);
    case SEL_NOP         :return selectionNoP        (directions, route_data);
	case SEL_RCA         :return selectionRCA(directions,route_data); //2D RCA (Jimmy added on 2012.06.27)
	case SEL_PROPOSED    :return selectionProposed   (directions, route_data);
	case SEL_THERMAL     :return selectionThermal    (directions, route_data); //Jimmy added on 2014.12.17
	default              :assert(false);
    }
    return 0;
}

vector < int >NoximRouter::routingXYZ(const NoximCoord & current,const NoximCoord & destination){
    vector < int >directions;
	     if (destination.x > current.x)	directions.push_back(DIRECTION_EAST );
    else if (destination.x < current.x)	directions.push_back(DIRECTION_WEST );
    else if (destination.y > current.y)	directions.push_back(DIRECTION_SOUTH);
    else if (destination.y < current.y)	directions.push_back(DIRECTION_NORTH);
	else if (destination.z > current.z)	directions.push_back(DIRECTION_DOWN );
    else if (destination.z < current.z)	directions.push_back(DIRECTION_UP   );
    return directions;
}
vector<int> NoximRouter::routingZXY(const NoximCoord& current, const NoximCoord& destination){
  vector<int> directions;
       if (destination.z > current.z) directions.push_back(DIRECTION_DOWN );
  else if (destination.z < current.z) directions.push_back(DIRECTION_UP   );
  else if (destination.x > current.x) directions.push_back(DIRECTION_EAST );
  else if (destination.x < current.x) directions.push_back(DIRECTION_WEST );
  else if (destination.y > current.y) directions.push_back(DIRECTION_SOUTH);
  else if (destination.y < current.y) directions.push_back(DIRECTION_NORTH);
  return directions;
}

vector<int> NoximRouter::routingDownward(const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination){

	int down_level = NoximGlobalParams::down_level;
	vector<int> directions;
	int layer;	//means which layer that packet should be transmitted

	if( (source.z + down_level)> NoximGlobalParams::mesh_dim_z-1)
		layer = NoximGlobalParams::mesh_dim_z-1;
	else 
		layer = source.z + down_level;

	if(current.z < layer && (current.x != destination.x || current.y != destination.y) ){
			if(current.z == destination.z && ( (current.x-destination.x== 1 && current.y==destination.y)	//while the dest. is one hop to source, don't downward and transmit directly
																			 ||(current.x-destination.x==-1 && current.y==destination.y) 
																			 ||(current.y-destination.y== 1 && current.x==destination.x) 
																			 ||(current.y-destination.y==-1 && current.x==destination.x) ))
			{
				directions = routingXYZ(current, destination);	 
			}
			else {
				directions.push_back(DIRECTION_DOWN);
			}
		}
		else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) )	//forward by xyz routing
		{
			 directions = routingXYZ(current, destination);	 
		}
		else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z)	//same (x,y), forward to up
		{	
			directions.push_back(DIRECTION_UP);
		}	
		else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z)  //same (x,y), forward to down
		{	
			directions.push_back(DIRECTION_DOWN);
		}	
		else 
		{ 
			cout<<"ERROR! Out of condition!?!?"<<endl;
			cout<<"current: ("<<current.x<<","<<current.y<<","<<current.z<<")"<<endl;
			cout<<"destination: ("<<destination.x<<","<<destination.y<<","<<destination.z<<")"<<endl;
			cout<<"layer: "<<layer<<endl;
			exit(1);			
		}

  return directions;
}

vector < int >NoximRouter::routingWestFirst(const NoximCoord & current,
					    const NoximCoord & destination)
{
    vector < int >directions;

    if (destination.x <= current.x || destination.y == current.y)
	return routingXYZ(current, destination);

    if (destination.y < current.y) {
	directions.push_back(DIRECTION_NORTH);
	directions.push_back(DIRECTION_EAST);
    } else {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_EAST);
    }

    return directions;
}

vector < int >NoximRouter::routingNorthLast(const NoximCoord & current,
					    const NoximCoord & destination)
{
    vector < int >directions;

    if (destination.x == current.x || destination.y <= current.y)
	return routingXYZ(current, destination);

    if (destination.x < current.x) {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_WEST);
    } else {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_EAST);
    }

    return directions;
}

vector < int >NoximRouter::routingNegativeFirst(const NoximCoord & current,
						const NoximCoord &
						destination)
{
    vector < int >directions;

    if ((destination.x <= current.x && destination.y <= current.y) ||
	(destination.x >= current.x && destination.y >= current.y))
	return routingXYZ(current, destination);

    if (destination.x > current.x && destination.y < current.y) {
	directions.push_back(DIRECTION_NORTH);
	directions.push_back(DIRECTION_EAST);
    } else {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_WEST);
    }

    return directions;
}

vector < int >NoximRouter::routingOddEven(const NoximCoord & current,
					  const NoximCoord & source,
					  const NoximCoord & destination)
{
    vector < int >directions;

    int c0 = current.x;
    int c1 = current.y;
    int s0 = source.x;
    //  int s1 = source.y;
    int d0 = destination.x;
    int d1 = destination.y;
    int e0, e1;

    e0 = d0 - c0;
    e1 = -(d1 - c1);

    if (e0 == 0) {
	if (e1 > 0)
	    directions.push_back(DIRECTION_NORTH);
	else
	    directions.push_back(DIRECTION_SOUTH);
    } else {
	if (e0 > 0) {
	    if (e1 == 0)
		directions.push_back(DIRECTION_EAST);
	    else {
		if ((c0 % 2 == 1) || (c0 == s0)) {
		    if (e1 > 0)
			directions.push_back(DIRECTION_NORTH);
		    else
			directions.push_back(DIRECTION_SOUTH);
		}
		if ((d0 % 2 == 1) || (e0 != 1))
		    directions.push_back(DIRECTION_EAST);
	    }
	} else {
	    directions.push_back(DIRECTION_WEST);
	    if (c0 % 2 == 0) {
		if (e1 > 0)
		    directions.push_back(DIRECTION_NORTH);
		if (e1 < 0)
		    directions.push_back(DIRECTION_SOUTH);
	    }
	}
    }

    if (!(directions.size() > 0 && directions.size() <= 2)) {
	cout << "\n PICCININI, CECCONI & ... :";	// STAMPACCHIA
	cout << source << endl;
	cout << destination << endl;
	cout << current << endl;
    }
    assert(directions.size() > 0 && directions.size() <= 2);
    return directions;
}
/*=============================Odd Even-3D==========================*/
vector<int> NoximRouter::routingOddEven_for_3D(const NoximCoord& current, 
				    const NoximCoord& source, const NoximCoord& destination)
{
	vector<int> directions;
	
	int c0 = current.y;
	int c1 = current.z;
	int s0 = source.y;
	//  int s1 = source.y;
	int d0 = destination.y;
	int d1 = destination.z;
	int e0, e1;
	
	e0 = d0 - c0;
	e1 = d1 - c1;

	if (e0 == 0){
		if (e1 > 0)
			directions.push_back(DIRECTION_DOWN);
		else if (e1 < 0)
			directions.push_back(DIRECTION_UP);
    }
	else{
		if (e0 > 0){
			if (e1 == 0)
				directions.push_back(DIRECTION_SOUTH);
			else{
				if ( (c0 % 2 == 1) || (c0 == s0) ){ //Odd
					if (e1 > 0)
						directions.push_back(DIRECTION_DOWN);
					else if (e1 < 0)
						directions.push_back(DIRECTION_UP);
				}
				if ( (d0 % 2 == 1) || (e0 != 1) )
					directions.push_back(DIRECTION_SOUTH);
			}
		}
		else{
			directions.push_back(DIRECTION_NORTH);
			if (c0 % 2 == 0){
				if (e1 > 0)
					directions.push_back(DIRECTION_DOWN);
				if (e1 < 0) 
					directions.push_back(DIRECTION_UP);
			}
		}
    }
	return directions;
}
vector<int> NoximRouter::routingOddEven_3D (const NoximCoord& current, 
				    const NoximCoord& source, const NoximCoord& destination)
{
  vector<int> directions;

  int c0 = current.x;
  int c1 = current.y;
  int c2 = current.z;
  int s0 = source.x;
  int s1 = source.y;
  int s2 = source.z;
  int d0 = destination.x;
  int d1 = destination.y;
  int d2 = destination.z; //z = 0, which is far from heat sink
  int e0, e1, e2;

  e0 = d0 - c0;
  e1 = -(d1 - c1); //positive: North, negative: South
  e2 = d2 - c2; //positive: Down, negative: Up

  if (e0 == 0){
	directions = routingOddEven_for_3D(current, source, destination);
  }
  else{
	if (e0 < 0){ //x-
		if ( (c0 % 2 == 0) ){
			directions = routingOddEven_for_3D(current, source, destination);
		}
		directions.push_back(DIRECTION_WEST);
	}
	else{ //e0 > 0, x+
		if( (e1 == 0) && (e2 == 0) )
			directions.push_back(DIRECTION_EAST);	
		else{
			if ( (d0 % 2 == 1) || (e0 != 1) ){
				directions.push_back(DIRECTION_EAST);
			}
			if ( (c0 % 2 == 1) || (c0 == s0) ){
				directions = routingOddEven_for_3D(current, source, destination);
			}
		}
	}
  }
  if (!(directions.size() > 0 && directions.size() <= 3)){
      cout << "\n STAMPACCHIO :";
      cout << source << endl;
      cout << destination << endl;
      cout << current << endl;
  }
  assert(directions.size() > 0 && directions.size() <= 3);
  
  return directions;
}
//=============================Odd Even + Downward==========================
//�ª��A�Y���쪺Downward tag

vector<int> NoximRouter::routingOddEven_Downward (const NoximCoord& current, 
				    const NoximCoord& source, const NoximCoord& destination, const NoximRouteData& route_data)
{
	vector<int> directions;
	int layer;	//���ܦ�packet���Ӧb����layer��XY�ǻ�
	/*int down_level = NoximGlobalParams::down_level;
	if( (source.z + down_level)> NoximGlobalParams::mesh_dim_z-1)
		layer = NoximGlobalParams::mesh_dim_z-1;
	else 
		layer = source.z + down_level;*/
	layer = NoximGlobalParams::mesh_dim_z-1;
	if(current.z < layer && (current.x != destination.x || current.y != destination.y) )  
	{

		if(current.z == destination.z && ( (current.x-destination.x== 1 && current.y==destination.y)	//X or Y��V�W�u�۶Z�@��ɴN�����L�h,��DW
										||(current.x-destination.x==-1 && current.y==destination.y) 
										||(current.y-destination.y== 1 && current.x==destination.x) 
										||(current.y-destination.y==-1 && current.x==destination.x) ))
		{
			directions = routingOddEven(current, source, destination);	 
		}
		else 
		{
					directions.push_back(DIRECTION_DOWN);
		}
	}
	else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) )	//�b�����Hxy routing��
	{
			directions = routingOddEven(current, source, destination);
	}
	else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z)	//xy�ۦP, z��V���W��
	{	
		directions.push_back(DIRECTION_UP);
	}	
	else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z)  //xy�ۦP, z��V���U��
	{	
		directions.push_back(DIRECTION_DOWN);
	}	
	else 
	{ 
		cout<<"ERROR! Out of condition!?!?"<<endl;
		cout<<"current: ("<<current.x<<","<<current.y<<","<<current.z<<")"<<endl;
		cout<<"destination: ("<<destination.x<<","<<destination.y<<","<<destination.z<<")"<<endl;
		cout<<"layer: "<<layer<<endl;
		exit(1);			
	}
  assert(directions.size() > 0 && directions.size() <= 3);
  return directions;
}
vector<int> NoximRouter::routingOddEven_Z (const NoximCoord& current, 
				    const NoximCoord& source, const NoximCoord& destination)
{
	vector<int> directions;

	if(current.x == destination.x && current.y == destination.y){
		if(current.z < destination.z)
			directions.push_back(DIRECTION_DOWN);
		else
			directions.push_back(DIRECTION_UP);
	}
	else
		directions = routingOddEven(current, source, destination);	
	
	if (!(directions.size() > 0 && directions.size() <= 3)){
      cout << "\n STAMPACCHIO :";
      cout << source << endl;
      cout << destination << endl;
      cout << current << endl;
	}
	assert(directions.size() > 0 && directions.size() <= 3);
	return directions;	
}

vector < int >NoximRouter::routingDyAD(const NoximCoord & current,
				       const NoximCoord & source,
				       const NoximCoord & destination)
{
    vector < int >directions;

    directions = routingOddEven(current, source, destination);

    if (!inCongestion())
	directions.resize(1);

    return directions;
}

vector<int> NoximRouter::routingFullyAdaptive   (const NoximCoord& current                          , const NoximCoord& destination){
    vector < int >directions;

    if        (destination.x == current.x || destination.y == current.y)
		return routingXYZ(current, destination);

    if        (destination.x > current.x && destination.y < current.y) {
		directions.push_back(DIRECTION_NORTH);
		directions.push_back(DIRECTION_EAST);
    } else if (destination.x > current.x && destination.y > current.y) {
		directions.push_back(DIRECTION_SOUTH);
		directions.push_back(DIRECTION_EAST);
    } else if (destination.x < current.x && destination.y > current.y) {
		directions.push_back(DIRECTION_SOUTH);
		directions.push_back(DIRECTION_WEST);
    } else {
		directions.push_back(DIRECTION_NORTH);
		directions.push_back(DIRECTION_WEST);
    }

    return directions;
}
vector<int> NoximRouter::routingTableBased      (const NoximCoord& current, const int dir_in        , const NoximCoord& destination){
    NoximAdmissibleOutputs ao =
	routing_table.getAdmissibleOutputs(dir_in, coord2Id(destination));

    if (ao.size() == 0) {
	cout << "dir: " << dir_in << ", (" << current.x << "," << current.
	    y << ") --> " << "(" << destination.x << "," << destination.
	    y << ")" << endl << coord2Id(current) << "->" <<
	    coord2Id(destination) << endl;
    }
    assert(ao.size() > 0);
    //-----
    /*
       vector<int> aov = admissibleOutputsSet2Vector(ao);
       cout << "dir: " << dir_in << ", (" << current.x << "," << current.y << ") --> "
       << "(" << destination.x << "," << destination.y << "), outputs: ";
       for (int i=0; i<aov.size(); i++)
       cout << aov[i] << ", ";
       cout << endl;
     */
    //-----
    return admissibleOutputsSet2Vector(ao);
}
vector<int> NoximRouter::routingDLADR           (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination,const int select_routing, int dw_layer){
	switch ( select_routing ){
	case ROUTING_XYZ                 :return routingXYZ         (                 current, destination                   );break;
	case ROUTING_WEST_FIRST          :return routingWestFirst   (                 current, destination                   );break;
	case ROUTING_DOWNWARD_CROSS_LAYER:return DW_layerSelFunction( select_routing, current, destination, source, dw_layer );break;
	default                          :cout<<getCurrentCycleNum()<<":Wrong with Cross-Layer!"<<endl;
	                                  cout<<"Current    :"<<current<<endl;
									  cout<<"Source     :"<<source <<endl;
									  cout<<"Destination:"<<destination<<endl;
									  cout<<"Select routing:"<<select_routing<<endl;assert(false)                                 ;break;
	}
}
vector<int> NoximRouter::routingDLAR            (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination,const int select_routing, int dw_layer){
	switch ( select_routing ){
	case ROUTING_WEST_FIRST          :return routingOddEven_Z   (                 current, source, destination           );break;
	case ROUTING_XYZ                 :return DW_layerSelFunction( select_routing, current, destination, source, dw_layer );break;
	case ROUTING_DOWNWARD_CROSS_LAYER:return DW_layerSelFunction( select_routing, current, destination, source, dw_layer );break;
	default                          :cout<<"Wrong with Cross-Layer!"<<select_routing<<endl;assert(false)                 ;break;
	}
}
vector<int> NoximRouter::routingDLDR            (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination,const int select_routing, int dw_layer){
	switch ( select_routing ){
	case ROUTING_XYZ                 :return routingXYZ         (                 current, destination                   );break;
	case ROUTING_WEST_FIRST          :return routingXYZ         (                 current, destination                   );break;
	//case ROUTING_DOWNWARD_CROSS_LAYER:return DW_layerSelFunction( select_routing, current, destination, source, dw_layer );break;
	default                          :cout<<"Wrong with Cross-Layer!"<<endl;assert(false)                                 ;break;
	}
}
vector<int> NoximRouter::routingTLAR_DW         (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination){
	vector<int> directions;
	int layer = NoximGlobalParams::mesh_dim_z - 1 ;	//���ܦ�packet���Ӧb����layer��XY�ǻ�
	   if(current.z < layer && (current.x != destination.x || current.y != destination.y) )  
		{
            directions.push_back(DIRECTION_DOWN);
		}
		else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) )	//�b�����Hxy routing��
		{
			switch (NoximGlobalParams::routing_algorithm){
			case ROUTING_DLAR : directions = routingOddEven_Z(current, source, destination);break;
			case ROUTING_DLADR: directions = routingWestFirst(current,         destination);break;
			default           :	directions = routingXYZ      (current,         destination);break;
			}
		}
		else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z)
		{	
			directions.push_back(DIRECTION_UP);
		}	
		else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z)  
		{	
			directions.push_back(DIRECTION_DOWN);
		}	
		else 
		{ 
			cout<<"ERROR! Out of condition!?!?"<<endl;
			cout<<"current: ("<<current.x<<","<<current.y<<","<<current.z<<")"<<endl;
			cout<<"destination: ("<<destination.x<<","<<destination.y<<","<<destination.z<<")"<<endl;
			cout<<"layer: "<<layer<<endl;
			exit(1);			
		}
  return directions;
}
vector<int> NoximRouter::routingTLAR_DW_VBDR    (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination){
	vector<int> directions;
	int max = 0;
	int layer = 3;
	int free_slot[4];
	NoximNoP_data nop_tmp[4];
	for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ ){
		nop_tmp[i] = vertical_free_slot_in[i].read();
		free_slot[i] = 0 ;
		}
	if ( destination.x > current.x ){
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_EAST].free_slots;
	}
	else{
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_WEST].free_slots;
	}
	if ( destination.y > current.y ){
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_SOUTH].free_slots;
	}
	else{
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_NORTH].free_slots;
	}
	for( int i = 2 ; i < NoximGlobalParams::mesh_dim_z ; i++ ){
		if( free_slot[i] > max ){
			layer = i;
			max = free_slot[i];
		}		
	}
	
	if(current.z < layer && (current.x != destination.x || current.y != destination.y) )  
		{
			//Increasing Path Diversity
			if( !on_off_neighbor[0] && !on_off_neighbor[1] && !on_off_neighbor[2] && !on_off_neighbor[3] )
				directions = routingXYZ(current, destination);
			directions.push_back(DIRECTION_DOWN);
		}
		else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) )	//�b�����Hxy routing��
		{
			switch (NoximGlobalParams::routing_algorithm){
			case ROUTING_DLAR : directions = routingOddEven_Z(current, source, destination);break;
			case ROUTING_DLADR: directions = routingWestFirst(current,         destination);break;
			default           :	directions = routingXYZ      (current,         destination);break;
			}
		}
		else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z)	//xy�ۦP, z��V���W��
		{	
			directions.push_back(DIRECTION_UP);
		}	
		else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z)  //xy�ۦP, z��V���U��
		{	
			directions.push_back(DIRECTION_DOWN);
		}	
		else 
		{ 
			cout<<"ERROR! Out of condition!?!?"<<endl;
			cout<<"current: ("<<current.x<<","<<current.y<<","<<current.z<<")"<<endl;
			cout<<"destination: ("<<destination.x<<","<<destination.y<<","<<destination.z<<")"<<endl;
			cout<<"layer: "<<layer<<endl;
			exit(1);			
		}
	return directions;
}
vector<int> NoximRouter::routingTLAR_DW_ODWL_IPD(const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination, int dw_layer){
	vector<int> directions;
	int layer = dw_layer;
	if(current.z < layer && (current.x != destination.x || current.y != destination.y) ){
		if( NoximGlobalParams::routing_algorithm == ROUTING_DLAR )
			directions = routingOddEven(current, source, destination);
		else if ( NoximGlobalParams::routing_algorithm == ROUTING_DLADR )
			directions = routingWestFirst(current, destination);
		else
			directions = routingXYZ(current, destination);
			
		for(int i = directions.size() - 1  ; i >= 0 ; i--){
			if ( directions[i] < 4 ) //for lateral direction
				if ( on_off_neighbor[ directions[i] ].read() == 1 ){//if the direction is throttled
					directions.erase( directions.begin() + i);//then erase that way	
			}
		}
	
		if(current.z == destination.z && ( (current.x-destination.x== 1 && current.y==destination.y)	//while the dest. is one hop to source, don't downward and transmit directly
										 ||(current.x-destination.x==-1 && current.y==destination.y) 
										 ||(current.y-destination.y== 1 && current.x==destination.x) 
										 ||(current.y-destination.y==-1 && current.x==destination.x) ))
			directions = routingOddEven(current, source, destination);
		else 
			directions.push_back(DIRECTION_DOWN);
	}
	else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) ){	//�b�����Hxy routing��
		switch (NoximGlobalParams::routing_algorithm){
			case ROUTING_DLAR : directions = routingOddEven_Z(current, source, destination);break;
			case ROUTING_DLADR: directions = routingWestFirst(current,         destination);break;
			default           :	directions = routingXYZ      (current,         destination);break;
		}
	}
	else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z){	//xy�ۦP, z��V���W��
		directions.push_back(DIRECTION_UP);
	}	
	else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z){  //xy�ۦP, z��V���U��
		directions.push_back(DIRECTION_DOWN);
	}	
	else{ 
		cout<<"ERROR! Out of condition!?!?"<<endl;
		cout<<"current:     "<<current    <<endl;
		cout<<"destination: "<<destination<<endl;
		cout<<"layer:       "<<layer      <<endl;
		exit(1);			
	}
	if( directions.size() == 0 ){
		cout<<"current:     "<<current    <<endl;
		cout<<"source:      "<<source     <<endl;
		cout<<"destination: "<<destination<<endl;
		assert(false);
		}
	return directions;
}
vector<int> NoximRouter::routingTLAR_DW_ADWL    (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination){
	vector<int> directions;
	int max = 0;
	int layer = 3;              //���ܦ�packet���Ӧb����layer��XY�ǻ�
	int free_slot[4];
	NoximNoP_data nop_tmp[4];
	for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ ){
		nop_tmp[i] = vertical_free_slot_in[i].read();
		free_slot[i] = 0 ;
		}
	if ( destination.x > current.x ){
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_EAST].free_slots;
	}
	else{
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_WEST].free_slots;
	}
	if ( destination.y > current.y ){
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_SOUTH].free_slots;
	}
	else{
		for( int i = 0 ; i < NoximGlobalParams::mesh_dim_z ; i++ )
			free_slot[i] = nop_tmp[i].channel_status_neighbor[DIRECTION_NORTH].free_slots;
	}
	for( int i = 3 ; i < NoximGlobalParams::mesh_dim_z ; i++ ){
		if( free_slot[i] > max ){
			layer = i;
			max = free_slot[i];
		}		
	}	
	
	if(current.z < layer && (current.x != destination.x || current.y != destination.y) )  
		{
			directions.push_back(DIRECTION_DOWN);
		}
		else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) )	//�b�����Hxy routing��
		{
			switch (NoximGlobalParams::routing_algorithm){
			case ROUTING_DLAR : directions = routingOddEven_Z(current, source, destination);break;
			case ROUTING_DLADR: directions = routingWestFirst(current,         destination);break;
			default           :	directions = routingXYZ      (current,         destination);break;
			}
		}
		else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z)	//xy�ۦP, z��V���W��
		{	
			directions.push_back(DIRECTION_UP);
		}	
		else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z)  //xy�ۦP, z��V���U��
		{	
			directions.push_back(DIRECTION_DOWN);
		}	
		else 
		{ 
			cout<<"ERROR! Out of condition!?!?"<<endl;
			cout<<"current: ("<<current.x<<","<<current.y<<","<<current.z<<")"<<endl;
			cout<<"destination: ("<<destination.x<<","<<destination.y<<","<<destination.z<<")"<<endl;
			cout<<"layer: "<<layer<<endl;
			exit(1);			
		}
	return directions;
}
vector<int> NoximRouter::routingTLAR_DW_ODWL    (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination, int dw_layer){
	vector<int> directions;
	int layer = dw_layer;
	
	if(current.z < layer && (current.x != destination.x || current.y != destination.y) )  
		{
			//Increasing Path Diversity
			if( !on_off_neighbor[0] && !on_off_neighbor[1] && !on_off_neighbor[2] && !on_off_neighbor[3] && (current.z == source.z) ){
				if ( NoximGlobalParams::routing_algorithm == ROUTING_DLAR ){
					directions = routingOddEven_Z(current, source, destination);
					bool adaptive = true;
					for(int i = 0 ; i < directions.size() ; i++){
						NoximCoord sour =source;
						NoximCoord fake =id2Coord( getNeighborId(local_id, directions[i]) ); 
						if( !Adaptive_ok( sour, fake ) )
							adaptive = false;
					}
					if(!adaptive)directions.clear();
				}
				else
					directions = routingXYZ(current, destination);
			}
			if(current.z == destination.z && ( (current.x-destination.x== 1 && current.y==destination.y)	//while the dest. is one hop to source, don't downward and transmit directly
										 ||(current.x-destination.x==-1 && current.y==destination.y) 
										 ||(current.y-destination.y== 1 && current.x==destination.x) 
										 ||(current.y-destination.y==-1 && current.x==destination.x) ))
				directions = routingOddEven_Z(current, source, destination);
			else
				directions.push_back(DIRECTION_DOWN);
		}
		else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) )	//�b�����Hxy routing��
		{
			switch (NoximGlobalParams::routing_algorithm){
			case ROUTING_DLAR : directions = routingOddEven_Z(current, source, destination);break;
			case ROUTING_DLADR: directions = routingWestFirst(current,         destination);break;
			default           :	directions = routingXYZ      (current,         destination);break;
			}
		}
		else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z)	//xy�ۦP, z��V���W��
		{	
			directions.push_back(DIRECTION_UP);
		}	
		else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z)  //xy�ۦP, z��V���U��
		{	
			directions.push_back(DIRECTION_DOWN);
		}	
		else 
		{ 
			cout<<"ERROR! Out of condition!?!?"<<endl;
			cout<<"Router ID:"<<local_id<<endl;
			cout<<"current: ("<<current.x<<","<<current.y<<","<<current.z<<")"<<endl;
			cout<<"destination: ("<<destination.x<<","<<destination.y<<","<<destination.z<<")"<<endl;
			cout<<"layer: "<<layer<<endl;
			assert(0);
			exit(1);			
		}
	assert(directions.size() > 0 );
	assert(directions.size() <= 3);
	return directions;
}	
vector<int> NoximRouter::routingTLAR_DW_IPD     (const NoximCoord& current, const NoximCoord& source, const NoximCoord& destination){
	vector<int> directions;
	int layer = NoximGlobalParams::mesh_dim_z - 1;	
	
	if(current.z < layer && (current.x != destination.x || current.y != destination.y) ) {
		//Increasing Path Diversity
		if( !on_off_neighbor[0] && !on_off_neighbor[1] && !on_off_neighbor[2] && !on_off_neighbor[3] && (current.z == source.z)){
			if      ( NoximGlobalParams::routing_algorithm == ROUTING_DLAR ){
				directions = routingOddEven_Z(current, source, destination);
				bool adaptive = true;
				for(int i = 0 ; i < directions.size() ; i++){
					NoximCoord sour =source;
					NoximCoord fake =id2Coord( getNeighborId(local_id, directions[i]) ); 
					if( !Adaptive_ok( sour, fake ) )
						adaptive = false;
				}
				if(!adaptive)directions.clear();
				}
			else
				directions = routingXYZ(current, destination);
		}
		directions.push_back(DIRECTION_DOWN);
	}
	else if(current.z >=layer && (current.x != destination.x || current.y != destination.y) ){
		switch (NoximGlobalParams::routing_algorithm){
		case ROUTING_DLAR : directions = routingOddEven_Z(current, source, destination);break;
		case ROUTING_DLADR: directions = routingWestFirst(current,         destination);break;
		default           :	directions = routingXYZ      (current,         destination);break;
		}
	}
	else if((current.x == destination.x && current.y == destination.y) && current.z > destination.z){
		directions.push_back(DIRECTION_UP);
	}	
	else if((current.x == destination.x && current.y == destination.y) && current.z < destination.z){
		directions.push_back(DIRECTION_DOWN);
	}	
	else{ 
		cout<<"ERROR! Out of condition!?!?"<<endl;
		cout<<"current: ("<<current.x<<","<<current.y<<","<<current.z<<")"<<endl;
		cout<<"destination: ("<<destination.x<<","<<destination.y<<","<<destination.z<<")"<<endl;
		cout<<"layer: "<<layer<<endl;
		exit(1);			
	}
	return directions;
}	

void NoximRouter::configure(const int _id,
			    const double _warm_up_time,
			    const unsigned int _max_buffer_size,
			    NoximGlobalRoutingTable & grt)
{
	_emergency_trigger_flag = false;
	_man_set_max_buffer_size = _max_buffer_size;
    local_id = _id;
    stats.configure(_id, _warm_up_time);

    start_from_port = DIRECTION_LOCAL;

    if (grt.isValid())
	routing_table.configure(grt, _id);

	if( !NoximGlobalParams::buffer_alloc )
		for (int i = 0; i < DIRECTIONS + 1; i++)
			buffer[i].SetMaxBufferSize(_max_buffer_size);
	/*else{
		int z = id2Coord(_id).z;
		switch(z){
			case 0:
				for (int i = 0; i < 4; i++)
					buffer[i].SetMaxBufferSize(BUFFER_L_LAYER_0);
				buffer[DIRECTION_DOWN].SetMaxBufferSize(BUFFER_D_LAYER_0);
				buffer[DIRECTION_UP  ].SetMaxBufferSize(BUFFER_U_LAYER_0);
				break;
			case 1:
				for (int i = 0; i < 4; i++)
					buffer[i].SetMaxBufferSize(BUFFER_L_LAYER_1);
				buffer[DIRECTION_DOWN].SetMaxBufferSize(BUFFER_D_LAYER_1);
				buffer[DIRECTION_UP  ].SetMaxBufferSize(BUFFER_U_LAYER_1);
				break;
			case 2:
				for (int i = 0; i < 4; i++)
					buffer[i].SetMaxBufferSize(BUFFER_L_LAYER_2);
				buffer[DIRECTION_DOWN].SetMaxBufferSize(BUFFER_D_LAYER_2);
				buffer[DIRECTION_UP  ].SetMaxBufferSize(BUFFER_U_LAYER_2);
				break;
			case 3:
				for (int i = 0; i < 4; i++)
					buffer[i].SetMaxBufferSize(BUFFER_L_LAYER_3);
				buffer[DIRECTION_DOWN].SetMaxBufferSize(BUFFER_D_LAYER_3);
				buffer[DIRECTION_UP  ].SetMaxBufferSize(BUFFER_U_LAYER_3);
				break;
		}
	}*/
}

/***** Thermal-aware Dynamic Buffer Allocation (Jimmy added on 2014.12.17) ******/
void NoximRouter::TBDB()
{

	//if(NoximGlobalParams::Buffer_Allocation == BUFFER_ALLOCATION/*&& stats.temperature>92.2*/)//Calvin add on 2015/07/27 
	//{
		//Check the Rate of Temperature Increasing (RTI) for further buffer depth allocation
		float Temp_diff_east  ;
		float Temp_diff_west  ;
		float Temp_diff_north ;
		float Temp_diff_south ;
		float Temp_diff_up    ;
		float Temp_diff_down  ;
		/*float Try_param_self[4] = {0}, //Calvin's work
			  Try_param_dir[4] = {0};*/
		float Try_param_self = 0, //Consider the vertical direction (Jimmy modified on 2017/05/02)
			  Try_param_dir[6] = {0},
			  Try_param_diff[6] = {0};
		int Try_flag, Try_flag2= 0;
		
		
		//if( reset.read() ){
			Temp_diff_east  = 0;
			Temp_diff_west  = 0;
			Temp_diff_north = 0;
			Temp_diff_south = 0;
			Temp_diff_up    = 0;
			Temp_diff_down  = 0;
		//}
		//else{
		//	int CurrentCycle    = getCurrentCycleNum();
		//	int CurrentCycleMod = (getCurrentCycleNum() % (int) (TEMP_REPORT_PERIOD));
			
		//	if( CurrentCycleMod == 0  ){
				buffer_allocation_time = 0;
							
				//Start to dynamically adjust buffer depth based on the results of RTI
				//Assume that The current router is A.
				//Case of RTI(A)<RTI(B) ---> The length of input buffer, which direct to B, is decrease.
				
				//* setup Try_param of all directions. */
			
		/*cout<<"Local id: "<<local_id<<" "<<endl;
		for (int i = 0; i < DIRECTIONS + 1; i++) {
			cout<<"DIRECTION = "<< i <<" : "<< buffer[i].GetMaxBufferSize()<<endl;
		}*/
				
				
				if(NoximGlobalParams::TBDB_Type == TBDB_Type_Original){		//RTI
					Temp_diff_east  = (stats.pre_temperature1-stats.temperature) - PDT_neighbor[DIRECTION_EAST].read(); //stats.pre_temperature1=the prediciton result after 1 delta t
					Temp_diff_west  = (stats.pre_temperature1-stats.temperature) - PDT_neighbor[DIRECTION_WEST].read(); //stats.temperature=the real temperature
					Temp_diff_north = (stats.pre_temperature1-stats.temperature) - PDT_neighbor[DIRECTION_NORTH].read();
					Temp_diff_south = (stats.pre_temperature1-stats.temperature) - PDT_neighbor[DIRECTION_SOUTH].read();
					Temp_diff_up    = (stats.pre_temperature1-stats.temperature) - PDT_neighbor[DIRECTION_UP].read();
					Temp_diff_down  = (stats.pre_temperature1-stats.temperature) - PDT_neighbor[DIRECTION_DOWN].read();			
				}
			
			
				else if (NoximGlobalParams::TBDB_Type == TBDB_Type_PRE){
			
					Temp_diff_east  = (stats.pre_temperature1) - PDT_neighbor[DIRECTION_EAST].read();
					Temp_diff_west  = (stats.pre_temperature1) - PDT_neighbor[DIRECTION_WEST].read();
					Temp_diff_north = (stats.pre_temperature1) - PDT_neighbor[DIRECTION_NORTH].read();
					Temp_diff_south = (stats.pre_temperature1) - PDT_neighbor[DIRECTION_SOUTH].read();
					Temp_diff_up    = (stats.pre_temperature1) - PDT_neighbor[DIRECTION_UP].read();
					Temp_diff_down  = (stats.pre_temperature1) - PDT_neighbor[DIRECTION_DOWN].read();
				//cout<<"###### PDT_neighbor[DIRECTION_EAST].read() #######: "<<PDT_neighbor[DIRECTION_EAST].read()<<endl;
				}
			
			
				else if (NoximGlobalParams::TBDB_Type == TBDB_Type_GAME){
					int local_total_buf_length = 0;
					
					
					for (int i = 0 ; i < sizeof(Try_param_dir)/sizeof(Try_param_dir[0]); i++) {  //sizeof�ΨӨ��o�ܼƪ��줸�դj�p�C
						int neighbor_total_buf_length = 0;
						
						for (int j = 0 ; j < sizeof(Try_param_dir)/sizeof(Try_param_dir[0]); j++) {
							neighbor_total_buf_length += buf_neighbor[j][i].read();
						}
						//cout<<"Local ID = "<<local_id<<": ";
						//cout<<"###### neighbor_total_buf_length: "<<neighbor_total_buf_length<<" ####"<<endl;
						local_total_buf_length += buffer[i].GetMaxBufferSize();
						Try_param_dir[i] = (1.0/neighbor_total_buf_length) * PDT_neighbor[i].read();						
					}
					
					Try_param_self = (1.0/local_total_buf_length) * stats.pre_temperature1;
					for (int i = 0 ; i < sizeof(Try_param_dir)/sizeof(Try_param_dir[0]); i++){
						Try_param_diff[i] = Try_param_self - Try_param_dir[i];
						if(Try_param_diff[i]>0) Try_flag++;
					}
					
				}
		
		
			//	 printf("TBDB_Type_GAME  1  \n");
				//* Compare Try_param_self with all directions. */
				/*for ( i = 0 ; i < sizeof(Try_param_dir)/sizeof(Try_param_dir[0]); i++) {
					printf("i: %d, free_slots:%d, temperature: %f, free_slots_neighbor: %d, PDT_neighbor: %f;\n", i, free_slots[i], stats.temperature, free_slots_neighbor[i], PDT_neighbor[i].read());
					if ( Try_param_self[i] < Try_param_dir[i]) {
						printf("enter\n");
						Try_flag++;
						//break;
					}*/
				//}
				//}
				
			
				//}
				
				if(NoximGlobalParams::TBDB_Type == TBDB_Type_Original){
				
					if(Temp_diff_east<0){
						if(buffer[DIRECTION_EAST].GetMaxBufferSize()>2){	
							buffer[DIRECTION_EAST].SetMaxBufferSize(buffer[DIRECTION_EAST].GetMaxBufferSize()-2);							
							//buf_budget++;
						}
						
					}
					                						      		
					if(Temp_diff_west<0){
						if(buffer[DIRECTION_WEST].GetMaxBufferSize()>2){
							buffer[DIRECTION_WEST].SetMaxBufferSize(buffer[DIRECTION_WEST].GetMaxBufferSize()-2);
							//buf_budget++;
						}
					}					                	
					
					if(Temp_diff_north<0){
						if(buffer[DIRECTION_NORTH].GetMaxBufferSize()>2){
							buffer[DIRECTION_NORTH].SetMaxBufferSize(buffer[DIRECTION_NORTH].GetMaxBufferSize()-2);
							//buf_budget++;
						}
					}
					if(Temp_diff_south<0){
						if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()>2){
							buffer[DIRECTION_SOUTH].SetMaxBufferSize(buffer[DIRECTION_SOUTH].GetMaxBufferSize()-2);
							//buf_budget++;
						}
					}	
					if(Temp_diff_up<0){
					if(buffer[DIRECTION_UP].GetMaxBufferSize()>2){
							buffer[DIRECTION_UP].SetMaxBufferSize(buffer[DIRECTION_UP].GetMaxBufferSize()-2);
							//buf_budget++;
						}
					}
					if(Temp_diff_down<0){
						if(buffer[DIRECTION_DOWN].GetMaxBufferSize()>2){
							buffer[DIRECTION_DOWN].SetMaxBufferSize(buffer[DIRECTION_DOWN].GetMaxBufferSize()-2);
							//buf_budget++;
						}
					}
	            	
					//Assume that The current router is A.
					//Case of RTI(A)>RTI(B) ---> The length of input buffer, which direct to B, is increase.
					if(Temp_diff_east>0){
						if(buffer[DIRECTION_EAST].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_EAST].SetMaxBufferSize(buffer[DIRECTION_EAST].GetMaxBufferSize()+2);
							//buf_budget--;
						}
					}
					if(Temp_diff_west>0){
						if(buffer[DIRECTION_WEST].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_WEST].SetMaxBufferSize(buffer[DIRECTION_WEST].GetMaxBufferSize()+2);
							//buf_budget--;
						}
					}
					if(Temp_diff_north>0){
						if(buffer[DIRECTION_NORTH].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_NORTH].SetMaxBufferSize(buffer[DIRECTION_NORTH].GetMaxBufferSize()+2);
							//buf_budget--;
						}
					}
					if(Temp_diff_south>0){
						if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_SOUTH].SetMaxBufferSize(buffer[DIRECTION_SOUTH].GetMaxBufferSize()+2);
							//buf_budget--;
						}
					}
					if(Temp_diff_up>0){
						if(buffer[DIRECTION_UP].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_UP].SetMaxBufferSize(buffer[DIRECTION_UP].GetMaxBufferSize()+2);
							//buf_budget--;
						}
					}
					if(Temp_diff_down>0){
						if(buffer[DIRECTION_DOWN].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_DOWN].SetMaxBufferSize(buffer[DIRECTION_DOWN].GetMaxBufferSize()+2);
							//buf_budget--;
						}
					}
				}
				
				if(NoximGlobalParams::TBDB_Type == TBDB_Type_PRE){
				
					if(Temp_diff_east<0){
						//if(buffer[DIRECTION_EAST].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_EAST].GetMaxBufferSize()>1){	
							buffer[DIRECTION_EAST].SetMaxBufferSize(buffer[DIRECTION_EAST].GetMaxBufferSize()-1);							
							//buf_budget++;
						}
						
					}
					                						      		
					if(Temp_diff_west<0){
						//if(buffer[DIRECTION_WEST].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_WEST].GetMaxBufferSize()>1){
							buffer[DIRECTION_WEST].SetMaxBufferSize(buffer[DIRECTION_WEST].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}					                	
					
					if(Temp_diff_north<0){
						//if(buffer[DIRECTION_NORTH].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_NORTH].GetMaxBufferSize()>1){
							buffer[DIRECTION_NORTH].SetMaxBufferSize(buffer[DIRECTION_NORTH].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}
					if(Temp_diff_south<0){
						//if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()>1){
							buffer[DIRECTION_SOUTH].SetMaxBufferSize(buffer[DIRECTION_SOUTH].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}	
					if(Temp_diff_up<0){
					//if(buffer[DIRECTION_UP].GetMaxBufferSize()>2){
					if(buffer[DIRECTION_UP].GetMaxBufferSize()>1){
							buffer[DIRECTION_UP].SetMaxBufferSize(buffer[DIRECTION_UP].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}
					if(Temp_diff_down<0){
						//if(buffer[DIRECTION_DOWN].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_DOWN].GetMaxBufferSize()>1){
							buffer[DIRECTION_DOWN].SetMaxBufferSize(buffer[DIRECTION_DOWN].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}
	            	
					//Assume that The current router is A.
					//Case of RTI(A)>RTI(B) ---> The length of input buffer, which direct to B, is increase.
					if(Temp_diff_east>0){
						if(buffer[DIRECTION_EAST].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_EAST].SetMaxBufferSize(buffer[DIRECTION_EAST].GetMaxBufferSize()+1);
							//buf_budget--;
						}
					}
					if(Temp_diff_west>0){
						if(buffer[DIRECTION_WEST].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_WEST].SetMaxBufferSize(buffer[DIRECTION_WEST].GetMaxBufferSize()+1);
							//buf_budget--;
						}
					}
					if(Temp_diff_north>0){
						if(buffer[DIRECTION_NORTH].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_NORTH].SetMaxBufferSize(buffer[DIRECTION_NORTH].GetMaxBufferSize()+1);
							//buf_budget--;
						}
					}
					if(Temp_diff_south>0){
						if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_SOUTH].SetMaxBufferSize(buffer[DIRECTION_SOUTH].GetMaxBufferSize()+1);
							//buf_budget--;
						}
					}
					if(Temp_diff_up>0){
						if(buffer[DIRECTION_UP].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_UP].SetMaxBufferSize(buffer[DIRECTION_UP].GetMaxBufferSize()+1);
							//buf_budget--;
						}
					}
					if(Temp_diff_down>0){
						if(buffer[DIRECTION_DOWN].GetMaxBufferSize()<NoximGlobalParams::buffer_depth){
							buffer[DIRECTION_DOWN].SetMaxBufferSize(buffer[DIRECTION_DOWN].GetMaxBufferSize()+1);
							//buf_budget--;
						}
					}
				}
				
			///}
			//}	
			 else if (NoximGlobalParams::TBDB_Type == TBDB_Type_GAME ){
					
					/*if(Try_param_self[DIRECTION_EAST] > Try_param_dir[DIRECTION_WEST])
							{Try_flag++;}
					if(Try_param_self[DIRECTION_EAST]<Try_param_dir[DIRECTION_WEST])
							{Try_flag2++;}
					if(Try_param_self[DIRECTION_WEST] > Try_param_dir[DIRECTION_EAST])
							{Try_flag++;}
					if(Try_param_self[DIRECTION_WEST]<Try_param_dir[DIRECTION_EAST])
							{Try_flag2++;}
					if(Try_param_self[DIRECTION_NORTH] > Try_param_dir[DIRECTION_SOUTH])
							{Try_flag++;}
					if(Try_param_self[DIRECTION_NORTH]<Try_param_dir[DIRECTION_SOUTH])
								{Try_flag2++;}
					if(Try_param_self[DIRECTION_SOUTH] > Try_param_dir[DIRECTION_NORTH])
							{Try_flag++;}
					if(Try_param_self[DIRECTION_SOUTH]<Try_param_dir[DIRECTION_NORTH])
							{Try_flag2++;}
					if(Try_param_self[DIRECTION_DOWN] > Try_param_dir[DIRECTION_UP])
							{Try_flag++;}
					if(Try_param_self[DIRECTION_DOWN]<Try_param_dir[DIRECTION_UP])
							{Try_flag2++;}
					if(Try_param_self[DIRECTION_UP] > Try_param_dir[DIRECTION_DOWN])
							{Try_flag++;}
					if(Try_param_self[DIRECTION_UP]<Try_param_dir[DIRECTION_DOWN])
							{Try_flag2++;}*/
						//printf("TRY_F %d\n, TRY_F2 %d",Try_flag,Try_flag2);
						//* Try_param_self does greater than all directions. */
					
					if(Try_param_diff[0]<0){
						//if(buffer[DIRECTION_NORTH].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_NORTH].GetMaxBufferSize()>1){
							buffer[DIRECTION_NORTH].SetMaxBufferSize(buffer[DIRECTION_NORTH].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}
					if(Try_param_diff[1]<0){
						//if(buffer[DIRECTION_EAST].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_EAST].GetMaxBufferSize()>1){
							buffer[DIRECTION_EAST].SetMaxBufferSize(buffer[DIRECTION_EAST].GetMaxBufferSize()-1);
							//buf_budget++;
						}
						
					}
					if(Try_param_diff[2]<0){
						//if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()>1){
							buffer[DIRECTION_SOUTH].SetMaxBufferSize(buffer[DIRECTION_SOUTH].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}                						      		
					if(Try_param_diff[3]<0){
						//if(buffer[DIRECTION_WEST].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_WEST].GetMaxBufferSize()>1){
							buffer[DIRECTION_WEST].SetMaxBufferSize(buffer[DIRECTION_WEST].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}					                		
					if(Try_param_diff[4]<0){
					//if(buffer[DIRECTION_UP].GetMaxBufferSize()>2){
					if(buffer[DIRECTION_UP].GetMaxBufferSize()>1){
							buffer[DIRECTION_UP].SetMaxBufferSize(buffer[DIRECTION_UP].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}
					if(Try_param_diff[5]<0){
						//if(buffer[DIRECTION_DOWN].GetMaxBufferSize()>2){
						if(buffer[DIRECTION_DOWN].GetMaxBufferSize()>1){
							buffer[DIRECTION_DOWN].SetMaxBufferSize(buffer[DIRECTION_DOWN].GetMaxBufferSize()-1);
							//buf_budget++;
						}
					}
					
					/*if (Try_flag2>3){
				
			
						if(buffer[DIRECTION_EAST].GetMaxBufferSize()>2){
							buffer[DIRECTION_EAST].SetMaxBufferSize(buffer[DIRECTION_EAST].GetMaxBufferSize()-1);
							//buf_budget++;
							buffer_allocation_time++;
						}
		
						if(buffer[DIRECTION_WEST].GetMaxBufferSize()>2){
							buffer[DIRECTION_WEST].SetMaxBufferSize(buffer[DIRECTION_WEST].GetMaxBufferSize()-1);
							//buf_budget++;
							buffer_allocation_time++;
						}
		
						if(buffer[DIRECTION_NORTH].GetMaxBufferSize()>2){
							buffer[DIRECTION_NORTH].SetMaxBufferSize(buffer[DIRECTION_NORTH].GetMaxBufferSize()-1);
							//buf_budget++;
							buffer_allocation_time++;
						}
			
						if(buffer[DIRECTION_SOUTH].GetMaxBufferSize()>2){
							buffer[DIRECTION_SOUTH].SetMaxBufferSize(buffer[DIRECTION_SOUTH].GetMaxBufferSize()-1);
							//buf_budget++;
							buffer_allocation_time++;
						}
			
						if(buffer[DIRECTION_UP].GetMaxBufferSize()>2){
							buffer[DIRECTION_UP].SetMaxBufferSize(buffer[DIRECTION_UP].GetMaxBufferSize()-1);
							//buf_budget++;
							buffer_allocation_time++;
						}
			
						if(buffer[DIRECTION_DOWN].GetMaxBufferSize()>2){
							buffer[DIRECTION_DOWN].SetMaxBufferSize(buffer[DIRECTION_DOWN].GetMaxBufferSize()-1);
							//buf_budget++;
							buffer_allocation_time++;
						}
					}*/
					if (Try_flag>3){ //add the consideration of interaction between the adjacent nodes (Jimmy modified on 2017/04/29)
				
						if((buffer[DIRECTION_EAST].GetMaxBufferSize()<NoximGlobalParams::buffer_depth) &&
						   (Try_param_self > Try_param_dir[DIRECTION_WEST])){
							buffer[DIRECTION_EAST].SetMaxBufferSize(buffer[DIRECTION_EAST].GetMaxBufferSize()+1);
							//buf_budget--;
							buffer_allocation_time++;
						}
			
						if((buffer[DIRECTION_WEST].GetMaxBufferSize()<NoximGlobalParams::buffer_depth) &&
						   (Try_param_self > Try_param_dir[DIRECTION_EAST])){
							buffer[DIRECTION_WEST].SetMaxBufferSize(buffer[DIRECTION_WEST].GetMaxBufferSize()+1);
							//buf_budget--;
							buffer_allocation_time++;
						}
			
						if((buffer[DIRECTION_NORTH].GetMaxBufferSize()<NoximGlobalParams::buffer_depth) &&
						   (Try_param_self > Try_param_dir[DIRECTION_SOUTH])){
							buffer[DIRECTION_NORTH].SetMaxBufferSize(buffer[DIRECTION_NORTH].GetMaxBufferSize()+1);
							//buf_budget--;
							buffer_allocation_time++;
						}
			
						if((buffer[DIRECTION_SOUTH].GetMaxBufferSize()<NoximGlobalParams::buffer_depth) &&
						   (Try_param_self > Try_param_dir[DIRECTION_NORTH])){
							buffer[DIRECTION_SOUTH].SetMaxBufferSize(buffer[DIRECTION_SOUTH].GetMaxBufferSize()+1);
							//buf_budget--;
							buffer_allocation_time++;
						}
			
						if((buffer[DIRECTION_UP].GetMaxBufferSize()<NoximGlobalParams::buffer_depth) &&
						   (Try_param_self > Try_param_dir[DIRECTION_DOWN])){
							buffer[DIRECTION_UP].SetMaxBufferSize(buffer[DIRECTION_UP].GetMaxBufferSize()+1);
							//buf_budget--;
							buffer_allocation_time++;
						}
			
						if((buffer[DIRECTION_DOWN].GetMaxBufferSize()<NoximGlobalParams::buffer_depth) &&
						   (Try_param_self > Try_param_dir[DIRECTION_UP])){
							buffer[DIRECTION_DOWN].SetMaxBufferSize(buffer[DIRECTION_DOWN].GetMaxBufferSize()+1);
							//buf_budget--;
							buffer_allocation_time++;
						}
					}
				
				}
		    //}
				
		//}	      
	//} 
	/*cout<<"Local id: "<<local_id<<" TD="<<Try_param_self<<endl;
	for (int i = 0; i < DIRECTIONS + 1; i++) {
		cout<<"DIRECTION = "<< i <<" : TD="<< Try_param_dir[i]<<endl;
	}*/
}
		
	
  

unsigned long NoximRouter::getRoutedFlits()
{
	unsigned long total = 0 ; 
	for (int i = 0; i < DIRECTIONS + 1; i++)
		total += routed_flits[i];
    return total;
}

unsigned long NoximRouter::getRoutedDWFlits()
{
	return routed_DWflits;
}

unsigned long NoximRouter::getRoutedFlits(int i)
{
    return routed_flits[i];
}

unsigned long NoximRouter::getWaitingTime(int i)
{
	return waiting[i];
}
unsigned long NoximRouter::getTotalWaitingTime()
{
	return _total_waiting;
} 
unsigned long NoximRouter::getRoutedPackets()
{
	return routed_packets;
}
void NoximRouter::CalcDelay( vector <int > & router_delay){
	_buffer_pkt_count = 0;
	_buffer_pkt_nw_delay = 0;
	_buffer_pkt_ni_delay = 0;
	_buffer_pkt_msg_delay= 0;
	for (int i = 0; i < DIRECTIONS + 1; i++){
		while( !buffer[i].IsEmpty() ){
			NoximFlit flit = buffer[i].Front();
			if( flit.flit_type == FLIT_TYPE_HEAD ){
				router_delay.push_back( (int)getCurrentCycleNum() - (int)flit.timestamp );
				_buffer_pkt_msg_delay += (int)getCurrentCycleNum() - (int)flit.timestamp    ;
				_buffer_pkt_ni_delay  += (int)getCurrentCycleNum() - (int)flit.timestamp_ni ;
				_buffer_pkt_nw_delay  += (int)getCurrentCycleNum() - (int)flit.timestamp_nw ;
				_buffer_pkt_count     ++;
			}
			buffer[i].Pop();
		}
	}
}

int NoximRouter::getFlitRoute(int i){
	NoximFlit flit = buffer[i].Front();
	NoximRouteData route_data;
	route_data.current_id = local_id;
	route_data.src_id     = (flit.arr_mid)?flit.mid_id:flit.src_id;
	route_data.dst_id     = (flit.arr_mid)?flit.dst_id:flit.mid_id;
	route_data.dir_in     = i;
	route_data.routing    = flit.routing_f;
	route_data.DW_layer   = flit.DW_layer;
	route_data.arr_mid    = flit.arr_mid;

	return route(route_data);
}

int NoximRouter::getDirAvailable(int i){
	
	return reservation_table.isAvailable(i);
	
}

double NoximRouter::getPower()
{
    //return stats.power.getPower();
    //return stats.power.getTransientRouterPower();
    return stats.power.getSteadyStateRouterPower();
}

int NoximRouter::reflexDirection(int direction) const
{
    switch( direction ){
	case DIRECTION_NORTH:return DIRECTION_SOUTH;break;
    case DIRECTION_EAST :return DIRECTION_WEST ;break;
    case DIRECTION_WEST :return DIRECTION_EAST ;break;
    case DIRECTION_SOUTH:return DIRECTION_NORTH;break;
	case DIRECTION_UP   :return DIRECTION_DOWN ;break;
    case DIRECTION_DOWN :return DIRECTION_UP   ;break;
	default             :return NOT_VALID      ;break;
	}
}

int NoximRouter::getNeighborId(int _id, int direction) const
{
    NoximCoord my_coord = id2Coord(_id);
    switch ( direction ) {
    case DIRECTION_NORTH: if (my_coord.y == 0                                ) return NOT_VALID;my_coord.y--;break;
    case DIRECTION_SOUTH: if (my_coord.y == NoximGlobalParams::mesh_dim_y - 1) return NOT_VALID;my_coord.y++;break;
    case DIRECTION_EAST : if (my_coord.x == NoximGlobalParams::mesh_dim_x - 1) return NOT_VALID;my_coord.x++;break;
    case DIRECTION_WEST : if (my_coord.x == 0                                ) return NOT_VALID;my_coord.x--;break;
	case DIRECTION_UP   : if (my_coord.z == 0                                ) return NOT_VALID;my_coord.z--;break;
	case DIRECTION_DOWN : if (my_coord.z == NoximGlobalParams::mesh_dim_z-1  ) return NOT_VALID;my_coord.z++;break;
    default             : cout << "direction not valid : " << direction; assert(false);
    }
    return coord2Id(my_coord);
}

//�Y�bemergency mode, �Btraffic�W�Ltraffic quota, �hthrottle (Jimmy modified on 2014.12.17)
void NoximRouter::TraffThrottlingProcess()	
{
	/*if( reset.read() )for(int i=0; i<4; i++)on_off[i].write(0);
	else              for(int i=0; i<4; i++)on_off[i].write( _emergency );*/ //Original program
	
	NoximCoord local = id2Coord(local_id);
	
	if( reset.read() ){
		for(int i=0; i<4; i++) on_off[i].write(0);
		for(int i=0; i<DIRECTIONS; i++){ //Note: If we want to notice the router in vertical. "4" should be chaged to "DIRECTIONS"
			buf[0][i].write(0);
			buf[1][i].write(0);
			buf[2][i].write(0);
			buf[3][i].write(0);
			buf[4][i].write(0);
			buf[5][i].write(0);
			buf[6][i].write(0);
			buf[7][i].write(0);
			//RST = 0;
			//DFS = 8;
		}
	}
	else{
		for(int i=0; i<4; i++) on_off[i].write( _emergency );
		for(int i=0; i<DIRECTIONS; i++){
			if(NoximGlobalParams::TBDB_Type == TBDB_Type_Original)
			{
				PDT[i].write (stats.pre_temperature1-stats.temperature);//(stats.pre_temperature1-stats.temperature);
			}
			else
			{
				PDT[i].write (stats.pre_temperature1);
			}
			if(throttling[local.x][local.y][local.z]){
				buf[0][i].write(100);
				buf[1][i].write(100);
				buf[2][i].write(100);
				buf[3][i].write(100);
				buf[4][i].write(100);
                buf[5][i].write(100);
                buf[6][i].write(100);
                buf[7][i].write(100);
			}
			else{
				if(NoximGlobalParams::TBDB_Type == TBDB_Type_Original){
					buf[0][i].write(buffer[DIRECTION_NORTH     ].Size());
					buf[1][i].write(buffer[DIRECTION_EAST      ].Size());
					buf[2][i].write(buffer[DIRECTION_SOUTH     ].Size());
					buf[3][i].write(buffer[DIRECTION_WEST      ].Size());
					buf[4][i].write(buffer[DIRECTION_UP        ].Size());
					buf[5][i].write(buffer[DIRECTION_DOWN      ].Size());
					buf[6][i].write(buffer[DIRECTION_LOCAL     ].Size());
					buf[7][i].write(buffer[DIRECTION_SEMI_LOCAL].Size());
				}
				else{
					buf[0][i].write(buffer[DIRECTION_NORTH     ].GetMaxBufferSize());
					buf[1][i].write(buffer[DIRECTION_EAST      ].GetMaxBufferSize());
					buf[2][i].write(buffer[DIRECTION_SOUTH     ].GetMaxBufferSize());
					buf[3][i].write(buffer[DIRECTION_WEST      ].GetMaxBufferSize());
					buf[4][i].write(buffer[DIRECTION_UP        ].GetMaxBufferSize());
					buf[5][i].write(buffer[DIRECTION_DOWN      ].GetMaxBufferSize());
					buf[6][i].write(buffer[DIRECTION_LOCAL     ].GetMaxBufferSize());
					buf[7][i].write(buffer[DIRECTION_SEMI_LOCAL].GetMaxBufferSize());
				}
				
			}
		}
	}
}

void NoximRouter::IntoEmergency(){
	_emergency  = true;
	_emergency_level = 1;
	_emergency_trigger_flag = true;
}

void NoximRouter::IntoSemiEmergency(int emergency_level){ //Jimmy added on 2012.04.12
	_emergency  = false;
	//_emergency_level = 1;
	_emergency_level = emergency_level;
}

void NoximRouter::OutOfEmergency(){
	_emergency       = false;
	_emergency_level = 0;
}

void NoximRouter::IntoCleanStage(){ //Jimmy added on 2017.05.02
	for (int i = 0; i < DIRECTIONS; i++) {	
		_reset_buf[i]       = buffer[i].GetMaxBufferSize();	
		buffer[i].SetMaxBufferSize(_man_set_max_buffer_size); 				
	}
}

void NoximRouter::OutOfCleanStage(){ //Jimmy modified on 2017.06.02
	NoximFlit f;
	NoximCoord flit_src;
	for (int i = 0; i < DIRECTIONS + 2; i++) {
		if ( !(buffer[i].IsEmpty()) ){		
			while(!(buffer[i].IsEmpty())){
				f = buffer[i].Front();
				
				if(f.flit_type == FLIT_TYPE_HEAD){
					flit_src = id2Coord(f.src_id);
					reShot[flit_src.x][flit_src.y][flit_src.z] += 1;
				}
				buffer[i].Pop();
			} 
		}				
		if(i<DIRECTIONS) buffer[i].SetMaxBufferSize(_reset_buf[i]);		
	}
	//if(local_id==0) cout<<(!buffer[DIRECTIONS].IsFull())<<"   "<<(!buffer[DIRECTIONS+1].IsFull())<<"   "<<(!_emergency)<<endl;	
}

bool NoximRouter::inCongestion()
{
    for (int i = 0; i < DIRECTIONS; i++) {
	int flits =        NoximGlobalParams::buffer_depth - free_slots_neighbor[i];
	if (flits > (int) (NoximGlobalParams::buffer_depth * NoximGlobalParams::dyad_threshold) )return true;
    }
    return false;
}
bool NoximRouter::Adaptive_ok( NoximCoord & sour, NoximCoord & dest)
{
	int x_diff = dest.x - sour.x;
	int y_diff = dest.y - sour.y;
	int z_diff = dest.z - sour.z;
    int x_search,y_search;

	for( int y_a = 0 ; y_a < abs(y_diff) + 1 ; y_a ++ )
	for( int x_a = 0 ; x_a < abs(x_diff) + 1 ; x_a ++ ){
		x_search = (x_diff > 0)?(sour.x + x_a):(sour.x - x_a);
		y_search = (y_diff > 0)?(sour.y + y_a):(sour.y - y_a);
		if( throttling[x_search][y_search][sour.z] == 1 ) 
		   return false;
	}
	return true;
}

