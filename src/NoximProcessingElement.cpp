/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the processing element
 */

#include "NoximProcessingElement.h"
extern int throttling[10][10][4];
extern int reShot[10][10][4]; //Jimmy added on 2017.05.29

int NoximProcessingElement::randInt(int min, int max)
{
    return min +
	(int) ((double) (max - min + 1) * rand() / (RAND_MAX + 1.0));
}

void NoximProcessingElement::rxProcess(){
    if (reset.read()) {
		_round_MC = 0;
		ack_rx.write(1);
		ack_semi_rx.write(1);
		_adaptive_transmit              =0;
		_dor_transmit                   =0;
		_dw_transmit                    =0;
		_mid_adaptive_transmit          =0;
		_mid_dor_transmit               =0;
		_mid_dw_transmit                =0;
		_beltway_transmit               =0;
		_Transient_adaptive_transmit    =0; 
		_Transient_dor_transmit         =0; 
		_Transient_dw_transmit          =0; 
		_Transient_mid_adaptive_transmit=0; 
		_Transient_mid_dor_transmit     =0;
		_Transient_mid_dw_transmit      =0;
		_Transient_beltway_transmit 	=0;
		_Transient_transmit_tmp         =0;     //Jimmy added on 2017.05.30
		_full_pkt                       =false; //Jimmy added on 2017.05.30
		refly_pkt                       =0;
		while( !flit_queue.empty() ) 
			flit_queue.pop();
    }
	else if((getCurrentCycleNum() % (int) (TEMP_REPORT_PERIOD))==0){//////////////
		ack_rx.write(1);
		ack_semi_rx.write(1);
	}
	else{
		if( req_rx.read()==1 && ack_rx.read() == 1 ){
			NoximFlit flit_tmp = flit_rx.read();
			NoximFlit flit_tmp1; //Jimmy added on 2017.06.01
			if (NoximGlobalParams::verbose_mode > VERBOSE_OFF && local_id == 17) {
				cout << sc_simulation_time() << ": ProcessingElement[" <<
				local_id << "] RECEIVING " << flit_tmp << endl;
			}
			
			/*** Jimmy added on 2017.05.30***/
			if(flit_tmp.flit_type == FLIT_TYPE_HEAD){
				if(_full_pkt){
					flit_src = id2Coord(flit_tmp1.src_id);
					reShot[flit_src.x][flit_src.y][flit_src.z] += 1;
					_full_pkt = false;
					_Transient_transmit_tmp = 0;
				}
				flit_tmp1 = flit_tmp;
				_full_pkt = true;
				_Transient_transmit_tmp ++;
			}
			else if(flit_tmp.flit_type == FLIT_TYPE_BODY){_Transient_transmit_tmp ++;}
			else{ 
				_Transient_transmit_tmp ++;
				//if(_Transient_transmit_tmp!=8) cout<<"local_id = "<<local_id<<" ;_Transient_transmit_tmp = "<<_Transient_transmit_tmp<<" ;_full_pkt = "<<_full_pkt<<endl;
				//assert(_Transient_transmit_tmp == 8);
				//if(_full_pkt && (flit_tmp.sequence_no==(_Transient_transmit_tmp-1))){
				if(_full_pkt){
					if(_Transient_transmit_tmp!=8) cout<<"local_id = "<<local_id<<" ;_Transient_transmit_tmp = "<<_Transient_transmit_tmp<<" ;_full_pkt = "<<_full_pkt<<endl;
					assert(_Transient_transmit_tmp == 8);
					_flit_static(flit_tmp, _Transient_transmit_tmp);
				}
				else{
					flit_src = id2Coord(flit_tmp.src_id);
					reShot[flit_src.x][flit_src.y][flit_src.z] += 1;
				}
				_full_pkt = false;
				_Transient_transmit_tmp = 0;
			}
			/******/
			//_flit_static(flit_tmp);//statics
		}
		if( req_semi_rx.read()==1 && ack_semi_rx.read() == 1 ){
			NoximFlit flit = flit_semi_rx.read(); //Jimmy modified on 2011.10.25
			if(flit.flit_type == FLIT_TYPE_TAIL) refly_pkt++;
			if (NoximGlobalParams::verbose_mode > VERBOSE_OFF && local_id == 17) {
				cout << sc_simulation_time() << ": ProcessingElement[" <<
					local_id << "] RECEIVING " << flit << endl;
			}
			flit_queue.push(flit); //Jimmy modified on 2011.05.29
		}
		ack_rx.write( !_emergency );
		ack_semi_rx.write( !_emergency );
    }
}

void NoximProcessingElement::txProcess()
{
	
    if (reset.read()){
		req_tx.write(0);
		req_semi_tx.write(0);
		not_transmit      = 0;
		transmit          = 0;
		while( !packet_queue.empty() ) 
			packet_queue.pop();
		transmittedAtPreviousCycle = false;
		_clean_all          = false;
		packet_queue_length = 0; // Jimmy added on 2012.04.24
		drop_pkt            = 0; //Jimmy added on 2012.04.24
		inject_pkt          = 0; //Jimmy added on 2012.04.26
		reshot_pkt          = 0; //Jimmy added on 2017.05.30
		if ( NoximGlobalParams::traffic_distribution == TRAFFIC_RANDOM) {
			/*if (
			
				local_id == xyz2Id(1,1,0) || 
				local_id == xyz2Id(1,2,0) ||
				local_id == xyz2Id(2,1,0) ||
				local_id == xyz2Id(2,2,0) ||
				local_id == xyz2Id(5,5,0) ||
				local_id == xyz2Id(5,6,0) ||
				local_id == xyz2Id(6,5,0) ||
				local_id == xyz2Id(6,6,0) ||
				local_id == xyz2Id(1,1,1) ||
				local_id == xyz2Id(1,2,1) ||
				local_id == xyz2Id(2,1,1) ||
				local_id == xyz2Id(2,2,1) ||
			    local_id == xyz2Id(5,5,1) ||
				local_id == xyz2Id(5,6,1) ||
				local_id == xyz2Id(6,5,1) ||
				local_id == xyz2Id(6,6,1) ||
				local_id == xyz2Id(1,1,2) ||
				local_id == xyz2Id(1,2,2) ||
				local_id == xyz2Id(2,1,2) ||
				local_id == xyz2Id(2,2,2) ||
			    local_id == xyz2Id(5,5,2) ||
				local_id == xyz2Id(5,6,2) ||
				local_id == xyz2Id(6,5,2) ||
				local_id == xyz2Id(6,6,2)
				
				/*local_id == xyz2Id(3,3,0) ||
				local_id == xyz2Id(4,4,0) ||
				local_id == xyz2Id(3,3,1) ||
				local_id == xyz2Id(4,4,1) ||
				local_id == xyz2Id(3,3,2) ||
				local_id == xyz2Id(4,4,2)*/ 
			//)
				//t_quota = 80000;
			//else
				t_quota = NoximGlobalParams::static_quota;
		}
		/* other mode */
		else{
			t_quota = NoximGlobalParams::static_quota;
		}
    }
	else if((getCurrentCycleNum() % (int) (TEMP_REPORT_PERIOD))==0){///////////
		req_tx.write(0);
		req_semi_tx.write(0);
		//while( !packet_queue.empty() ) 
		//	packet_queue.pop();
		//_clean_all          = false;
	}		
	else{
		NoximPacket packet;
		NoximFlit   flit;
		
		/*if((!_emergency && !_clean_all && num_ni_pkt <= NoximGlobalParams::ni_queue_depth) && 
		   (t_quota > 0) && (NoximGlobalParams::static_quota > TQUOTA_OFF)){
			if (canShot(packet) && (packet.dst_id != NOT_VALID) ){
				//cout<<"Enter Pkt Generation"<<endl;
				//assert(packet.src_id < 1000);
				//assert(packet.dst_id < 1000);
				inject_pkt ++;
				if(packet.routing!=INVALID_ROUTING){
					packet_queue.push(packet);
					transmittedAtPreviousCycle = true;
					num_ni_pkt ++;
				}
				else drop_pkt ++; //要再加上一個counter以計算有多少packet被丟掉，以用來模擬message level的事情
			} 
			else
				transmittedAtPreviousCycle = false;
		}
		if(_emergency && canShot(packet)){
			inject_pkt ++;
			drop_pkt ++;
		}*/
		
		// hfsken-120629 
		//產生packet 並推送到message queue
		/*if(!_emergency && !_clean_all ){
			if ((NoximGlobalParams::static_quota > TQUOTA_OFF) && (t_quota > 0)){ //Each PE generate a fixed numbers of pkts
				if ( canShot(packet) ){
				message_queue.push(packet);
				transmittedAtPreviousCycle = true;
				inject_pkt ++; //Jimmy move this statement to the pkt queue (2017.05.29)
                                t_quota --; //Jimmy move this statement to the pkt queue (2017.05.29)
					
				} 
				else
					transmittedAtPreviousCycle = false;
			}
			if (NoximGlobalParams::static_quota == TQUOTA_OFF){ // Each PE can keep generating pkts
				if ( canShot(packet) ){
				message_queue.push(packet);
				transmittedAtPreviousCycle = true;
				inject_pkt ++; //Jimmy move this statement to the pkt queue (2017.05.29)
				} 
				else
					transmittedAtPreviousCycle = false;
			}
		}
		
		//判斷message queue是否可以送到packet queue
		if (packet_queue_length < DEFAULT_PACKET_QUEUE_LENGTH && !message_queue.empty() && !_clean_all){
			packet = message_queue.front();
			if(NoximGlobalParams::tla_support){ //Does TLA schemem support? (Jimmy added on 2012.07.15)
				if ( TLA(packet)){//The routing > 10 all have downward routing. 
				                  //The downward routing is not allowed, if the routing algo. doesn't belong to TLAR scheme, such as XYZ
					if( NoximGlobalParams::routing_algorithm > 10 || packet.routing != ROUTING_DOWNWARD_CROSS_LAYER){
						TAAR(packet);
						packet.timestamp_ni = getCurrentCycleNum();
						//cout<<"packet.timestamp_ni ="<<packet.timestamp_ni<<endl;
						packet_queue.push( packet );
						message_queue.pop();
						packet_queue_length++;
						//inject_pkt ++;
						//t_quota --;
					}
					else drop_pkt ++; //For the pkts using non-TLA routing, they should be dropped, if they want to use DW
				}
				else drop_pkt ++; //The pkt should be dropped in INVALID_ROUTING
			}
			else{//No TLA support. Therefore, every packet will enter to NI's pkt queue, if there is any free slot. (Jimmy added on 2012.07.15)
				TAAR(packet);
				packet.timestamp_ni = getCurrentCycleNum();
				//cout<<"packet.timestamp_ni ="<<packet.timestamp_ni<<endl;
				packet_queue.push( packet );
				message_queue.pop();
				packet_queue_length++;
				//inject_pkt ++;
				//t_quota --;
			}
				
		}*/
		
		/*************************/
		if (packet_queue_length < DEFAULT_PACKET_QUEUE_LENGTH && !_clean_all){
			if ((NoximGlobalParams::static_quota > TQUOTA_OFF) && (t_quota > 0)){
				if ( canShot(packet) && (local_id <NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_x) ){ ////Andy add on 2016.4.26
					if(NoximGlobalParams::tla_support){ //Does TLA schemem support? (Jimmy added on 2012.07.15)
						if ( TLA(packet)){//The routing > 10 all have downward routing. 
				                  //The downward routing is not allowed, if the routing algo. doesn't belong to TLAR scheme, such as XYZ
							if( NoximGlobalParams::routing_algorithm > 10 || packet.routing != ROUTING_DOWNWARD_CROSS_LAYER){
								TAAR(packet);
								packet.timestamp_ni = getCurrentCycleNum();
								//cout<<"packet.timestamp_ni ="<<packet.timestamp_ni<<endl;
								packet_queue.push( packet );
								packet_queue_length++;
								//inject_pkt ++;
								t_quota --;
							}
							else drop_pkt ++; //For the pkts using non-TLA routing, they should be dropped, if they want to use DW
						}
						else drop_pkt ++; //The pkt should be dropped in INVALID_ROUTING
					}
					else{//No TLA support. Therefore, every packet will enter to NI's pkt queue, if there is any free slot. (Jimmy added on 2012.07.15)
						TAAR(packet);
						packet.timestamp_ni = getCurrentCycleNum();
						//cout<<"packet.timestamp_ni ="<<packet.timestamp_ni<<endl;
						packet_queue.push( packet );
						packet_queue_length++;
						//inject_pkt ++;
						t_quota --;
					}
				}
			}
			if (NoximGlobalParams::static_quota == TQUOTA_OFF){
				if ( canShot(packet) && (local_id < NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_x) ){ //Andy add on 2016.4.26
					if(NoximGlobalParams::tla_support){ //Does TLA schemem support? (Jimmy added on 2012.07.15)
						if ( TLA(packet)){//The routing > 10 all have downward routing. 
				                  //The downward routing is not allowed, if the routing algo. doesn't belong to TLAR scheme, such as XYZ
							
							if( NoximGlobalParams::routing_algorithm > 10 || packet.routing != ROUTING_DOWNWARD_CROSS_LAYER){
								TAAR(packet);
								packet.timestamp_ni = getCurrentCycleNum();
								//cout<<"packet.timestamp_ni ="<<packet.timestamp_ni<<endl;
								packet_queue.push( packet );
								packet_queue_length++;
								//inject_pkt ++;
								//t_quota --;
							}
							else drop_pkt ++; //For the pkts using non-TLA routing, they should be dropped, if they want to use DW
						}
						else drop_pkt ++; //The pkt should be dropped in INVALID_ROUTING
					}
					else{//No TLA support. Therefore, every packet will enter to NI's pkt queue, if there is any free slot. (Jimmy added on 2012.07.15)
						TAAR(packet);
						packet.timestamp_ni = getCurrentCycleNum();
						//cout<<"packet.timestamp_ni ="<<packet.timestamp_ni<<endl;
						packet_queue.push( packet );
						packet_queue_length++;
						//inject_pkt ++;
						//t_quota --;
					}
				}
			}
		}
		/*************************/
		
		if( _clean_all ){
			if ( !flit_queue.empty() ){
				flit = flit_queue.front();
				if(flit.flit_type==FLIT_TYPE_HEAD &&  refly_pkt > 0){
					while( refly_pkt > 0 && !NoximGlobalParams::message_level){
						if (flit.flit_type==FLIT_TYPE_TAIL)
							refly_pkt--;
						flit_queue.pop();
						if( flit_queue.empty() )
							break;
						else
							flit = flit_queue.front();
					}
				}
			}
		}
		/*Send Flit*/
		if(ack_tx.read() == 1){
			req_tx.write(0);
			if (!packet_queue.empty()){
				flit = nextFlit();	// Generate a new flit
				if (NoximGlobalParams::verbose_mode > VERBOSE_OFF && local_id == 17) {
					cout << getCurrentCycleNum() << ": ProcessingElement[" << local_id <<
					"] SENDING " << flit << endl;
				}
				flit_tx->write(flit);	// Send the generated flit
				req_tx.write(1);
				if (flit.flit_type==FLIT_TYPE_HEAD) 
					cnt_local++;
				/*if (flit.flit_type==FLIT_TYPE_TAIL){ //Jimmy added on 2017.05.30
					inject_pkt ++;
					t_quota --;
				}*/ 
			}	
		}
		//////////////////////////////////////////
		if( ack_semi_tx.read() == 1){
			req_semi_tx.write(0);
			if( !flit_queue.empty() ){
				if( refly_pkt > 0 ){
					flit = flit_queue.front();
					NoximPacket p;
					assert( flit.mid_id < MAX_ID + 1);
					assert( flit.dst_id < MAX_ID + 1);
					p.src_id = flit.mid_id;
					p.dst_id = flit.dst_id;
					while(!TLA(p)){
						
						if( flit.flit_type == FLIT_TYPE_HEAD ){
							NoximCoord flit_dst = id2Coord(flit.dst_id);
							if ( throttling[flit_dst.x][flit_dst.y][flit_dst.z] )cout<< getCurrentCycleNum() << ":Packet drop.(dst)\t";
							else cout<< getCurrentCycleNum() << ":"<<flit<<" drop."<<endl;
						}
						flit_queue.pop();
						if ( flit.flit_type == FLIT_TYPE_TAIL) refly_pkt--;
						if ( refly_pkt == 0)break;
						flit = flit_queue.front();
						p.src_id = flit.mid_id;
						p.dst_id = flit.dst_id;
					}
				}
				if( refly_pkt > 0 ){
					flit = flit_queue.front();
					NoximPacket p;
					assert( flit.mid_id < MAX_ID + 1);
					assert( flit.dst_id < MAX_ID + 1);
					p.src_id = flit.mid_id;
					p.dst_id = flit.dst_id;
					if( !TLA(p)){
						cout<<getCurrentCycleNum()<<":"<<flit<<endl;
						cout<<"flit.current_id"<<"="<<local_id       <<id2Coord(local_id   )<<endl; 
						cout<<"flit.src_id    "<<"="<<flit.src_id    <<id2Coord(flit.src_id)<<endl;
						cout<<"flit.mid_id    "<<"="<<flit.mid_id    <<id2Coord(flit.mid_id)<<endl; 		
						cout<<"flit.dst_id    "<<"="<<flit.dst_id    <<id2Coord(flit.dst_id)<<endl; 
						cout<<"flit.routing   "<<"="<<flit.routing_f <<endl; 
						cout<<"flit.DW_layer  "<<"="<<flit.DW_layer  <<endl; 
						cout<<"flit.arr_mid   "<<"="<<flit.arr_mid   <<endl; 
						assert(0);
					}
					//
					flit.arr_mid   = true;
					//
					if( flit.beltway && NoximGlobalParams::beltway)
						flit.routing_f = ROUTING_XYZ;
					else 
						flit.routing_f = p.routing;
					flit_semi_tx->write(flit);
					req_semi_tx ->write(1);
					flit_queue.pop();
					if ( flit.flit_type == FLIT_TYPE_TAIL) refly_pkt--;
					if ( NoximGlobalParams::verbose_mode > VERBOSE_OFF && local_id == 17) {
						cout << "TEST" <<getCurrentCycleNum() << ": ProcessingElement[" << local_id <<
						"] SENDING " << flit << endl;
					}
				}
			}
		}
    }
}

NoximFlit NoximProcessingElement::nextFlit()
{
    NoximFlit flit;
    NoximPacket packet = packet_queue.front();
 
    flit.src_id      = packet.src_id;
    flit.dst_id      = packet.dst_id;
	flit.mid_id      = packet.mid_id;
    flit.timestamp   = packet.timestamp;
	flit.timestamp_ni= packet.timestamp_ni;
	flit.timestamp_nw= getCurrentCycleNum();
    flit.sequence_no = packet.size - packet.flit_left;
    flit.hop_no      = 0;
	flit.routing_f   = packet.routing;
	flit.DW_layer    = packet.DW_layer;
	flit.arr_mid     = packet.arr_mid;
	flit.beltway     = packet.beltway;
	flit.tampered    = packet.tampered;
    //  flit.payload     = DEFAULT_PAYLOAD;
    if (packet.size == packet.flit_left)
		flit.flit_type = FLIT_TYPE_HEAD;
    else if (packet.flit_left == 1)
		flit.flit_type = FLIT_TYPE_TAIL;
    else
		flit.flit_type = FLIT_TYPE_BODY;

    packet_queue.front().flit_left--;
    if (packet_queue.front().flit_left == 0){
		packet_queue.pop();
		packet_queue_length --; //Jimmy added on 2012.04.24
		//t_quota --; //Jimmt added on 2012.04.24
	}

    return flit;
}

bool NoximProcessingElement::canShot(NoximPacket & packet)
{
    bool shot;
    double threshold;

    if (NoximGlobalParams::traffic_distribution != TRAFFIC_TABLE_BASED){
		if (!transmittedAtPreviousCycle)
			threshold = NoximGlobalParams::packet_injection_rate;
		else
			threshold = NoximGlobalParams::probability_of_retransmission;
		
		//shot = (_emergency_level==1 && _emergency_level!=2)? (((double) rand()) / RAND_MAX < threshold/2) : (((double) rand()) / RAND_MAX < threshold); //Jimmy modified on 2011.11.10
		//shot = (((double) rand()) / RAND_MAX < threshold);
		//cout<<"_emergency_level = "<<_emergency_level<<endl;
		
		/*
		if (local_id == 27 || local_id == 28 || local_id == 35 || local_id == 36)  //added for hotspot
		{
		  if(_emergency_level!=0) shot = (((double) rand()) / RAND_MAX < threshold*2/_emergency_level);
			else shot =(((double) rand()) / RAND_MAX < threshold*2);
			}
		else
		{
		  if(_emergency_level!=0) shot = (((double) rand()) / RAND_MAX < threshold/_emergency_level);
			else shot =(((double) rand()) / RAND_MAX < threshold);
		}
		*/
		
		if(_emergency_level!=0) shot = (((double) rand()) / RAND_MAX < threshold/_emergency_level);//for random
		else shot =(((double) rand()) / RAND_MAX < threshold);
		
		/*if(_emergency_level==2) shot =(((double) rand()) / RAND_MAX < threshold/2);
		else if(_emergency_level==3) shot =(((double) rand()) / RAND_MAX < threshold/3);
		else shot =(((double) rand()) / RAND_MAX < threshold);*/
		//cout<<"shot = "<<shot<<endl;
		if (shot) {
			switch (NoximGlobalParams::traffic_distribution) {
			case TRAFFIC_RANDOM:
			packet = trafficRandom();
			break;
	
			case TRAFFIC_TRANSPOSE1:
			packet = trafficTranspose1();
			break;
	
			case TRAFFIC_TRANSPOSE2:
			packet = trafficTranspose2();
			break;
	
			case TRAFFIC_BIT_REVERSAL:
			packet = trafficBitReversal();
			break;
	
			case TRAFFIC_SHUFFLE:
			packet = trafficShuffle();
			break;
	
			case TRAFFIC_BUTTERFLY:
			packet = trafficButterfly();
			break;
	
			default:
			assert(false);
			}
		}
    }
	else 
	{			// Table based communication traffic
		if (never_transmit)
			return false;

		int now = getCurrentCycleNum() ;
		bool use_pir = (transmittedAtPreviousCycle == false);
		vector < pair < int, double > > dst_prob;
		double threshold =
			traffic_table->getCumulativePirPor(local_id, now,
							   use_pir, dst_prob);

		double prob = (double) rand() / RAND_MAX;
		shot = (prob < threshold);
		if (shot) 
		{
			for (unsigned int i = 0; i < dst_prob.size(); i++) {
			if (prob < dst_prob[i].second) {
				packet.make(local_id, dst_prob[i].first, now, getRandomSize());
				//assert( TLA(packet) );
				if ( NoximGlobalParams::beltway ){
					if( _non_beltway_layer > id2Coord(local_id).z ){
						if( NoximGlobalParams::Mbeltway )
							packet.mid_id = sel_int_node_Mbelt(packet.src_id,packet.dst_id,packet.beltway);
						else
							packet.mid_id = sel_int_node_belt(packet.src_id,packet.dst_id,packet.beltway);
					}
					else
						packet.mid_id = sel_int_node(packet.src_id,packet.dst_id);
					if( (id2Coord(packet.mid_id).x == id2Coord(packet.dst_id).x) && (id2Coord(packet.mid_id).y == id2Coord(packet.dst_id).y) ){
						packet.arr_mid = true;
						packet.mid_id  = packet.src_id;				    
					}
					else{
						packet.arr_mid = false;
						//If the packet is going to the corner, force them to route XYZ
						int s_z = id2Coord(packet.src_id).z;
						if( packet.beltway ){
							packet.routing = ROUTING_XYZ;
						}
						else{
							packet.routing = ROUTING_WEST_FIRST;
						}
					}
				}
				else if ( packet.routing == ROUTING_DOWNWARD_CROSS_LAYER && NoximGlobalParams::cascade_node ){
					assert( packet.src_id < MAX_ID + 1);
					assert( packet.dst_id < MAX_ID + 1);
					if( NoximGlobalParams::Mcascade )
						packet.mid_id    = sel_int_node_Mcascade(packet.src_id,packet.dst_id);
					else
						packet.mid_id    = sel_int_node(packet.src_id,packet.dst_id);
					if(packet.mid_id == packet.src_id)packet.arr_mid = true;
					else {
						packet.arr_mid   = false;
						packet.routing   = ROUTING_WEST_FIRST;
					}
				}
				else{
					packet.arr_mid = true;
					packet.mid_id  = packet.src_id;
				}	
				break;
			}
			}
		}
    }
	
    return shot;
}


NoximPacket NoximProcessingElement::trafficRandom(){
    // int max_id = MAX_ID;//x*y*z - 1     Andy remove on 2016.4.26
    int max_id = MAX_ID; //Andy add on 2016.4.26
	NoximPacket p;
    p.src_id = local_id;
    double rnd = rand() / (double) RAND_MAX;
    double range_start = 0.0;
    //cout << "\n " << getCurrentCycleNum() << " PE " << local_id << " rnd = " << rnd << endl;
	int re_transmit = 1; //1

	//if(_emergency_level == 1) cout<<"Enter Random Traffic"<<endl;
	//if(_emergency_level == 1) cout<<"Source ID = "<<p.src_id<<endl;
    // Random destination distribution
    do {
		transmit++;
		//p.dst_id = randInt(0, max_id);//Tong remove on 2018.2.05
		p.dst_id = randInt(0, max_id);//Tong add on 2018.2.05
		//assert( p.dst_id < MAX_ID + 1 );//Tong remove on 2018.2.05
		assert( p.dst_id < max_id + 1 );//Tong add on 2018.2.05
		// check for hotspot destination
		for (unsigned int i = 0; i < NoximGlobalParams::hotspots.size(); i++) {
	    //cout << getCurrentCycleNum() << " PE " << local_id << " Checking node " << NoximGlobalParams::hotspots[i].first << " with P = " << NoximGlobalParams::hotspots[i].second << endl;
	    if (rnd >= range_start && rnd <	range_start + NoximGlobalParams::hotspots[i].second) {
			//if (local_id != NoximGlobalParams::hotspots[i].first ) { //Tong remove on 2018.2.05
			  if (local_id != NoximGlobalParams::hotspots[i].first && local_id<NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_x ) {//Tong add on 2018.2.05
				//cout << getCurrentCycleNum() << " PE " << local_id <<" That is ! " << endl;
				p.dst_id = NoximGlobalParams::hotspots[i].first;
			}	
			break;
	    } 
		else
			//if (local_id<64){//Tong add on 2018.2.05
				range_start += NoximGlobalParams::hotspots[i].second;	// try next
			//}//Tong add on 2018.2.05
		}
		if (p.dst_id == p.src_id)
			re_transmit = 1;
		else{
			re_transmit = !TLA(p);
			TAAR(p);
		}
    } while ((p.dst_id == p.src_id) /*|| (re_transmit)*/);
	//if(_emergency_level == 1) cout<<"Out Random Traffic"<<endl;

	//assert( p.routing >= 0 );
    return p;
}

NoximPacket NoximProcessingElement::trafficTranspose1()
{
   NoximPacket p;
    p.src_id = local_id;
    NoximCoord src, dst;

    // Transpose 1 destination distribution
    src   = id2Coord(p.src_id);
	dst.x = NoximGlobalParams::mesh_dim_x - 1 - src.y;
    dst.y = NoximGlobalParams::mesh_dim_y - 1 - src.x;
 //   dst.z = NoximGlobalParams::mesh_dim_z - 1 - src.z; Andy remove on 2016.4.26
  dst.z = 0; //Andy add on 2016.4.26
	fixRanges(src, dst);
    p.dst_id = coord2Id(dst);

    p.timestamp = getCurrentCycleNum() ;
    p.size = p.flit_left = getRandomSize();
	/*** Cross Layer Solution***/
	//if(!TLA(p)) p.dst_id = NOT_VALID;
	p.arr_mid = true;
	p.mid_id  = p.src_id;
    return p;
}

NoximPacket NoximProcessingElement::trafficTranspose2()
{
    NoximPacket p;
    p.src_id = local_id;
    NoximCoord src, dst;

    // Transpose 2 destination distribution
    src   = id2Coord(p.src_id);
    dst.x = src.y;
    dst.y = src.x;
	dst.z = src.z;
    fixRanges(src, dst);
    p.dst_id = coord2Id(dst);

    p.timestamp = getCurrentCycleNum();
    p.size = p.flit_left = getRandomSize();
	/*** Cross Layer Solution***/
	//if(!TLA(p)) p.dst_id = NOT_VALID;
	p.arr_mid = true;
	p.mid_id  = p.src_id;
    return p;
}

void NoximProcessingElement::setBit(int &x, int w, int v)
{
    int mask = 1 << w;

    if (v == 1)
	x = x | mask;
    else if (v == 0)
	x = x & ~mask;
    else
	assert(false);
}

int NoximProcessingElement::getBit(int x, int w)
{
    return (x >> w) & 1;
}

inline double NoximProcessingElement::log2ceil(double x)
{
    return ceil(log(x) / log(2.0));
}

NoximPacket NoximProcessingElement::trafficBitReversal()
{
    
    int nbits = (int)log2ceil(
		(double)(NoximGlobalParams::mesh_dim_x *
                 NoximGlobalParams::mesh_dim_y //*  Tong remove on 2018.2.03
				 //NoximGlobalParams::mesh_dim_z    Tong remove on 2018.2.03
				 ));
    int dnode = 0;
    for (int i = 0; i < nbits; i++)
	//setBit(dnode, i, getBit(local_id, nbits - i - 1)); Tong remove on 2018.2.05
	setBit(dnode, i, getBit(local_id%64, nbits - i - 1)); //Tong add on 2018.2.05

    NoximPacket p;
    p.src_id = local_id;
    p.dst_id = dnode;

    p.timestamp = getCurrentCycleNum();
    p.size      = p.flit_left = getRandomSize();
	p.arr_mid = true;
	p.mid_id  = p.src_id;
    return p;
}

NoximPacket NoximProcessingElement::trafficShuffle()
{
    int nbits = (int)log2ceil(
		(double)(NoximGlobalParams::mesh_dim_x *
                 NoximGlobalParams::mesh_dim_y //*   Tong remove on 2018.2.03
				 //NoximGlobalParams::mesh_dim_z     Tong remove on 2018.2.03
				 ));
    int dnode = 0;
    for   (int i = 0; i < nbits - 1; i++)
	//setBit(dnode, i + 1, getBit(local_id, i        ));  Tong remove on 2018.2.05
    //setBit(dnode, 0    , getBit(local_id, nbits - 1));  Tong remove on 2018.2.05
	setBit(dnode, i + 1, getBit(local_id%64, i        )); //Tong add on 2018.2.05
	setBit(dnode, 0    , getBit(local_id%64, nbits - 1)); //Tong add on 2018.2.05

    NoximPacket p;
    p.src_id = local_id;
    p.dst_id = dnode   ;

    p.timestamp = getCurrentCycleNum();
    p.size      = p.flit_left = getRandomSize();
	p.arr_mid = true;
	p.mid_id  = p.src_id;
	
    return p;
}

NoximPacket NoximProcessingElement::trafficButterfly()
{
    int nbits = (int)log2ceil(
		(double)(NoximGlobalParams::mesh_dim_x *
                 NoximGlobalParams::mesh_dim_y //*    Tong remove on 2018.2.03
				 //NoximGlobalParams::mesh_dim_z      Tong remove on 2018.2.03
				 ));
    int dnode = 0;
    for   (int i = 1; i < nbits - 1; i++)
	//setBit(dnode, i        , getBit(local_id, i        )); Tong remove on 2018.2.05
    //setBit(dnode, 0        , getBit(local_id, nbits - 1)); Tong remove on 2018.2.05
    //setBit(dnode, nbits - 1, getBit(local_id, 0        )); Tong remove on 2018.2.05
	setBit(dnode, i        , getBit(local_id%64, i        )); //Tong add on 2018.2.05
	setBit(dnode, 0        , getBit(local_id%64, nbits - 1)); //Tong add on 2018.2.05
	setBit(dnode, nbits - 1, getBit(local_id%64, 0        )); //Tong add on 2018.2.05
	

    NoximPacket p;
    p.src_id = local_id;
    p.dst_id = dnode  ;

    p.timestamp = getCurrentCycleNum() ;
    p.size      = p.flit_left = getRandomSize();
	p.arr_mid = true;
	p.mid_id  = p.src_id;
    return p;
}

void NoximProcessingElement::fixRanges(const NoximCoord src,
				       NoximCoord & dst)
{
    // Fix ranges
    if (dst.x < 0)
		dst.x = 0;
    if (dst.y < 0)
		dst.y = 0;
	if (dst.z < 0)
		dst.z = 0;
    if (dst.x >= NoximGlobalParams::mesh_dim_x)
		dst.x = NoximGlobalParams::mesh_dim_x - 1;
    if (dst.y >= NoximGlobalParams::mesh_dim_y)
		dst.y = NoximGlobalParams::mesh_dim_y - 1;
	if (dst.z >= NoximGlobalParams::mesh_dim_z)
		dst.z = NoximGlobalParams::mesh_dim_z - 1;
}

int NoximProcessingElement::getRandomSize()
{
    return randInt(NoximGlobalParams::min_packet_size,
		   NoximGlobalParams::max_packet_size);
}

void NoximProcessingElement::TraffThrottlingProcess(){
    /*if( reset.read() ) _clean_all = false;
	else{
	    if(NoximGlobalParams::throt_type == THROT_VERTICAL){	
			if( getCurrentCycleNum()%(int)TEMP_REPORT_PERIOD > TEMP_REPORT_PERIOD - NoximGlobalParams::clean_stage_time )//clean-up stage
			     _clean_all = true ;
			else _clean_all = false;
		}
	}*/
}

void NoximProcessingElement::IntoEmergency(){
	_emergency  = true;
	_emergency_level = 1;
}

void NoximProcessingElement::IntoSemiEmergency(int emergency_level){ //Jimmy added on 2012.04.12
	_emergency  = false;
	//_emergency_level = 1;
	_emergency_level = emergency_level;
}

void NoximProcessingElement::OutOfEmergency(){ //Jimmy added on 2012.04.12
	_emergency       = false;
	_emergency_level = 0;
}

void NoximProcessingElement::IntoCleanStage(){
	_clean_all       = true;
}
void NoximProcessingElement::OutOfCleanStage(){
	NoximCoord curr = id2Coord( local_id ); //Jimmy added on 2017.05.30
	NoximFlit f;
	
	while(packet_queue_length!=0){
		packet_queue.pop();
		packet_queue_length --;
		//t_quota ++;
		reShot[curr.x][curr.y][curr.z] +=1;
	}   
	
	while( !flit_queue.empty() ){
		f = flit_queue.front();
		if(f.flit_type == FLIT_TYPE_HEAD){
			flit_src = id2Coord(f.src_id);
			reShot[flit_src.x][flit_src.y][flit_src.z] += 1;
		}
		flit_queue.pop();
	} 
			
	
	//req_tx.write(0);
	//req_semi_tx.write(0);
	
	t_quota += reShot[curr.x][curr.y][curr.z]; //Jimmy added on 2017.05.30
	reshot_pkt += reShot[curr.x][curr.y][curr.z]; //Jimmy added on 2017.05.30
	reShot[curr.x][curr.y][curr.z] = 0; //Jimmy added on 2017.05.30
	_clean_all       = false;
	
	//if(t_quota!=0 || ack_tx.read()!=1) cout<<local_id<<": "<<t_quota<<"( "<<ack_tx.read()<<" )\t";
	
}

void NoximProcessingElement::RTM_set_var(int non_throt_layer, int non_beltway_layer, int RoC_col_max, int RoC_col_min, int RoC_row_max, int RoC_row_min){
    _non_throt_layer   = non_throt_layer  ;
    _non_beltway_layer = non_beltway_layer;
    _RoC_col_min       = RoC_col_min      ;
    _RoC_col_max       = RoC_col_max      ;
    _RoC_row_min       = RoC_row_min      ;
    _RoC_row_max       = RoC_row_max      ;
}

bool NoximProcessingElement::TLA( NoximPacket & packet )
{

 NoximCoord curr = id2Coord( packet.src_id );
 NoximCoord dest = id2Coord( packet.dst_id );

 int x_diff = dest.x - curr.x;
 int y_diff = dest.y - curr.y;
 int z_diff = dest.z - curr.z;

 int n_x, n_y, n_z,y_a,x_a,z_a;
 int a_x_search, a_y_search, a_z_search;
                    
 bool adaptive_fail         = false;
 bool xy_fail               = false;
 bool dw_fail_src,dw_fail_dest;
 int  routing               = ROUTING_XYZ;

    if(throttling[dest.x][dest.y][dest.z] != 1){//destination not throttle (Jimmy modified on 2012.04.25 ==0 --> !=1)
	    //XY-Plane
		for( y_a = 0 ; y_a < abs(y_diff) + 1 ; y_a ++ )
		for( x_a = 0 ; x_a < abs(x_diff) + 1 ; x_a ++ ){
		    a_x_search = (x_diff > 0)?(curr.x + x_a):(curr.x - x_a);
			a_y_search = (y_diff > 0)?(curr.y + y_a):(curr.y - y_a);
			if(throttling[a_x_search][a_y_search][curr.z]==1) adaptive_fail = true;
			//adaptive_fail |= throttling[a_x_search][a_y_search][curr.z];
		}
		//Z-direction
        for( z_a = 1 ; z_a < abs(z_diff) + 1 ; z_a++ ){
			a_z_search = (z_diff>0)?(curr.z + z_a):(curr.z - z_a);
			if(throttling[dest.x][dest.y][a_z_search]==1) adaptive_fail = true;
			//adaptive_fail |= throttling[dest.x][dest.y][a_z_search];
		}

	 /***Into XY Routing***/
	if( adaptive_fail == true ){
        //X-direction
		for( x_a = 1 ; x_a < abs(x_diff) + 1 ; x_a++ ){
			a_x_search = (x_diff>0)?(curr.x + x_a):(curr.x - x_a);
			if(throttling[a_x_search][curr.y][curr.z]==1) xy_fail = true;
			//xy_fail |= throttling[a_x_search][curr.y][curr.z];
		}
		//Y-direction
        for( y_a = 1 ; y_a < abs(y_diff) + 1 ; y_a++ ){
			a_y_search = (y_diff > 0)?(curr.y + y_a):(curr.y - y_a);
			if(throttling[dest.x][a_y_search][curr.z]==1) xy_fail = true;
			//xy_fail |= throttling[dest.x][a_y_search][curr.z];
		}
		//Z-direction
        for( z_a=1;z_a<abs(z_diff)+1;z_a++){
			a_z_search = (z_diff>0)?(curr.z + z_a):(curr.z - z_a);
			if(throttling[dest.x][dest.y][a_z_search]==1) xy_fail = true;
			//xy_fail |= throttling[dest.x][dest.y][a_z_search]; 
		}
	}
    /***Into Downward Routing***/
	if(xy_fail == true){
		dw_fail_src = false;
	    int z_diff_dw_s = (NoximGlobalParams::mesh_dim_z-1) - curr.z;
	    for( int zzt = 1 ; zzt < z_diff_dw_s + 1 ; zzt++ ){
			if(throttling[curr.x][curr.y][curr.z + zzt]==1) dw_fail_src = true;
			//dw_fail_src |= throttling[curr.x][curr.y][curr.z + zzt];
			if ( throttling[curr.x][curr.y][curr.z + zzt]==1 ){
				//cout<<getCurrentCycleNum()<<":"<<curr<<"->"<<dest<<endl;
				//cout<<"throttling[curr.x][curr.y][curr.z + zzt] is true, zzt = "<<zzt<<endl;
			}
		}
		dw_fail_dest = false;
		int z_diff_dw_d = (NoximGlobalParams::mesh_dim_z-1) - dest.z;
	    for( int zzt = 1 ; zzt < z_diff_dw_d + 1 ; zzt++ ){
			if(throttling[dest.x][dest.y][dest.z + zzt]==1) dw_fail_src = true;
			//dw_fail_dest |= throttling[dest.x][dest.y][dest.z + zzt];
			if ( throttling[dest.x][dest.y][dest.z + zzt]==1 ){
				//cout<<getCurrentCycleNum()<<":"<<curr<<"->"<<dest<<endl;
				//cout<<"throttling[dest.x][dest.y][dest.z + zzt] is true, zzt = "<<zzt<<endl;
			}
		}
	}

	//if(_emergency_level) cout<<"Enter TLA fun"<<endl;/////////////////////////
	if     ( adaptive_fail                 == false ) packet.routing = ROUTING_WEST_FIRST;
	else if(      xy_fail                  == false ) packet.routing = ROUTING_XYZ;
	else if( (dw_fail_dest || dw_fail_src) == false ) packet.routing = ROUTING_DOWNWARD_CROSS_LAYER; //Chihhao's TLAR scheme has this function
	else {  
		packet.routing = INVALID_ROUTING;
			//if(_emergency_level) cout<<"Enter INVALID_ROUTING"<<endl;/////////////////////////
		return false;
	}

	//packet.routing   = routing;
	packet.timestamp = getCurrentCycleNum() ;
    packet.size      = packet.flit_left = getRandomSize();
	if((NoximGlobalParams::mesh_dim_z - _non_throt_layer)!=0)
		packet.DW_layer  = _non_throt_layer + rand() % ( NoximGlobalParams::mesh_dim_z - _non_throt_layer);
	else
		packet.DW_layer = -1;
	
	//if(_emergency_level) cout<<"Out TLA fun"<<endl;///////////////////////////

    //return TAAR(packet);
		return true;
	}
	else{//destination throttled
		not_transmit++;
		//if(_emergency_level) cout<<"Enter Dst Throttled"<<endl;/////////////////////////
		packet.routing = INVALID_ROUTING;
		return false;
		//return true;
	}	
}

bool NoximProcessingElement::TAAR( NoximPacket & p ){
	
	//if(_emergency_level) cout<<"Enter TAAR fun"<<endl; ////////////////
	
	if ( NoximGlobalParams::beltway && ( _non_beltway_layer > id2Coord(local_id).z) )
		Beltway(p);
	else if ( p.routing == ROUTING_DOWNWARD_CROSS_LAYER && NoximGlobalParams::cascade_node ){
		assert( p.src_id < MAX_ID + 1);
		assert( p.dst_id < MAX_ID + 1);
		if( NoximGlobalParams::Mcascade )
			p.mid_id    = sel_int_node_Mcascade(p.src_id,p.dst_id);
		else
			p.mid_id    = sel_int_node(p.src_id,p.dst_id);
			
		if(p.mid_id == p.src_id){//if the packet needs to downward directly
			p.arr_mid = true;
		}
		else {
			p.arr_mid   = false;
			p.routing   = ROUTING_WEST_FIRST;
		}
	}
	else{
		p.arr_mid = true;
		p.mid_id  = p.src_id;
	}
	//if(_emergency_level) cout<<"Out TAAR fun"<<endl; /////////////////////////

	return true;
}

bool NoximProcessingElement::Beltway( NoximPacket & p ){
	if( NoximGlobalParams::Mbeltway )
		p.mid_id = sel_int_node_Mbelt(p.src_id,p.dst_id,p.beltway);
	else
		p.mid_id = sel_int_node_belt(p.src_id,p.dst_id,p.beltway);
	
	if( (id2Coord(p.mid_id).x == id2Coord(p.dst_id).x) && (id2Coord(p.mid_id).y == id2Coord(p.dst_id).y) ){//packet send directly
		p.arr_mid = true;
		p.mid_id  = p.src_id;			    
	}
	else if ( (id2Coord(p.mid_id).x == id2Coord(p.src_id).x) && (id2Coord(p.mid_id).y == id2Coord(p.src_id).y) ){//
		p.arr_mid = true;
	}
	else{
		p.arr_mid = false;
		if( p.beltway ){
			p.routing = ROUTING_XYZ;
		}
		else{
			p.routing = ROUTING_WEST_FIRST;
		}
	}
	return true;
}

/*Select the intemedium node (Jimmy modified on 2011.07.01)*/
int NoximProcessingElement::sel_int_node(int source, int  destination) 
{
	assert(source      < MAX_ID + 1 );
	assert(destination < MAX_ID + 1 );
	NoximCoord s = id2Coord(source     ); //transfer source id to coordination
	NoximCoord d = id2Coord(destination); //transfer destination id to coordination
	NoximCoord int_node; //intermedium node
	if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<getCurrentCycleNum()<<":sel_int_node-"<<s<<","<<d<<endl;
	int lateral_bound; //find the lateral searching bound
	int vertical_bound; //find the vertical searching bound
	int sel_candidate; //the area of intermedium node region
	int candidate_tmp;
	
	if(d.x > s.x && d.y > s.y)
	{
		if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"In case d.x > s.x & d.y > s.y"<<endl;
		vertical_bound = d.y;
		sel_candidate = 0;
		for(int x=s.x; x<=d.x; x++)
		{
			if(throttling[x][s.y][s.z]==1)
			{
				lateral_bound = x - 1;
				break;
			}
			for(int y=s.y; y<=vertical_bound; y++)
			{
				if(throttling[x][y][s.z]==1 || y==vertical_bound)
				{
					candidate_tmp = (y==vertical_bound && throttling[x][y][s.z]==0) ? (x - s.x +1) * (y - s.y + 1) : (x - s.x +1) * ((y-1) - s.y + 1);
					if(candidate_tmp >= sel_candidate)
					{
						vertical_bound = (y == s.y) ? s.y : (y == vertical_bound && throttling[x][y][s.z]==0) ? y : (y-1);
						lateral_bound = (x == s.x) ? s.x :  x; //always is x-1, because souce node cannot be throttled in this phase
						sel_candidate = candidate_tmp;
					}
					break;
				}
			}
		}//end outer-for
		int_node.x = lateral_bound;
		int_node.y = vertical_bound;
		int_node.z = s.z;

		//if(int_node.x == d.x) int_node.y = d.y; //XY routable
		//////////////////////////////////////////////////////////////////////
		int id = int_node.z*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y + (int_node.y * NoximGlobalParams::mesh_dim_x) + int_node.x;
		if(id > NoximGlobalParams::mesh_dim_x * NoximGlobalParams::mesh_dim_y * NoximGlobalParams::mesh_dim_z){
		cout<<"Enter the int_node1 computation"<<endl;
		cout<<"int_node ="<<id<<endl;
		cout<<"int_node.z ="<<int_node.z<<endl;
		cout<<"int_node.y ="<<int_node.y<<endl;
		cout<<"int_node.x ="<<int_node.x<<endl;
		}
		return coord2Id(int_node);
	}
	else if(d.x > s.x && d.y <= s.y)
	{
		if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"In case d.x > s.x & d.y < s.y"<<endl;
		vertical_bound = d.y;
		sel_candidate = 0;
		for(int x=s.x; x<=d.x; x++)
		{
			if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
			cout<<"In for loop of x="<<x<<endl;
			if(throttling[x][s.y][s.z]==1){
				lateral_bound = x - 1;
				break;
			}
			for(int y=s.y; y>=vertical_bound; y--)
			{
				if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
				cout<<"In for loop of y="<<y<<endl;
				if(throttling[x][y][s.z]==1 || y==vertical_bound)
				{
					candidate_tmp = (y == vertical_bound && throttling[x][y][s.z]==0) ? (x - s.x +1) * (s.y - y + 1) : (x - s.x +1) * (s.y - (y+1) + 1);
					if(candidate_tmp >=sel_candidate)
					{
						vertical_bound = (y == s.y) ? s.y : (y == vertical_bound && throttling[x][y][s.z]==0) ? y : (y+1);
						lateral_bound  = (x == s.x) ? s.x :  x; //always is x-1, because souce node cannot be throttled in this phase
						sel_candidate  = candidate_tmp;
					}
					break;
				}
			}
		}//end outer-for
		int_node.x = lateral_bound;
		int_node.y = vertical_bound;
		int_node.z = s.z;
		
		//////////////////////////////////////////////////////////////////////
		int id = int_node.z*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y + (int_node.y * NoximGlobalParams::mesh_dim_x) + int_node.x;
		if(id > NoximGlobalParams::mesh_dim_x * NoximGlobalParams::mesh_dim_y * NoximGlobalParams::mesh_dim_z){
		cout<<"Enter the int_node2 computation"<<endl;
		cout<<"int_node ="<<id<<endl;
		cout<<"int_node.z ="<<int_node.z<<endl;
		cout<<"int_node.y ="<<int_node.y<<endl;
		cout<<"int_node.x ="<<int_node.x<<endl;
		}
		
		//if(int_node.x == d.x) int_node.y = d.y; //XY routable
		return coord2Id(int_node);
	}
	else if(d.x <= s.x && d.y > s.y)
	{	if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"In case d.x < s.x & d.y > s.y"<<endl;
		vertical_bound = d.y;
		sel_candidate = 0;
		for(int x=s.x; x>=d.x; x--)
		{
			if(throttling[x][s.y][s.z]==1)
			{
				lateral_bound = x + 1;
				break;
			}
			for(int y=s.y; y<=vertical_bound; y++)
			{
				if(throttling[x][y][s.z]==1 || y==vertical_bound)
				{
					candidate_tmp = (y==vertical_bound && throttling[x][y][s.z]==0) ? (s.x - x +1) * (y -s. y + 1) : (s.x - x +1) * ((y-1) -s. y + 1);
					if(candidate_tmp >= sel_candidate)
					{
						vertical_bound = (y == s.y) ? s.y : (y == vertical_bound && throttling[x][y][s.z]==0) ? y : (y-1);
						lateral_bound = (x == s.x) ? s.x : x; //always is x+1, because souce node cannot be throttled in this phase
						sel_candidate = candidate_tmp;
					}
					break;
				}
			}
		}//end outer-for
		int_node.x = lateral_bound;
		int_node.y = vertical_bound;
		int_node.z = s.z;
		
		//////////////////////////////////////////////////////////////////////
		int id = int_node.z*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y + (int_node.y * NoximGlobalParams::mesh_dim_x) + int_node.x;
		if(id > NoximGlobalParams::mesh_dim_x * NoximGlobalParams::mesh_dim_y * NoximGlobalParams::mesh_dim_z){
		cout<<"Enter the int_node3 computation"<<endl;
		cout<<"int_node ="<<id<<endl;
		cout<<"int_node.z ="<<int_node.z<<endl;
		cout<<"int_node.y ="<<int_node.y<<endl;
		cout<<"int_node.x ="<<int_node.x<<endl;
		}
		
		//if(int_node.x == d.x) int_node.y = d.y; //XY routable
		return coord2Id(int_node);
	}
	else if(d.x <= s.x && d.y <= s.y)
	{	if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"In case d.x < s.x & d.y < s.y"<<endl;
		vertical_bound = d.y;
		sel_candidate = 0;
		for(int x=s.x; x>=d.x; x--)
		{
			if(throttling[x][s.y][s.z]==1)
			{
				lateral_bound = x + 1;
				break;
			}
			for(int y=s.y; y>=vertical_bound; y--)
			{
				if(throttling[x][y][s.z]==1 || y==vertical_bound)
				{
					candidate_tmp = (y==vertical_bound && throttling[x][y][s.z]==0) ? (s.x - x +1) * (s.y - y + 1) : (s.x - x +1) * (s.y - (y+1) + 1);
					if(candidate_tmp >= sel_candidate)
					{
						vertical_bound = (y == s.y) ? s.y : (y == vertical_bound && throttling[x][y][s.z]==0) ? y : (y+1);
						lateral_bound = (x == s.x) ? s.x : x; //always is x+1, because souce node cannot be throttled in this phase
						sel_candidate = candidate_tmp;
					}
					break;
				}
			}
		}//end outer-for
		int_node.x = lateral_bound;
		int_node.y = vertical_bound;
		int_node.z = s.z;
		
		//////////////////////////////////////////////////////////////////////
		int id = int_node.z*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y + (int_node.y * NoximGlobalParams::mesh_dim_x) + int_node.x;
		if(id > NoximGlobalParams::mesh_dim_x * NoximGlobalParams::mesh_dim_y * NoximGlobalParams::mesh_dim_z){
		cout<<"Enter the int_node4 computation"<<endl;
		cout<<"int_node ="<<id<<endl;
		cout<<"int_node.z ="<<int_node.z<<endl;
		cout<<"int_node.y ="<<int_node.y<<endl;
		cout<<"int_node.x ="<<int_node.x<<endl;
		}
		
		//if(int_node.x == d.x) int_node.y = d.y; //XY routable
		return coord2Id(int_node);
	}
	else{
		if (NoximGlobalParams::verbose_mode > VERBOSE_LOW ) 
		cout<<"Mid node return destination"<<endl;
		return destination; //if(d.x == s.x & d.y > s.y),  if(d.x == s.x & d.y < s.y),  if(d.x > s.x & d.y == s.y),  if(d.x < s.x & d.y == s.y)
	}
}
int NoximProcessingElement::sel_int_node_belt(int source, int  destination, bool &beltway){
	NoximCoord s = id2Coord(source     ); //transfer source id to coordination
	NoximCoord d = id2Coord(destination); //transfer destination id to coordination
	NoximCoord m = id2Coord(sel_int_node(source,destination)); //intermedium node
	int mid,i;
	int min_hop = 2*(abs(s.x - d.x) + abs(s.y - d.y));
	int beltway_hop;
	float threshold = NoximGlobalParams::beltway_ratio;
	if( s.x > d.x && !inROC(s,d) ){
		if( ((double) rand()) / RAND_MAX > threshold ){
			return sel_int_node( source, destination);
		}
		else{
			if( d.y > 4 ){
				mid = xyz2Id( NoximGlobalParams::mesh_dim_x - 1, NoximGlobalParams::mesh_dim_y - 1, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = NoximGlobalParams::mesh_dim_x - 1 - s.x + NoximGlobalParams::mesh_dim_y - 1 - s.y + NoximGlobalParams::mesh_dim_x - 1 - d.x + NoximGlobalParams::mesh_dim_y - 1 - d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
			else{
				mid = xyz2Id( NoximGlobalParams::mesh_dim_x - 1, 0, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = NoximGlobalParams::mesh_dim_x - 1 - s.x + s.y + NoximGlobalParams::mesh_dim_x - 1 - d.x + d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
		}
	}
	else if ( s.x < d.x && !inROC(s,d) ){
		if( ((double) rand()) / RAND_MAX > threshold ){
			return sel_int_node( source, destination);
		}
		else{
			if( d.y > 4 ){
				mid = xyz2Id( 0, NoximGlobalParams::mesh_dim_y - 1, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = s.x + NoximGlobalParams::mesh_dim_y - 1 - s.y + d.x + NoximGlobalParams::mesh_dim_y - 1 - d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
			else{
				mid = xyz2Id( 0, 0, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = s.x + s.y + d.x + d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
		}
	}
	else
		return sel_int_node( source, destination);
}
int NoximProcessingElement::sel_int_node_Mbelt(int source, int  destination, bool &beltway){
	NoximCoord s = id2Coord(source     ); //transfer source id to coordination
	NoximCoord d = id2Coord(destination); //transfer destination id to coordination
	NoximCoord m = id2Coord(sel_int_node(source,destination)); //intermedium node
	int ring_level = inRing(d);
	int mid,i;
	int min_hop = 2*(abs(s.x - d.x) + abs(s.y - d.y));
	int beltway_hop;
	float threshold = NoximGlobalParams::beltway_ratio;
	if( s.x > d.x && !inROC(s,d) ){
		if( ((double) rand()) / RAND_MAX > threshold ){
			return sel_int_node( source, destination);
		}
		else{
			if( d.y > 4 ){
				mid = xyz2Id( NoximGlobalParams::mesh_dim_x - 1 - ring_level, NoximGlobalParams::mesh_dim_y - 1 - ring_level, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = NoximGlobalParams::mesh_dim_x - 1 - s.x + NoximGlobalParams::mesh_dim_y - 1 - s.y + NoximGlobalParams::mesh_dim_x - 1 - d.x + NoximGlobalParams::mesh_dim_y - 1 - d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
			else{
				mid = xyz2Id( NoximGlobalParams::mesh_dim_x - 1 - ring_level, ring_level, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = NoximGlobalParams::mesh_dim_x - 1 - s.x + s.y + NoximGlobalParams::mesh_dim_x - 1 - d.x + d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
		}
	}
	else if ( s.x < d.x && !inROC(s,d) ){
		if( ((double) rand()) / RAND_MAX > threshold ){
			return sel_int_node( source, destination);
		}
		else{
			if( d.y > 4 ){
				mid = xyz2Id( ring_level, NoximGlobalParams::mesh_dim_y - 1 - ring_level, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = s.x + NoximGlobalParams::mesh_dim_y - 1 - s.y + d.x + NoximGlobalParams::mesh_dim_y - 1 - d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
			else{
				mid = xyz2Id( ring_level, ring_level, s.z );
				NoximPacket p( source, mid, 0, 2);
				beltway_hop = s.x + s.y + d.x + d.y;
				if( TLA(p) && ( p.routing == ROUTING_WEST_FIRST || p.routing == ROUTING_XYZ ) && ( beltway_hop < min_hop )){
					beltway = true;
					return mid;
					}
				else
					return sel_int_node( source, destination);
			}
		}
	}
	else
		return sel_int_node( source, destination);
}
int NoximProcessingElement::sel_int_node_Mcascade(int source, int  destination) {
	
	assert(source      < MAX_ID + 1 );
	assert(destination < MAX_ID + 1 );
	NoximCoord s = id2Coord(source     ); //transfer source id to coordination
	NoximCoord d = id2Coord(destination); //transfer destination id to coordination
	NoximCoord m = id2Coord(sel_int_node(source,destination)); //intermediate node
	if( m == s || m == d )
		return coord2Id(m);
	int i,j,k;
	
	if( m.x != s.x){
		if      ( abs(m.x - s.x) > 2 && abs(m.y - s.y) > 2 && NoximGlobalParams::Mcascade_step > 2){
			if( ( _round_MC % 4 ) == 0 ){
				if      ( m.x > s.x ) i = m.x - 3;
				else if ( m.x < s.x ) i = m.x + 3;
				else                  i = m.x    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_x) )
					m.x = i;
				}
			else if( ( _round_MC % 4 ) == 1 ){
				if      ( m.y > s.y ) i = m.y - 3;
				else if ( m.y < s.y ) i = m.y + 3;
				else                  i = m.y    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_y) )
					m.y = i;
			}
			else if ( ( _round_MC % 4 ) == 2 ){
				if      ( m.y > s.y ) i = m.y - 1;
				else if ( m.y < s.y ) i = m.y + 1;
				else                  i = m.y    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_y) )
					m.y = i;
				if      ( m.x > s.x ) i = m.x - 2;
				else if ( m.x < s.x ) i = m.x + 2;
				else                  i = m.x    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_x) )
					m.x = i;
			}
			else {
				if      ( m.y > s.y ) i = m.y - 2;
				else if ( m.y < s.y ) i = m.y + 2;
				else                  i = m.y    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_y) )
					m.y = i;
				if      ( m.x > s.x ) i = m.x - 1;
				else if ( m.x < s.x ) i = m.x + 1;
				else                  i = m.x    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_x) )
					m.x = i;
			}
		}
		else if      ( abs(m.x - s.x) > 1 && abs(m.y - s.y) > 1 && NoximGlobalParams::Mcascade_step > 1){
			if( ( _round_MC % 3 ) == 0 ){
				if      ( m.x > s.x ) i = m.x - 2;
				else if ( m.x < s.x ) i = m.x + 2;
				else                  i = m.x    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_x) )
					m.x = i;
				}
			else if( ( _round_MC % 3 ) == 1 ){
				if      ( m.y > s.y ) i = m.y - 2;
				else if ( m.y < s.y ) i = m.y + 2;
				else                  i = m.y    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_y) )
					m.y = i;
			}
			else{
				if      ( m.y > s.y ) i = m.y - 1;
				else if ( m.y < s.y ) i = m.y + 1;
				else                  i = m.y    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_y) )
					m.y = i;
				if      ( m.x > s.x ) i = m.x - 1;
				else if ( m.x < s.x ) i = m.x + 1;
				else                  i = m.x    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_x) )
					m.x = i;
			}
		}
		else if ( abs(m.x - s.x) > 0 && abs(m.y - s.y) > 0  && NoximGlobalParams::Mcascade_step > 0){
			// if( ( rand() % 2 ) == 0 ){
			if( ( _round_MC % 2 ) == 0 ){
				if      ( m.x > s.x ) i = m.x - 1;
				else if ( m.x < s.x ) i = m.x + 1;
				else                  i = m.x    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_x) )
					m.x = i;
			}
			else{
				if      ( m.y > s.y ) i = m.y - 1;
				else if ( m.y < s.y ) i = m.y + 1;
				else                  i = m.y    ;
				if( (i > -1) && ( i < NoximGlobalParams::mesh_dim_y) )
					m.y = i;
			}
		}
		else if ( NoximGlobalParams::Mcascade_step == 0){
			//////////////////////////////////////////////////////////////////////
		int id = m.z*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y + (m.y * NoximGlobalParams::mesh_dim_x) + m.x;
		if(id > NoximGlobalParams::mesh_dim_x * NoximGlobalParams::mesh_dim_y * NoximGlobalParams::mesh_dim_z){
		cout<<"Enter the m1 computation"<<endl;
		cout<<"m ="<<id<<endl;
		cout<<"m.z ="<<m.z<<endl;
		cout<<"m.y ="<<m.y<<endl;
		cout<<"m.x ="<<m.x<<endl;
		}
			return coord2Id(m);
			
		}
	}
	_round_MC++;
	//////////////////////////////////////////////////////////////////////
		int id = m.z*NoximGlobalParams::mesh_dim_x*NoximGlobalParams::mesh_dim_y + (m.y * NoximGlobalParams::mesh_dim_x) + m.x;
		if(id > NoximGlobalParams::mesh_dim_x * NoximGlobalParams::mesh_dim_y * NoximGlobalParams::mesh_dim_z){
		cout<<"Enter the m2 computation"<<endl;
		cout<<"m ="<<id<<endl;
		cout<<"m.z ="<<m.z<<endl;
		cout<<"m.y ="<<m.y<<endl;
		cout<<"m.x ="<<m.x<<endl;
		}
	return coord2Id(m);
}

bool NoximProcessingElement::inROC(NoximCoord s,NoximCoord d){
	/*if(  ((d.x < _RoC_col_min) && ( s.x < _RoC_col_min)) ||
	     ((d.x > _RoC_col_max) && ( s.x > _RoC_col_max)) ||
		 ((d.x < _RoC_col_min) && ( s.x < _RoC_col_min)) ||
		 ((d.y > _RoC_row_max) && ( s.x > _RoC_row_max)) 	)
		return true;
	else if( inROC_S(d) )
    	return true;
	else
		return false;*/
	return inROC_S(d);
}

bool NoximProcessingElement::inROC_S(NoximCoord s){
	if( s.x >= _RoC_col_min && s.x <= _RoC_col_max && 
	    s.y >= _RoC_row_min && s.y <= _RoC_row_max )
	   	return true;
	else
		return false;
}

int NoximProcessingElement::inRing(NoximCoord dest){//Find the ring level of destination
	//Find Min. of a,b,c,d
	if( NoximGlobalParams::Sbeltway && NoximGlobalParams::Sbeltway_ring > -1 ){
		return NoximGlobalParams::Sbeltway_ring;
	}
	else if( NoximGlobalParams::Sbeltway && NoximGlobalParams::Sbeltway_ring == -1){
		int a = _RoC_col_min;
		int b = NoximGlobalParams::mesh_dim_x - _RoC_col_max;
		int c = _RoC_row_min;
		int d = NoximGlobalParams::mesh_dim_y - _RoC_row_max;
		int e = ( a < b )?a:b;
		int f = ( c < d )?c:d;
		int g = ( e < f )?e:f;
		g--;
		if ( g < 0 ) g = 0;
		return g;
	}
	else{
	int a = dest.x;
	int b = NoximGlobalParams::mesh_dim_x - dest.x;
	int c = dest.y;
	int d = NoximGlobalParams::mesh_dim_y - dest.y;
	int e = ( a < b )?a:b;
	int f = ( c < d )?c:d;
	int g = ( e < f )?e:f;
	return g;
	}
	
}

/*void NoximProcessingElement::_flit_static(NoximFlit flit_tmp){
	_Transient_total_transmit++;
	_total_transmit++;
	if( flit_tmp.src_id == flit_tmp.mid_id ){//not cascade flit
		if     (flit_tmp.routing_f == ROUTING_WEST_FIRST          ){
			_adaptive_transmit++;
			_Transient_adaptive_transmit++;
		}
		else if(flit_tmp.routing_f == ROUTING_XYZ                 ){
			_dor_transmit       ++;	
			_Transient_dor_transmit++;	
		}
		else if(flit_tmp.routing_f == ROUTING_DOWNWARD_CROSS_LAYER){
			_dw_transmit        ++;	
			_Transient_dw_transmit++;
		}
	}
	else{//cascade flit
		if( flit_tmp.beltway == true){
			_Transient_beltway_transmit++;
			_beltway_transmit++;
		}
		else if(flit_tmp.routing_f == ROUTING_WEST_FIRST          ){
			_mid_adaptive_transmit++;
			_Transient_mid_adaptive_transmit++;
		}
		else if(flit_tmp.routing_f == ROUTING_XYZ                 ){
			_mid_dor_transmit     ++;
			_Transient_mid_dor_transmit     ++;
		}
		else if(flit_tmp.routing_f == ROUTING_DOWNWARD_CROSS_LAYER){
			_mid_dw_transmit++;	
			_Transient_mid_dw_transmit++;
		}
	}
}*/

void NoximProcessingElement::_flit_static(NoximFlit flit_tmp, int _receive_flit){
	_Transient_total_transmit+=_receive_flit;
	_total_transmit+=_receive_flit;
	if( flit_tmp.src_id == flit_tmp.mid_id ){//not cascade flit
		if     (flit_tmp.routing_f == ROUTING_WEST_FIRST          ){
			_adaptive_transmit+=_receive_flit;
			_Transient_adaptive_transmit+=_receive_flit;
		}
		else if(flit_tmp.routing_f == ROUTING_XYZ                 ){
			_dor_transmit       +=_receive_flit;	
			_Transient_dor_transmit+=_receive_flit;	
		}
		else if(flit_tmp.routing_f == ROUTING_DOWNWARD_CROSS_LAYER){
			_dw_transmit        +=_receive_flit;	
			_Transient_dw_transmit+=_receive_flit;
		}
	}
	else{//cascade flit
		if( flit_tmp.beltway == true){
			_Transient_beltway_transmit+=_receive_flit;
			_beltway_transmit+=_receive_flit;
		}
		else if(flit_tmp.routing_f == ROUTING_WEST_FIRST          ){
			_mid_adaptive_transmit+=_receive_flit;
			_Transient_mid_adaptive_transmit+=_receive_flit;
		}
		else if(flit_tmp.routing_f == ROUTING_XYZ                 ){
			_mid_dor_transmit     +=_receive_flit;
			_Transient_mid_dor_transmit     +=_receive_flit;
		}
		else if(flit_tmp.routing_f == ROUTING_DOWNWARD_CROSS_LAYER){
			_mid_dw_transmit+=_receive_flit;	
			_Transient_mid_dw_transmit+=_receive_flit;
		}
	}
}

void NoximProcessingElement::CalcDelay( vector< int > &pe_packet_delay ){
	_packet_queue_num   = 0;
	_packet_queue_msg_delay = 0;
	_packet_queue_ni_delay  = 0;
	while( !packet_queue.empty() ){
		NoximPacket packet = packet_queue.front();
		if( packet.flit_left == packet.size ){//only calc packets
			_packet_queue_num       ++;
			_packet_queue_msg_delay +=  (int)getCurrentCycleNum() - (int)packet.timestamp    ;
			_packet_queue_ni_delay  +=  (int)getCurrentCycleNum() - (int)packet.timestamp_ni ;
			pe_packet_delay.push_back( ( (int)getCurrentCycleNum() - (int)packet.timestamp )  );
		}
		packet_queue.pop();
	}
}

void NoximProcessingElement::CalcMessageDelay( vector< int > &pe_msg_delay ){
	_message_queue_num   = 0;
	_message_queue_delay = 0;
	while( !message_queue.empty() ){
		NoximPacket packet = message_queue.front();
		_message_queue_num   ++;
		_message_queue_delay += ( (int)getCurrentCycleNum() - (int)packet.timestamp );
		pe_msg_delay.push_back( (int)getCurrentCycleNum() - (int)packet.timestamp );
		message_queue.pop(); 
	}
}

void NoximProcessingElement::ResetTransient_Transmit(){
	_Transient_adaptive_transmit    =0;
	_Transient_dor_transmit         =0;
	_Transient_dw_transmit          =0;
	_Transient_mid_adaptive_transmit=0;
	_Transient_mid_dor_transmit     =0;
	_Transient_mid_dw_transmit      =0;
	_Transient_beltway_transmit     =0;
}
