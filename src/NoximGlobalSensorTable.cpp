/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the definition of the global thermal sensor table
 * This file is not the original file in Noxim (Jimmy create on 2016.03.19)
 */

#include "NoximGlobalSensorTable.h"

NoximGlobalSensorTable::NoximGlobalSensorTable()
{	
}

NoximGlobalRecoverCoefTable::NoximGlobalRecoverCoefTable()
{	
}

bool NoximGlobalSensorTable::load(const char *fname) // load the sensor placement table, which record the node id with sensor
{
    // Open file
    ifstream fin(fname, ios::in);
    if (!fin)
	return false;

    // Initialize variables
    sensor_table.clear();
	
    // Cycle reading file
    while (!fin.eof()) {
		char line[512];
		fin.getline(line, sizeof(line) - 1);
		
		if (line[0] != '\n' || line[0] != '\0') {
			int location_id;
			NoximSensor sensor_location;
			
			sscanf(line, "%d", &location_id);
			
			sensor_location.location = location_id;
			sensor_table.push_back(sensor_location);
		}

    }

    return true;
}

bool NoximGlobalRecoverCoefTable::load(const char *fname) // load the coefficient table, which is used to recover the temperature via linear combination
{
	// Open file
    ifstream fin(fname, ios::in);
    if (!fin)
	return false;

    // Initialize variables
    coef_table.clear();
	
    // Cycle reading file
    while (!fin.eof()) {
		char line[512];
		fin.getline(line, sizeof(line) - 1);
		
		if (line[0] != '\n' || line[0] != '\0') {
			double coef = atof(line);
			NoximSensor rector_linear;
			
			//sscanf(line, "%f", &coef);
			//printf("%d", coef);
			rector_linear.rector_line_coef = coef;
			coef_table.push_back(rector_linear);
		}

    }
	/*for(int i=0;i<coef_table.size();i++){
		printf("%.4f\t", coef_table[i].rector_line_coef);
		if(i%4==0) printf("\n");
	} */
    return true;
}
