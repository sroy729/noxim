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

#ifndef __NOXIMGLOBALSENSOR_TABLE_H__
#define __NOXIMGLOBALSENSOR_TABLE_H__

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "NoximMain.h"
using namespace std;

// Structure used to store information into the table
struct NoximSensor {
    int    location;			// ID of the location for sensor placement
	double  rector_line_coef;    // coefficient for temperature recover via linear combination
};

class NoximGlobalSensorTable {

  public:

    NoximGlobalSensorTable();

    // Load sensor table from file. Returns true if ok, false otherwise
    bool load(const char *fname);

  
     vector < NoximSensor > sensor_table;
};

class NoximGlobalRecoverCoefTable {

  public:

    NoximGlobalRecoverCoefTable();

    // Load coefficient table from file. Returns true if ok, false otherwise
    bool load(const char *fname);

  
     vector < NoximSensor > coef_table;
};

#endif
