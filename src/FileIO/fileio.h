/*
 * fileio.h
 *
 *  Created on: 2019. 5. 27.
 *      Author: keti-hajun
 */

#ifndef FILEIO_H_
#define FILEIO_H_

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <stdio.h>

using namespace std;

// Text file data load
void load_data(string file_name, vector< vector<double> > *data);
void load_data(string file_name, vector< vector<float> > *data);

#endif /* FILEIO_H_ */
