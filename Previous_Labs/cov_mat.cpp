/*
 * cov_mat.cpp
 *
 *  Created on: 19 Oct 2016
 *      Author: jb914
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <string>

using namespace std;

double mean(vector<double>& values);

int main(){

	cout << "Filename: " << endl;


	ifstream infile;
	infile.open("robot_points.txt");

	if (!infile.is_open()){
		cout << "Error - data file not found!" << endl;
		exit(EXIT_FAILURE);
	}

	vector<double> xVals, yVals;
	double tempx, tempy;

	while(infile >> tempx >> tempy){
		xVals.push_back(tempx);
		yVals.push_back(tempy);
	}

	infile.close();

	double meanX = mean(xVals);
	double meanY = mean(yVals);

	double N = xVals.size();

	//Top-left
	double answer_tl = 0;
	double answer_br = 0;
	int i;
	for(i=0 ; i<N ; i++){


		answer_tl = answer_tl + pow((xVals[i] - meanX), 2);
		answer_br = answer_br + pow((yVals[i] - meanY), 2);
	}
	//Top-right

	double answer_tr = 0;
	double xDiff, yDiff, product;

	for(i=0 ; i<N ; i++){
		xDiff = xVals[i] - meanX;
		yDiff = yVals[i] - meanY;
		product = xDiff * yDiff;

		answer_tr = answer_tr + product;
	}
	answer_tl /= N;
	answer_br /= N;
	answer_tr /= N;

	//Output answer
	cout << "Covarince matrix from " << N << " points..." << endl;

	cout << "Top left: " << answer_tl << endl;
	cout << "Top right: " << answer_tr << endl;
	cout << "Bottom left: " << answer_tr << endl;
	cout << "Bottom right: " << answer_br << endl;

	return 0;
}

double mean(vector<double>& values){
	double sum=0;

	for(vector<double>::iterator i = values.begin() ; i != values.end() ; ++i){
		sum = sum + *(i);
	}

	return (sum/values.size());
}
