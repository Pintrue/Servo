#include "FwdKM.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define TO_DECIMAL_PLACE(v, n) (roundf(v * pow(10, n)) / pow(10, n))


static threeDOFs* arm;

threeDOFs* init(double linkLength[JNT_NUMBER], double baseHeight,
			double initJntAngles[JNT_NUMBER]) {
	threeDOFs* arm = (threeDOFs*) malloc(sizeof(threeDOFs));

	arm->l1 = linkLength[0];
	arm->l2 = linkLength[1];
	arm->l3 = linkLength[2];

	arm->baseHeight = baseHeight;

	// for (int i = 0; i < JNT_NUMBER; ++i) {
	// 	arm->jntAngles[i] = initJntAngles[i];
	// }
	arm->a1 = initJntAngles[0];
	arm->a2 = 0.58337;
	arm->a3 = initJntAngles[1];
	arm->a4 = initJntAngles[2];

	return arm;
}


double* getPoseByJnts(double jntArray[JNT_NUMBER]) {
	/**
	 * Calculate y from a side view, where y has the following eqaution
	 * 
	 * 					y = d2 + d3 + d4 + d5
	 **/
	arm->a1 += jntArray[0];
	arm->a3 += jntArray[1];
	arm->a4 += jntArray[2];
	double d2 = arm->baseHeight;
	double d3 = arm->l1 * sin(arm->a2);
	double d4 = arm->l2 * sin(arm->a3);
	double d5 = arm->l3 * sin(arm->a4);

	double y = d2 + d3 + d4 + d5;
	// printf("y is equal to %f \n", y);
	// printf("d5 is equal to %f \n", d5);

	/**
	 * Calculate x and z from a top view, where x has the following eqaution
	 * 
	 * 					z = d1 * cos(a1)
	 * and where z has the following equation
	 * 
	 * 					x = d1 * sin(a1)
	 * and d1 has the following equation
	 * 
	 * 					d1 = d6 - d7 + d8
	 **/
	// double a5 = arm->a3 + arm->a2;

	double d6 = arm->l3 * cos(arm->a4);
	double d7 = arm->l2 * cos(arm->a3);
	double d8 = arm->l1 * cos(arm->a2);

	double d1 = d6 - d7 + d8;
	// printf("d1 is equal to %f \n", d1);
	// printf("d5 is equal to %f \n", d5);
	// printf("d6 is equal to %f \n", d6);
	// printf("d7 is equal to %f \n", d7);
	double z = d1 * cos(arm->a1);
	double x = d1 * sin(arm->a1);
	double* coord = (double*) calloc(CART_COORD_DIM, sizeof(double));
	coord[0] = TO_DECIMAL_PLACE(x, 2); coord[1] = TO_DECIMAL_PLACE(y, 2); coord[2] = TO_DECIMAL_PLACE(z, 2);
	return coord;
}


int main() {
	double linkLength[3] = {5.9908, 10.7575, 18.7299};
	// double initJntAngles[3] = {0.0, 0.18700, 0.18797};
	double initJntAngles[3] = {0.0, atan2(2.0, 10.57), atan2(3.5, 18.4)};
	arm = init(linkLength, 4.20, initJntAngles);

	// printf("%f ", arm->l1);
	// printf("%f ", arm->l2);
	// printf("%f ", arm->l3);
	// printf("\n");
	double delta[3] = {0, 1.2, 0.0};
	double* res = getPoseByJnts(delta);
	printf("[ ");
	for (int i = 0; i < 3; ++i) {
		printf("%f ", res[i]);
	}
	printf("]\n");
	return 0;
}