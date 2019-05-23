#ifndef INVKM_H
#define INVKM_H

#include "FwdKM.h"

typedef struct _threeDOFsInv {
	double l1, l2, l3;	// lengths of the three links
	double baseHeight;

	// /* store the coordinates of two fixed-joints */
	// double baseCoord[CART_COORD_DIM];
	// double shoulderCoord[CART_COORD_DIM];

	/* four angles between links initially, where a2 is fixed */
	double initA1, initA2, initA3, initA4;
} threeDOFsInv;


int initInvKM();

int getAllPoseByJnts(const double jntArray[JNT_NUMBER],
						double allPoss[JNT_NUMBER][POSE_FRAME_DIM]);

int getJntsByEEPos(const double eePos[POSE_FRAME_DIM],
						double jntArray[JNT_NUMBER]);

int finishInvKM();

#endif