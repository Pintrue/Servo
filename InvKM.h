#ifndef INVKM_H
#define INVKM_H

typedef struct _threeDOFsInv {
	double l1, l2, l3;	// lengths of the three links
	double baseHeight;
	/* four angles between links initially, where a2 is fixed */
	double initA1, initA2, initA3, initA4;
} threeDOFsInv;


int initInvKM();

#endif