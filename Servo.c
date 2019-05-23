#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <pigpio.h>

#include "model.h"
#include "model_utils.h"
#include "matrix_op.h"
#include "FwdKM.h"
#include <math.h>

#define NUM_OF_SERVO 3
#define BASE_SERVO_GPIO_PIN 4
#define SHOULDER_SERVO_GPIO_PIN 17
#define FOREARM_SERVO_GPIO_PIN 27
#define MAGNET_GPIO_PIN 26

#define BASE_RANGE M_PI
#define SHOULDER_RANGE ((120.0 / 180.0) * M_PI)
#define FOREARM_RANGE (M_PI / 2.0)

#define BASE_MID 76
#define BASE_RIGHT 116
#define BASE_LEFT 32
#define SHOULDER_LEFT 83
#define SHOULDER_RIGHT 26
#define FOREARM_RIGHT 76
#define FOREARM_LEFT 32


int run = 1;

int used[3] = {BASE_SERVO_GPIO_PIN, SHOULDER_SERVO_GPIO_PIN, FOREARM_SERVO_GPIO_PIN};
double jntAngles[3];
double goal[3];


void init() {
	if (gpioInitialise() < 0) {
		exit(0);
	}
	
	/* Initialize the three servo signals */
	for (int i = 0; i < NUM_OF_SERVO; ++i) {
		int pin = used[i];
		gpioSetPWMfrequency(pin, 50);
		gpioSetPWMrange(pin, 1000);

		printf("Range for %d is %d\n", pin, gpioGetPWMrange(pin));
		printf("Frequency for %d is %d\n", pin, gpioGetPWMfrequency(pin));
	}

	for (int i = 0; i < 3; ++i) {
		jntAngles[i] = 0;
	}

	gpioSetMode(MAGNET_GPIO_PIN, PI_OUTPUT);
	// gpioPWM(MAGNET_GPIO_PIN, gpioGetPWMrange(MAGNET_GPIO_PIN) / 2);
	gpioPWM(MAGNET_GPIO_PIN, 140);
	printf("Range for %d is %d\n", MAGNET_GPIO_PIN, gpioGetPWMrange(MAGNET_GPIO_PIN));
}


matrix_t* initObs(double target[3]) {
	matrix_t* obs = new_matrix(1, 9);
	double* data = obs->data;

	for (int i = 0; i < 3; ++i) {
		data[i] = jntAngles[i];
	}

	double eePos[6];
	initFwdKM();
	int res = getEEPoseByJnts(jntAngles, eePos);

	if (res < 0) {
		printf("1. Joint angle moved out of bounds.\n");
		exit(0);
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 3] = eePos[i];
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 6] = target[i];
		goal[i] = target[i];
	}

	return obs;
}


void convertAnglesToDC(int dutyCycles[3], double shoulderDelta) {
	/* BASE SERVO */
	double m0 = (BASE_RIGHT - BASE_LEFT) / BASE_RANGE;
	double b0 = BASE_MID;

	dutyCycles[0] = (int) roundf(m0 * jntAngles[0] + b0);
	printf("Base DC is %d\n", dutyCycles[0]);


	double m1 = (SHOULDER_RIGHT - SHOULDER_LEFT) / SHOULDER_RANGE;
	double b1 = SHOULDER_LEFT;

	dutyCycles[1] = (int) roundf(m1 * jntAngles[1] + b1);
	printf("Shoulder DC is %d\n", dutyCycles[1]);


	double m2 = (FOREARM_RIGHT - FOREARM_LEFT) / FOREARM_RANGE;
	double b2 = FOREARM_RIGHT;

	dutyCycles[2] = (int) roundf(m2 * jntAngles[2] + b2);
	dutyCycles[2] += (SHOULDER_LEFT - dutyCycles[1]);
	printf("Forearm DC is %d\n", dutyCycles[2]);
}


matrix_t* denormalize_action(matrix_t* action) {
	matrix_t* ret = new_matrix(action->rows, action->cols);
	double upper = 0.0872664626;
	double lower = -0.0872664626;
	double d1 = (double)(action->data[0] + 1) / (double)2 * (upper - lower) + lower;
	double d2 = (double)(action->data[1] + 1) / (double)2 * (upper - lower) + lower;
	double d3 = (double)(action->data[2] + 1) / (double)2 * (upper - lower) + lower;
	ret->data[0] = d1;
	ret->data[1] = d2;
	ret->data[2] = d3;
	return ret;
}


matrix_t* evalStep(matrix_t* action) {
	matrix_t* newObs = new_matrix(1, 9);
	double* data = newObs->data;

	matrix_t* denormedAction = denormalize_action(action);
	print_matrix(denormedAction, 1);
	for (int i = 0; i < 3; ++i) {
		jntAngles[i] += denormedAction->data[i];
	}

	for (int i = 0; i < 3; ++i) {
		data[i] = jntAngles[i];
	}

	double eePos[6];
	initFwdKM();

	int res = getEEPoseByJnts(jntAngles, eePos);


	if (res < 0) {
		print_matrix(action, 1);
		printf("2. Joint angle moved out of bound.\n");
		exit(0);
	}


	int dutyCycles[3];
	convertAnglesToDC(dutyCycles, denormedAction->data[1]);

	for (int i = 0; i < 3; ++i) {
		gpioPWM(used[i], dutyCycles[i]);
		time_sleep(0.5);
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 3] = eePos[i];
	}

	for (int i = 0; i < 3; ++i) {
		data[i + 6] = goal[i];
	}

	return newObs;
}


void stop(int signum) {
	run = 0;
}


int main(int argc, char *argv[]) {
	init();

	while (1) {
		int stop = 0;
		scanf("%d", &stop);
		if (stop == 1) {
			// gpioWrite(MAGNET_GPIO_PIN, 0);
			gpioPWM(MAGNET_GPIO_PIN, 0);
			return 0;
		}
		time_sleep(0.5);
	}
	// double target[3] = { -7.495477e+00,0.000000e+00,1.870172e+01 };
	// matrix_t* obs = initObs(target);
	// print_matrix(obs, 1);

	// model* model = load_model("DDPG_ACTOR_SIM_NORM.model");
	// normalizer* norm = load_normalizer("DDPG_NORM_SIM_NORM.norm");


	// for (int i = 0; i < 50; ++i) {
	// 	print_matrix(obs, 1);
	// 	normalize_obs(norm, obs);
	// 	predict(model, obs);
	// 	obs = evalStep(obs);
	// 	printf("Done %d\n", i);
	// 	time_sleep(0.5);
	// }

	//gpioPWM(FOREARM_SERVO_GPIO_PIN, 60);
	// for (int i = 0; i < 10; ++i) {
	// 	// int base;
	// 	int shoulder;
	// 	// int fore_arm;
	// 	//scanf("base %d\n", &base);
		
	// 	scanf("%d", &shoulder);
		
	// 	//scanf("fore %d\n", &fore_arm);
	// 	// gpioPWM(BASE_SERVO_GPIO_PIN, 88);
	// 	gpioPWM(SHOULDER_SERVO_GPIO_PIN, shoulder);
	// 	time_sleep(0.5);
	// }

	// int base = 76;
	// int shoulder = 83;
	// int forearm = 76;

	// while (forearm > 60) {
	// 	forearm -= 1;
	// 	gpioPWM(FOREARM_SERVO_GPIO_PIN, forearm);
	// 	time_sleep(1);
	// }

	// gpioWrite_Bits_0_31_Clear( (1 << FOREARM_SERVO_GPIO_PIN));

	// gpioPWM(FOREARM_SERVO_GPIO_PIN, 60);
	// time_sleep(1);
	
	// gpioPWM(BASE_SERVO_GPIO_PIN, 88);
	// time_sleep(0.5);

	// while (shoulder > 37) {
	// 	shoulder -= 5;
	// 	forearm += 6;

	// 	gpioPWM(SHOULDER_SERVO_GPIO_PIN, shoulder);
	// 	gpioPWM(FOREARM_SERVO_GPIO_PIN, forearm);
	// 	time_sleep(0.5);
	// }
	// gpioPWM(SHOULDER_SERVO_GPIO_PIN, 37);
	// time_sleep(0.5);


	// if (gpioInitialise() < 0)
	// 	return -1;

	// gpioSetSignalFunc(SIGINT, stop);

	// if (argc == 1) {
	// 	gpioSetPWMfrequency(26, 50);
	// 	gpioSetPWMrange(26, 100);
	// 	// int range = gpioGetPWMdutycycle(26);
	// 	int range = gpioGetPWMrange(26);
	// 	int freq = gpioGetPWMfrequency(26);

	// 	printf("Range is %d\n", range);
	// 	printf("Frequency is %d\n", freq);

	// 	gpioPWM(26, 11);
	// 	time_sleep(0.5);
	// }
	finish();

	gpioTerminate();

	return 0;
}