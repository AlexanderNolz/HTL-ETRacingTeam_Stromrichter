//
// Created by alexn on 11.09.2023.
//

#ifndef REGELUNGSTECHNIK_CLARK_TRANSFORMATION_H
#define REGELUNGSTECHNIK_CLARK_TRANSFORMATION_H
#include "math.h"

typedef struct clark{
    float A,B,C,alpha,beta;
}clark;

typedef struct park{
	float d,q,alpha,beta,theta;
}park;

#define park_default {	\
		.d=0,				\
		.q=0,				\
		.alpha=0,			\
		.beta=0,			\
		.theta=0			\
};

void init_clark(clark *data);
void recalc_clark(clark *data, float A,float B,float C);
void recalc_park(park *data, float d, float q, float theta);

#endif //REGELUNGSTECHNIK_CLARK_TRANSFORMATION_H
