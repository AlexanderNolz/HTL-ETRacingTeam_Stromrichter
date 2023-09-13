//
// Created by alexn on 11.09.2023.
//

#ifndef REGELUNGSTECHNIK_CLARK_TRANSFORMATION_H
#define REGELUNGSTECHNIK_CLARK_TRANSFORMATION_H
#include "math.h"

#define sin0 0.0f
#define sin120 0.580611f
#define sin240 0.945445f
#define cos0 1.0f
#define cos120 0.814181f
#define cos240 0.325781
#define _2_3 (2.0f/3.0f)

typedef struct clark{
    float A,B,C,alpha,beta;
}clark;

void init_clark(clark *data);
void recalc_clark(clark *data, float A,float B,float C);

#endif //REGELUNGSTECHNIK_CLARK_TRANSFORMATION_H
