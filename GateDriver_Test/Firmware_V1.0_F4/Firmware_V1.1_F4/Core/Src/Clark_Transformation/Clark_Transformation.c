//
// Created by alexn on 11.09.2023.
//

#include "Clark_Transformation.h"
void init_clark(clark *data){
    data->A=0;
    data->B=0;
    data->C=0;
    data->alpha=0;
    data->beta=0;
}

void recalc_clark(clark *data, float A,float B,float C){
    data->A=A;
    data->B=B;
    data->C=C;
    data->alpha = (2.0f/3.0f) * (cosf(0.0f*M_PI/180.0f)*A+cosf(120.0f*M_PI/180.0f)*B+cosf(240.0f*M_PI/180.0f)*C);
    data->beta = (2.0f/3.0f) *(sinf(0.0f*M_PI/180.0f)*A+sinf(120.0f*M_PI/180.0f)*B+sinf(240.0f*M_PI/180.0f)*C);
}


void recalc_park(park *data, float d, float q, float theta){
	data->d=d;
	data->q=q;
	data->theta=theta;
	data->alpha= data->d * cosf(data->theta) - data->q * sinf(data->theta);
	data->beta=data->d * sinf(data->theta) + data->q * cosf(data->theta);
}
