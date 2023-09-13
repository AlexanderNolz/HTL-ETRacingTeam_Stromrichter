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
    data->alpha = _2_3 * (cos0*A+cos120*B+cos240*C);
    data->beta = _2_3 *(sin0*A+sin120*B+sin240*C);
}