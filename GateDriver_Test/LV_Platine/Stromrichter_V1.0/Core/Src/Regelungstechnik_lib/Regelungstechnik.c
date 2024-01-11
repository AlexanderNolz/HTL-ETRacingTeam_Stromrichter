//
// Created by alexn on 23.06.2023.
//

#include "Regelungstechnik.h"
//funktion zum konfigurieren des Integrators
void config_Integrator(Integrator *data,float K){
    data->K = K;
    data->val = 0;
}
//funktion um einen Weiteren wert zum Integrierer hinzu zufügen
void add_val_Integrator(Integrator *data,float newval,float dt){
    data->val += newval * data->K * dt;
}
//funktion zum konfigurieren eines P Gliedes
void config_P(P *data,float K){
    data->K=K;
    data->val= 0;
}
//funktion um den neuen Wert neu zu berechnen
void recall_P(P *data,float val){
    data->val= val * data->K;
}
//funktion zum konfiguirieren eines PI Gliedes
void config_PID(PI *data,float Kr,float T, float Tn, float Tv){
	data->K_r=Kr;
	data->T=T;
	data->Tn=Tn;
	data->Tv=Tv;
	data->ek=0;
	data->ek_1=0;
	data->ek_2=0;
	data->uk_1=0;
	data->uk=0;
}
//funktion um einen Weiteren wert zu einem PI Glied hinzuzufügen
void add_val_PID(PI *data,float ek,float anti_windup,uint8_t ein){
	data->ek=ek;
	data->uk = data->K_r*data->ek;
	/*
	if(ein){
	data->uk = data->uk_1+data->K_r*(data->ek-data->ek_1+
			data->T/data->Tn*data->ek+data->Tv/data->T*(data->ek-2*data->ek_1+data->ek_2));

	if(data->uk > anti_windup){
		data->uk = anti_windup;
	}
	data->uk_1=data->uk;
	data->ek_2=data->ek_1;
	data->ek_1=data->ek;
	}else{
		data->ek=0;
		data->ek_1=0;
		data->ek_2=0;
		data->uk_1=0;
		data->uk=0;
	} */
}
//funktion zum konfigurieren eines PT1 Gliedes
void config_PT1(PT1 *data,float K,float tau){
    data->K=K;
    data->T=tau;
    data->y=0;
    data->y_1=0;
}
//funktion um einen weitern wert zu einem PT1 Gliedes hinzuzufügen
float add_val_PT1(PT1 *data,float input, float dt){
	data->u=input;
	data->dt=dt;
	data->y=(data->T*data->y_1+data->K*data->dt*data->u)/(data->dt+data->T);
	data->y_1=data->y;
    return data->y;
}
