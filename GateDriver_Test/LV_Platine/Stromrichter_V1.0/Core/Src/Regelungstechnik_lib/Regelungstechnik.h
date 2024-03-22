//
// Created by alexn on 23.06.2023.
//

#ifndef REGELUNGSTECHNIK_REGELUNGSTECHNIK_H
#define REGELUNGSTECHNIK_REGELUNGSTECHNIK_H
#include "stdint.h"

//Integrator struct
typedef struct Integrator{
    float val;
    float K;
}Integrator;
//P struct
typedef struct P{
    float K;
    float val;
}P;
//PI struct
typedef struct PI{
	float K_r;
	float ek;
	float ek_1;
	float ek_2;
	float uk_1;
	float T;
	float Tn;
	float Tv;
    float uk;
}PI;
typedef struct PT1{
    float K; //Verstärkung
    float dt; //Zeitkonstante
    float y; // Ausgang
    float y_1;
    float T;
    float u;

}PT1;

typedef struct di_dt_protection{
	float I_a;
	float I_b;
	float _Is_;
	float _Is__1;
	float di_dt;
	float di_dt_threshold;
	uint8_t error;

}di_dt_protection;

typedef struct {
	float d1,d2,c0,c1,c2,uk,uk_1,uk_2,ek,ek_1,ek_2;

}PIDT1;

typedef struct {
	float uk,uk_1,ek,ek_1,Tv,T1,T,Kr;

}PDT1;

//funktion zum konfigurieren des Integrators
void config_Integrator(Integrator *data,float K);
//funktion um einen Weiteren wert zum Integrierer hinzu zufügen
void add_val_Integrator(Integrator *data,float newval,float dt);
//funktion zum konfigurieren eines P Gliedes
void config_P(P *data,float K);
//funktion um den neuen Wert neu zu berechnen
void recall_P(P *data,float val);
//funktion zum konfiguirieren eines PI Gliedes
void config_PID(PI *data, float K_r,float T, float Tn, float Tv);
//funktion um einen Weiteren wert zu einem PI Glied hinzuzufügen
void add_val_PID(PI *data,float ek,float uk_1,uint8_t ein);
//funktion zum konfigurieren eines PT1 Gliedes
void config_PT1(PT1 *data,float K,float tau);
//funktion um einen weitern wert zu einem PT1 Gliedes hinzuzufügen
float add_val_PT1(PT1 *data,float input, float dt);
//funktion zum konfiguirieren des stromänderungs schutzes
void config_didt_p(di_dt_protection *data, float threshold);
//funktion  zum berechnen der stromänderung
uint8_t check_di_dt(di_dt_protection *data, float alpha, float beta, float dt);
//funktion zum konfigurieren eines PIDT1-gliedes
void config_PIDT1(PIDT1 *data, float T, float Tn, float Tv, float T1, float Kr);
//funktion um einen Weiteren wert zum PIDT1 Regler hinzuzufügen
void add_val_PIDT1(PIDT1 *data, float ek, float uk_1, uint8_t ein);
//funktion zum konfigurieren eines PDT1-gliedes
void config_PDT1(PDT1 *data, float T, float Tv, float T1, float Kr);
//funktion um einen Weiteren wert zum PIDT1 Regler hinzuzufügen
void add_val_PDT1(PDT1 *data, float ek, float uk_1, uint8_t ein);



#endif //REGELUNGSTECHNIK_REGELUNGSTECHNIK_H
