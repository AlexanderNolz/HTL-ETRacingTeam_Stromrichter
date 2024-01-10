//
// Created by alexn on 23.06.2023.
//

#ifndef REGELUNGSTECHNIK_REGELUNGSTECHNIK_H
#define REGELUNGSTECHNIK_REGELUNGSTECHNIK_H
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
    float *anti_windup;
}PI;
typedef struct PT1{
    float Kv; //Verstärkung
    float tau; //Zeitkonstante
    float y; // Ausgang
    float last_Input; //Letzter Eingang

}PT1;

//funktion zum konfigurieren des Integrators
void config_Integrator(Integrator *data,float K);
//funktion um einen Weiteren wert zum Integrierer hinzu zufügen
void add_val_Integrator(Integrator *data,float newval,float dt);
//funktion zum konfigurieren eines P Gliedes
void config_P(P *data,float K);
//funktion um den neuen Wert neu zu berechnen
void recall_P(P *data,float val);
//funktion zum konfiguirieren eines PI Gliedes
void config_PID(PI *data,float *max_antiwinup, float K_r,float T, float Tn, float Tv);
//funktion um einen Weiteren wert zu einem PI Glied hinzuzufügen
void add_val_PID(PI *data,float ek);
//funktion zum konfigurieren eines PT1 Gliedes
void config_PT1(PT1 *data,float K,float tau);
//funktion um einen weitern wert zu einem PT1 Gliedes hinzuzufügen
float add_val_PT1(PT1 *data,float input, float dt);


#endif //REGELUNGSTECHNIK_REGELUNGSTECHNIK_H
