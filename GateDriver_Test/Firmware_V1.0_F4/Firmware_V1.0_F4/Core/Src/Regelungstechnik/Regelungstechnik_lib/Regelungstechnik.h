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
    Integrator I;
    P P;
    float val;
}PI;
//funktion zum konfigurieren des Integrators
void config_Integrator(Integrator *data,float K);
//funktion um einen Weiteren wert zum Integrierer hinzu zufügen
void add_val_Integrator(Integrator *data,float newval,float dt);
//funktion zum konfigurieren eines P Gliedes
void config_P(P *data,float K);
//funktion um den neuen Wert neu zu berechnen
void recall_P(P *data,float val);
//funktion zum konfiguirieren eines PI Gliedes
void config_PID(PI *data,float KP,float KI);
//funktion um einen Weiteren wert zu einem PI Glied hinzuzufügen
void add_val_PID(PI *data,float newval,float dt);


#endif //REGELUNGSTECHNIK_REGELUNGSTECHNIK_H
