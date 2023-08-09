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
void config_PI(PI *data,float KP,float KI){
    config_P(&data->P,KP);
    config_Integrator(&data->I,KI);
    data->val = 0;
}
//funktion um einen Weiteren wert zu einem PI Glied hinzuzufügen
void add_val_PI(PI *data,float newval, float dt){
    add_val_Integrator(&data->I,newval,dt);
    recall_P(&data->P,newval);
    data->val=data->P.val+data->I.val;
}