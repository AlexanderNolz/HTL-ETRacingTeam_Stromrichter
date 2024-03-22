//
// Created by Alexander Nolz on 20.06.2023.
//

#ifndef THIPWM_THIPWM_H
#define THIPWM_THIPWM_H

#ifdef __cplusplus
extern "C" {
#endif

//Alle Constanten Defeniert
#define div1_3 (1.0f/3.0f)
#define sqrt2_3 sqrtf(3.0f)


// variablen struct

typedef struct variablestruct {
        //Die berechneten Timer Compare werte um sie in die zugehörigen register zu schreiben
        unsigned int TIMCOMPA;
        unsigned int TIMCOMPB;
        unsigned int TIMCOMPC;
        //Timer Reload Wert und der Halbe
        float TIM_ReloadVal;
        float TIM_ReloadVal_halbe;
        //Zwischenkreisspannung
        float U_zk;
        // Spannungsvektor
        float alpha;
        float beta;
        float Us_b;
        float A,B,C;
        // Dies ist ein verhältniss von uzk/2 zum betrag des Spannungsvektores
        float M;
        float maximum_Us_b;
}THIPWMdata;
// zur konfiguration der libary momentan nur um den Timer autoreload wert einzustellen
void setupTHIMPWM(float TIM_RelaodVal,THIPWMdata *THIPWM);

/*
 * funktion zur Neuberechnen des Spannungs Vectors
 * Beim Aufrufen dieser Funktion werden die THIPWM.COMP-A/B/C neuberechnet.
 * Die TIMCOMP Werte müssen noch extra in die Timer geschrieben werden
 *
 * Returnwerte:
 *
 * 0x000* Dieses Bit sagt ob alles geklappt hat oder bei der berechnung ein Fehler aufgetretten ist also error eine 0
 * 0x00*0 Dieses Bit wird gestezt wenn die dritte harmonische iniziert wird also THIPWM.M > 1
 * 0x0*00 Dieses Bit wird gesetzt wenn der betrag des vektors zu groß ist und reduziert wird da es wegen der Busspannung nicht möglich ist
 */

 int vectorrecalc(float alpha, float beta, float busvoltage,THIPWMdata *THIPWM);

#ifdef __cplusplus
}
#endif

#endif //THIPWM_THIPWM_H
