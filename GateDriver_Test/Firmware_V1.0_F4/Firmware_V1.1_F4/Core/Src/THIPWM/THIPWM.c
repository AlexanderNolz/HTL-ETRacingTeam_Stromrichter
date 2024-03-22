//
// Created by Alexander Nolz on 20.06.2023.
//
#ifdef __cplusplus
extern "C" {
#endif

#include "THIPWM.h"
#include <math.h>
// struct in der alle variablen der libary enthalten sind

// zur konfiguration der libary momentan nur um den Timer autoreload wert einzustellen
void setupTHIMPWM(float TIM_RelaodVal,THIPWMdata *THIPWM){
    THIPWM->TIM_ReloadVal = TIM_RelaodVal;
    THIPWM->TIM_ReloadVal_halbe = TIM_RelaodVal/2;
}
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
int vectorrecalc(float alpha, float beta, float busvoltage,THIPWMdata *THIPWM){
    //Returnvariable der Funktion genauer beschrieben in der Funktionsbeschreibung
    int status= 0x0001;
    //übergebene Werte speichern
    THIPWM->U_zk = busvoltage;
    THIPWM->alpha = alpha;
    THIPWM->beta = beta;
    //Berechnung des Betrages des übergebenen Vektorens
    THIPWM->Us_b = sqrtf(THIPWM->alpha*THIPWM->alpha+THIPWM->beta*THIPWM->beta);
    //Berechnung der halben Busspannung
    float U_zk_halbe = THIPWM->U_zk *0.5f;
    //Berechnung des maximalen Betrages bei gegebener Busspannung
    THIPWM->maximum_Us_b = U_zk_halbe * 1.154f;
    //überprüft ob Betrag des übergebenen Vektores nicht zu groß ist
    if(THIPWM->Us_b > THIPWM->maximum_Us_b){
        //Wenn der Vektor zu groß ist wird das Status bit dafür gesetzt und der Vektor zurück gerechnet
        status |= 0x0010;
        float Us_b_overflow_prozent = (THIPWM->maximum_Us_b-THIPWM->Us_b)/-THIPWM->Us_b;
        //reduzierung des alpha anteiles des Vektores
        THIPWM->alpha -= THIPWM->alpha*Us_b_overflow_prozent;
        //reduzierung des beta anteils des Vektores
        THIPWM->beta -= THIPWM->beta*Us_b_overflow_prozent;
    }else{status &= 0x1101;}// wenn der Vektor nicht zu groß ist wird der status bit zurück gesetzt
    //Berechnet den Faktor M aus dieser ist ein verhältnis zwischen Halbe Busspannung un dem Vektor betrag
    THIPWM->M=THIPWM->Us_b/U_zk_halbe;
    //Spannungen an den Phasen des Motores berechnung
    float alpha_div1_3 = THIPWM->alpha*div1_3;
    float beta_div1_3_sqrt2_3 = THIPWM->beta*div1_3*sqrt2_3;
    THIPWM->A=2.0f*alpha_div1_3;
    THIPWM->B=(-alpha_div1_3)+beta_div1_3_sqrt2_3;
    THIPWM->C=(-alpha_div1_3)-beta_div1_3_sqrt2_3;
    //Wenn der faktor M größer wie 1 ist wird die 3 harmonische iniziert
    if(1){
        // es wird die 3 harmonische ausgerechnet
        float thirdharmonic = THIPWM->alpha*(THIPWM->alpha*THIPWM->alpha-3.0f*THIPWM->beta*THIPWM->beta)/(THIPWM->alpha*THIPWM->alpha+THIPWM->beta*THIPWM->beta)/9;
        //es wird die 3 harmonische iniziert
        THIPWM->A -= thirdharmonic;
        THIPWM->B -= thirdharmonic;
        THIPWM->C -= thirdharmonic;
    }
    THIPWM->TIMCOMPA = (unsigned int)(THIPWM->TIM_ReloadVal_halbe + THIPWM->TIM_ReloadVal * (THIPWM->A/THIPWM->U_zk));
    THIPWM->TIMCOMPB = (unsigned int)(THIPWM->TIM_ReloadVal_halbe + THIPWM->TIM_ReloadVal * (THIPWM->B/THIPWM->U_zk));
    THIPWM->TIMCOMPC= (unsigned int)(THIPWM->TIM_ReloadVal_halbe + THIPWM->TIM_ReloadVal * (THIPWM->C/THIPWM->U_zk));
    return status;
}

#ifdef __cplusplus
}
#endif
