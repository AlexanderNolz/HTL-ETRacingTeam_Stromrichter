//
// Created by alexn on 23.06.2023.
//

#include "Regelungstechnik.h"
#include "math.h"
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
void add_val_PID(PI *data,float ek,float uk_1,uint8_t ein){
	data->ek=ek;
	data->uk_1=uk_1;
	//data->uk = data->K_r*data->ek;

	if(ein){
	data->uk = data->uk_1+data->K_r*(data->ek-data->ek_1+
			data->T/data->Tn*data->ek+data->Tv/data->T*(data->ek-2*data->ek_1+data->ek_2));

	data->ek_2=data->ek_1;
	data->ek_1=data->ek;
	}else{
		data->ek=0;
		data->ek_1=0;
		data->ek_2=0;
		data->uk_1=0;
		data->uk=0;
	}
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
void config_didt_p(di_dt_protection *data, float threshold){
	data->I_a=0;
	data->I_b=0;
	data->_Is_=0;
	data->_Is__1=0;
	data->di_dt=0;
	data->di_dt_threshold=threshold;
	data->error=0;
}

uint8_t check_di_dt(di_dt_protection *data, float alpha, float beta, float dt){
	data->I_a=alpha;
	data->I_b=beta;
	data->_Is_ = sqrtf(data->I_a*data->I_a+data->I_b*data->I_b);
	data->di_dt=(data->_Is_-data->_Is__1)/dt;
	if(data->di_dt > data->di_dt_threshold){
		data->error =1;
	}
	data->_Is__1=data->_Is_;
	return data->error;
}

//funktion zum konfigurieren eines PIDT1-gliedes
void config_PIDT1(PIDT1 *data, float T, float Tn, float Tv, float T1, float Kr){
	data->d1 = (T+2.0f*T1)/(T+T1);
	data->d2=-T1/(T+T1);
	data->c0=Kr*(1+T/Tn+Tv/(T+T1));
	data->c1=Kr*((T*T+T*Tn-2.0f*Tv*Tn)/(Tn*(T+T1))-T/Tn-2.0f);
	data->c2=Kr*((Tv+T1)/(T+T1));
	data->uk= 0;
	data->uk_1= 0;
	data->uk_2= 0;
	data->ek= 0;
	data->ek_1= 0;
	data->ek_2 = 0;
}
//funktion um einen Weiteren wert zum PIDT1 Regler hinzuzufügen
void add_val_PIDT1(PIDT1 *data, float ek, float uk_1,uint8_t ein){
	data->ek=ek;
	data->uk_1=uk_1;
	if(ein){
		data->uk=data->d1*data->uk_1+data->d2*data->uk_2+data->c0*data->ek;//+data->c1*data->ek_1+data->c2*data->ek_2;
		data->uk_2=data->uk_1;
		data->ek_1=data->ek;
		data->ek_2=data->ek_1;
	}else{
		data->uk= 0;
		data->uk_1= 0;
		data->uk_2= 0;
		data->ek= 0;
		data->ek_1= 0;
		data->ek_2 = 0;
	}

}
//funktion zum konfigurieren eines PDT1-gliedes
void config_PDT1(PDT1 *data, float T, float Tv, float T1, float Kr){
	data->T=T;
	data->Tv=Tv;
	data->T1=T1;
	data->Kr=Kr;
	data->ek=0;
	data->ek_1=0;
	data->uk=0;
	data->uk_1=0;
}
//funktion um einen Weiteren wert zum PIDT1 Regler hinzuzufügen
void add_val_PDT1(PDT1 *data, float ek, float uk_1, uint8_t ein){
	data->ek=ek;
	data->uk_1=uk_1;
	if(ein){
		data->uk = (data->T1 * data->uk_1 - data->Kr * data->Tv * data->ek_1 + data->ek * (data->Kr * data->T + data->Kr * data->Tv)/(data->T+data->T1));
		data->uk_1=data->uk;
	}else{
		data->uk= 0;
		data->uk_1= 0;
		data->ek= 0;
		data->ek_1= 0;
	}
}



