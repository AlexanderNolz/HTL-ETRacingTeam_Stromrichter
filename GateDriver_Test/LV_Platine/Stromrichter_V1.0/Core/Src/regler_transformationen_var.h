  //####################################################################################################
//THIPWM intialisierung
THIPWMdata THIPWM;
  //####################################################################################################
//Park intialisieirung
park Park1 = park_default;
  //####################################################################################################
//Clark initalisierung
clark Clark;
  //####################################################################################################
//Stromregler alpha
PI Regler_alpha;
PDT1 filter_alpha;
  //####################################################################################################
//Stromregler beta
PI Regler_beta;
PDT1 filter_beta;
  //####################################################################################################
//winkel geschwindigkeit
float theta_1 = 0;
float theta_is_ist=0;
float theta_is_soll=0;
float theta_is_differenz;
PT1 theta_is_filter;
PT1 omega_filter;
float omega_rotoe;

uint32_t pwm_periode = 1500-1;
//####################################################################################################
//Regler Einstellungen
const float Kr=0.1656f;
const float Tn=0.0004029f;
const float Tv=1.058e-8f;
const float T1=4.719e-3f;
const float T1_PDT1=1.59155e-7f;
const float T2_PDT1=3.59155e-4f;
float T_pwm = 0.00005f;
//########################################################################################################
PT1 offset_corrc_U;
PT1 offset_corrc_V;
PT1 offset_corrc_W;

float PT1_offset_K =1.0f;
float PT1_offset_T=0.01f;

//####################################################################################################
//Soll Werte
float Iq=0;
float Id=0;
uint32_t ADC2_gas_uzk[2];
float gas=0;
float uzk=0;

float clalc_gas(uint32_t adc_val){
	float gas = 0;
	if(adc_val > 3550){
		gas=0;
	}else if(adc_val < 300){
		gas=1;
	}else if(adc_val < 100){
		gas=0;
	}else{
		gas=1.0f-(float)(adc_val)/3550.0f;
		}
	return gas;
}
float clalc_uzk(uint32_t uzk){
	float uzkf = (float)(uzk)*21.72121f;
	return uzkf;
}


//##################################################################################################################

di_dt_protection di_dt_schutz;
float di_dt_max = 5000/(0.001); // 50 Ampere pro milisekunde maximales di/dt

uint16_t omega_i=0;
float angle_sum;
float angle_1;
float omega_val;

float omega(float angle,float dt){
	if(omega_i < 5000){
		omega_i++;
		angle_sum+=angle - angle_1;
	}else{
		omega_val = angle_sum/(5001.0f * dt);
		omega_i=0;
	}
	angle_1 = angle;
	return omega_val;
}

