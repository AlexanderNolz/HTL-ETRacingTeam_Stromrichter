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
PI PI_Regler_alpha;
  //####################################################################################################
//Stromregler beta
PI PI_Regler_beta;
  //####################################################################################################

uint32_t pwm_periode = 1000-1;
//####################################################################################################
//Regler Einstellungen
float Kr=0.7;
float TN=0.001f*0.5f;
float TV=0.0001f*0.12f;
float T_pwm = 0.0001;
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
	if(adc_val < 50){
		gas=0;
	}else if(adc_val > 1300){
		gas=1;
	}else{
		gas=(float)(adc_val)/1300.0f;
		}
	return gas;
}
float clalc_uzk(uint32_t uzk){
	float uzkf = (float)(uzk)*21.72121f;
	return uzkf;
}
