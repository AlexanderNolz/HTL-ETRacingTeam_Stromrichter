uint32_t ADC3_Endcoder_sin_cos[2];
uint32_t sin_int;
uint32_t cos_int;
float theta;
float theta_grad;
float calctheta(uint32_t sin, uint32_t cos){
	float fcos = (float)(cos)-2857.67f;
	float fsin = (float)(sin)-2857.67f;
	float theta = atan2f(fcos,fsin);
	return theta;
}
