void delay(unsigned long int time)    // This function provides delay in terms of seconds
{
		//Roughly 1 second delay on 16MHz TM4C
    unsigned char i,j,k,l;
 
    for(i = 0; i < time; i++){
        for(j=0; j<250; j++){
					for(k=0; k< 250; k++){
						for (l=0; l< 60; l++){
						}
					}
				}
		}
}

float getCm(unsigned long ADCvalue){
	int adcTable[] = {4095, 3050, 1980, 1370, 950, 830, 730, 650, 570, 530, 460, 390, 330, 300, 0};
	int distTable[] = {0, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 999};
	float distance_ADC = 0;  //  <---- THIS USE TO BE CALLed distance but is now changed to distance_ADC so be aware
	float calibration = 0;
	float a = 0;
	float b = 0;
	int ia = 0;
	int ib = 0;
	float m = 0;
	float l = 0;
	float lm;
	int i;
	int f;
	for(i = 0; i < 15; i = i + 1){
			if(ADCvalue > adcTable[i]){
				break;
			}
			else{
				a = adcTable[i+1];
				ia = i+1;
			}
		}
		
		for(f = 0; f < 15; f = f + 1){
			if(ADCvalue < adcTable[f]){
				b = adcTable[f];
				ib = f;
			}
			else {
				break;
			}
		}
		 m = b - a;
		 l = b - ADCvalue;
		lm = l / m ;
		
		float distance = distTable[ib] + (lm * 5);
		return distance;
}

unsigned int getPercent(unsigned long ADCvalue){
	unsigned pct = ADCvalue/40;
	if (pct >= 100)	pct = 100;
	return pct;
}

unsigned int getAbs(int n) 
{ 
  int const mask = n >> (sizeof(int) * 8 - 1); 
  return ((n + mask) ^ mask); 
} 