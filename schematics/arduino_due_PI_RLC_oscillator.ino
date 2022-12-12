// arquivo do paper de politica estacionaria
// codigo realiza controle PI sobre circuito RLC
// usa a referencia senoidal (gerada por gerador de funcoes)
// para fazer com que o RLC tenha oscilacao senoidal

int out_DAC1 = 0;
int flag = 0;
int count = 0;
int i=0;
int q0=0,q1=0;
float a0,a1;
int Ref = 0;
float Error = 0;
float outputValue = 0;
float I = 0;
float Kp = 20;
float Ki = 0.0;
int u=0;


void setup() {
  //Serial.begin(9600);
  //int t=analogRead(0);

  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  analogReadResolution(12);   // set the analog input resolution to 12 bit

  //ADC->ADC_MR |= 0x80; // these lines set free running mode on adc 7 and adc 6 (pin A0 and A1 - see Due Pinout Diagram thread)
  //ADC->ADC_CR=2;
  //ADC->ADC_CHER=0xC0; // this is (1<<7) | (1<<6) for adc 7 and adc 6                                     
 
  analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC1
}

void loop() {
  //int t=micros();
    //for(int i=0;i<50000;i++){
      //i++;  
      //if ( ( i % 2) == 0){ Ref=0; u = 0; tiempo=100; }
      //else { Ref=1600; tiempo=200;}
    
      //Ref=1600;
      //for (int px = 0; px <= tiempo; px++) {
        
          //q0=0; q1=0;
          //for (int r = 1; r <= 5; r++) {
          //    while((ADC->ADC_ISR & 0xC0)!=0xC0);// wait for two conversions (pin A0[7]  and A1[6])
          //        q0 = q0 + ADC->ADC_CDR[7];              // read data on A0 pin;
          //        q1 = q1 + ADC->ADC_CDR[6];              // read data on A1 pin;
          //    }
          //    a0=q0/5;
          //    a1=q1/5;
          
          //while((ADC->ADC_ISR & 0xC0)!=0xC0);// wait for two conversions (pin A0[7]  and A1[6])
          //a0 = ADC->ADC_CDR[7];
          //a1 = ADC->ADC_CDR[6]; 
          a0 = analogRead(0);
          a1 = analogRead(1);
          
          Ref = a0;
        
          Error = Ref - 10*a1; 
          //I = I+Error;
          //outputValue = Error + Ki*I;
          outputValue = Error;

          u = (int) outputValue;
          //if (( u < 0)||(Ref == 0)){ u=0; I=0; Error = 0;} 
          //if (Ref == 0){ u=0; I=0; Error = 0;} 
          //out_DAC1 = Ref;
          out_DAC1 = a0; 
          
          //if (flag==0){flag = 4000;}
          //else {flag=0;}
          
          analogWrite(DAC1,out_DAC1);
          
          //out_DAC1 = flag;
          //dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
          //dacc_write_conversion_data(DACC_INTERFACE,  out_DAC1);//write on DAC

          //dacc_set_channel_selection(DACC_INTERFACE, 0);       //select DAC channel 0
          //dacc_write_conversion_data(DACC_INTERFACE,  a1);//write on DAC         
      //}
   //}
}
