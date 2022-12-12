// code to implement ZOH (single-input-single-output)

int out_DAC1 = 0;
float a0,a1;
float outputValue = 0;
int u=0;


void setup() {
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  analogReadResolution(12);   // set the analog input resolution to 12 bit 
  analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC1
}

void loop() {
          a0 = analogRead(0);
          a1 = analogRead(1);
          
          Ref = a0;
  
          outputValue = a0;

          u = (int) outputValue;

          out_DAC1 = u; 
          
          analogWrite(DAC1,out_DAC1);
}
