  /* Photoresistor Code Outline 
*
* TODO: Locate the commented lines with "TODO:" and 
* complete the task in the descriptions
*/
  /* Variables for Light Sensors*/
  // TODO: Replace "___", and assign the pin number connected to the Arduino
  int LDR_Pin0 = A0; // analog pin 
  int LDR_Pin1 = A1; // analog pin 
  int LDR_Pin2 = A2; // analog pin 
  int LDR_Pin3 = A3; // analog pin 
  int LDR_Pin4 = A4; // analog pin 
  int LDR_Pin5 = A5; // analog pin 
  int LDR_Pin6 = A6; // analog pin 
  
  // Initialize Photo Resistor Variables to zero
  int LDR0 = 0, LDR1 = 0, LDR2 = 0, LDR3 = 0, LDR4 = 0, LDR5 = 0, LDR6 = 0; 
 
void setup() {
 Serial.begin(9600); // TODO: Replace "___", and input the baud rate for serial communication
}
void loop() {
 ReadPhotoResistors(); // Read photoresistors and map to 0-100 based on calibration
 Print(); // Print values to serial monitor
}

// ************************************************************************************************* //
// function to read photo resistors
void ReadPhotoResistors()
{
// TODO: Replace "___", and use a function to read the value of the corresponding LDR Arduino pin
  LDR0 = analogRead(LDR_Pin0)/10;
    delay(2);
  LDR1 = analogRead(LDR_Pin1)/10;
    delay(2);
  LDR2 = analogRead(LDR_Pin2)/10;
    delay(2);
  LDR3 = analogRead(LDR_Pin3)/10;
    delay(2);
  LDR4 = analogRead(LDR_Pin4)/10;
    delay(2);
  LDR5 = analogRead(LDR_Pin5)/10;
    delay(2);
  LDR6 = analogRead(LDR_Pin6)/10;
    delay(2); 
    
}

// ************************************************************************************************* //
// function to print values of interest
void Print()
{
  Serial.print(LDR0); Serial.print(" ");
  Serial.print(LDR1); Serial.print(" ");
  Serial.print(LDR2); Serial.print(" ");
  Serial.print(LDR3); Serial.print(" ");
  Serial.print(LDR4); Serial.print(" ");
  Serial.print(LDR5); Serial.print(" ");
  Serial.println(LDR6); 
  delay(200); //just here to slow down the output for easier reading if desired
}