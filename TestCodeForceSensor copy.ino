// Teensy 4.1 Force Sensor Reader
// Reads analog voltage from FlexiForce circuit and streams to serial

const int PIN_FORCE = A0;      // analog input pin (PIN 14!!!)
const int VREF = 3;            // Teensy logic voltage
const int ADC_BITS = 12;       // 12-bit ADC (0–4095)

const int numReadings = 50;    // initializing array size to take
float readings[numReadings];   // the average of the last numReadings 
int idx = 0;                   // voltages recorded to plot a cleaned
float total = 0;               // version of the raw signal
float avg = 0;  

void setup() {                 // Runs once when program is started
  Serial.begin(115200);        // open serial connection for MATLAB to use (BAUD number in MATLAB)
  analogReadResolution(ADC_BITS);
  Serial.println("volts");     // header line for MATLAB

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;           // initializing array
  }
  
}

void loop() {                  // runs after setup and loops until stopped
  int raw = analogRead(PIN_FORCE); // returns number 0-4095 (0 = 0V; 4095 = 3V)
  float volts = (raw / (float)(pow(2, ADC_BITS - 1))) * VREF; // converts to voltage measurement
  
  total = total - readings[idx]; // removes oldest voltage measurements and replaces with newest
  readings[idx] = volts;         
  
  Serial.println(volts, 4);    // send as text to computer (4 decimal places)

  total = total + readings[idx]; // updates moving average of voltages to most recent measurements
  idx = idx + 1;

  if (idx >= numReadings) {    // resetting index tracker to 0 when reaches the end of the array
    idx = 0;
  }

  avg = total/numReadings;    // calculates average of last numReading voltage measurements
  Serial.println(avg, 4);
  delay(5);                 // small delay (200 Hz sampling)
  
}
