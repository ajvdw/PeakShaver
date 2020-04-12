#include <PID_v1.h>         // https://github.com/br3ttb/Arduino-PID-Library
#include <AltSoftSerial.h>  // https://github.com/PaulStoffregen/AltSoftSerial

#define OUTPUT_MIN 6.0      // Min Charging Amp
#define OUTPUT_MAX 16.0     // Max Charging Amp

#define SETPOINT   5.75     // kW
#define OVERSHOOT  0.25     // kW

#define PAUSE      600      // Sec
#define SAMPLES    300      // Number of samples  

int HourOfDay;

double inputPower;              // Input Nett Power measurement
double setpointPower=SETPOINT;  // Setpoint Power in kW
double outputCurrent=OUTPUT_MIN;// Ouput Charging current in A

#define KP  0.7
#define KI  0.1
#define KD  0.05

PID myPID(&inputPower, &outputCurrent, &setpointPower, KP, KI, KD, DIRECT);

AltSoftSerial altSerial; // D9=TX, D8=RX, D10=dir

char hex[] = "0123456789ABCDEF";

uint8_t getCheckSum8Xor(char *s) {
  int cs = 0; 
  for (int i = 0; s[i]; i++) cs = cs ^ (s[i]);
  return cs; 
}

uint8_t getCheckSumMod256(char *s) {
  int cs = 0; 
  for (int i = 0; s[i]; i++) cs = (cs + s[i]) & 255;
  return cs; 
}

bool parseTime( const char *parse, char *buffer, long *tim ) {
  int l=strlen(parse);
  if( strncmp( parse, buffer, l) == 0 ) {
     if( char *par = strchr(buffer,'(') ) {
        *tim = round( atof(par+7) ) % 1000000;
        return true;
     }
  }
  return false;
}

bool parseDouble( const char *parse, char *buffer, double *dval ) {
  int l=strlen(parse);
  if( strncmp( parse, buffer, l) == 0 ) {
     if( char *par = strchr(buffer,'(') ) {
        *dval = atof( par+1 );
        return true;
     }
  }
  return false;
}

bool receiveDSMR( void ) {
  long timepart;
  bool result = false;
  static long prevtimepart=0;
  static char buffer[256];
  static int bufpos=0;

  static double Import=0, Export=0;

  if (Serial.available()) {
    char input = Serial.read();
    buffer[bufpos] = input&127; // Remove Upper bit (parity)
    if( bufpos < 255) bufpos++;

    if (input == '\n' && bufpos > 1)  { // End of line
      buffer[bufpos]=0; // Terminate string

      // Capture Timestamp like (200314094022W)
      if( parseTime("0-0:1.0.0",buffer, &timepart) ) {
        if( timepart != prevtimepart ) result = true;
        prevtimepart = timepart;
        HourOfDay = (timepart / 10000) % 100;
      }

      // Capture Import power kW
      if( parseDouble("1-0:1.7.0",buffer,&Import) && (Import > 0 ) ) inputPower = (Import-Export);
      // Capture Export power kW
      if( parseDouble("1-0:2.7.0",buffer,&Export) && (Export > 0 ) ) inputPower = (Import-Export);
          
      bufpos=0; // Go to start of buffer
    }
  } 
  return result; 
} 


void sendEVBSendMessage( char *msg ) {
  uint8_t csm, csx;
 
  // Calc checksum
  csm = getCheckSumMod256( msg );
  csx = getCheckSum8Xor( msg );

  digitalWrite( 10, HIGH ); // TX mode 
  // StartOfMessage
  altSerial.print( char(2) );  
  // Actual Message
  altSerial.print( msg );  
  // Add checksum to message
  altSerial.print( char(hex[csm >> 4]) );  
  altSerial.print( char(hex[csm & 15]) );  
  altSerial.print( char(hex[csx >> 4]) );  
  altSerial.print( char(hex[csx & 15]) );  
  // EndOfMessage
  altSerial.print( char(3) ); altSerial.flush();
  digitalWrite( 10, LOW ); // RX mode

  Serial.print( "TX: " ); Serial.println( msg );
}

void sendEVBmaxCurrent( float amp ) {
  // MaxChargingCurrent command
  char buf[] = "80A06900__00__00__" ;   
  int  ta = round(10*amp);
  // Set current values (fill in the blanks)
  buf[8]=buf[12]=buf[16]=hex[(ta >> 4) & 15];
  buf[9]=buf[13]=buf[17]=hex[(ta >> 0) & 15];
  sendEVBSendMessage( buf );
}

void setup() {
  altSerial.begin(38400);
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  pinMode( 10, OUTPUT ); 
  digitalWrite( 10, LOW ); // RX mode

  myPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  myPID.SetSampleTime(1000);
  myPID.SetMode(AUTOMATIC);
  
  Serial.println("PeakShaving");
}

void loop() {
  char c;
  static int lenbuf=0;
  static char cmdbuf[256];
  static bool reading = false;
  static int countdown = 0;
  static int samples = 0;
  static double average = 0; 

  myPID.Compute(); 

  // Read smart meter data and process the data
  if( receiveDSMR() ) { // True if one second has passed 
    Serial.print( outputCurrent ); Serial.print( "A "); Serial.print( inputPower ); Serial.println( "kW ");
    if( samples < SAMPLES ){ // Collect first samples 
      average += inputPower / SAMPLES;
      samples ++;     
    }
    if( samples == SAMPLES ) // Once we have enough samples
    { // Calculate moving average
      average=(average * (SAMPLES-1) + inputPower )/ SAMPLES;
      // Minimum charging current can lead to overshoot 
      if( average > SETPOINT+OVERSHOOT ){
        sendEVBmaxCurrent( 0 );  // Stop charging for a while
        countdown = PAUSE;
      }
    }
    if( countdown )
    {
      Serial.print( "Waiting: " ); 
      Serial.println(countdown);
      countdown--; // Let time pass by 
    }
    else
    {
      if( HourOfDay > 9 && HourOfDay < 18 ) // Solar Charging
        sendEVBmaxCurrent( OUTPUT_MIN ); //
      else
        sendEVBmaxCurrent( outputCurrent ); // Set charging current
    }
  }

  // Capture EVBox data and just print it
  if (altSerial.available()) {
    c = altSerial.read();

    if( c == 2 ) { // Message Start  
       reading=true;
       lenbuf=0;
    }
    else if( c == 3 && lenbuf > 8) { // Message End 
      reading=false;
      cmdbuf[lenbuf]=0;
      Serial.print( "RX: " ); Serial.println( cmdbuf );  
      lenbuf=0;     
    }
    else if( reading && c >= 48 && c<= 70 ) { // Capture message data
      if( lenbuf<255) cmdbuf[lenbuf++]=c;
    }
    else { // Invalid data
      lenbuf=0;
    }
  }    
}
