#include <PID_v1.h>
#include <AltSoftSerial.h>

#define OUTPUT_MIN 6
#define OUTPUT_MAX 16
#define KP .5
#define KI .5
#define KD .5

double powerMeter;          // Input Nett Power measurement
double powerSetpoint=6.0;   // Setpoint Nett Power in kW
double outputCurrent=6;     // Ouput Charging current in A

//input/output variables passed by reference, so they are updated automatically
PID myPID(&powerMeter, &outputCurrent, &powerSetpoint, KP, KI, KD, DIRECT);

AltSoftSerial altSerial; // D9=TX, D8=RX, D10=dir

uint8_t getCheckSum8Xor(char *string)
{
  int cs = 0; 
  for (int i = 0; i < strlen(string); i++) cs = cs ^ (string[i]);
  return cs; 
}

uint8_t getCheckSumMod256(char *string)
{
  int cs = 0; 
  for (int i = 0; i < strlen(string); i++) cs = (cs + string[i]) & 255;
  return cs; 
}


bool parseTime( char *parse, char *buffer, long *tim )
{
  int l=strlen(parse);
  if( strncmp( parse, buffer, l) == 0 )
  {
     String str=String(buffer);
     int par = str.indexOf("(");

     if( par >= 0 )
     {
        String value = str.substring(par+7);    
        *tim = round(value.toFloat()) % 1000000;
        return true;
     }
  }
  return false;
}


bool parseDouble( char *parse, char *buffer, int nth, double *dval )
{
  int l=strlen(parse);
  if( strncmp( parse, buffer, l) == 0 )
  {
     String str=String(buffer);
     int par = -1; 
     while( nth > 0 )
     {
      
        str = str.substring(par+1);
        par = str.indexOf("(");
        nth--;
     }

     if( par >= 0 )
     {
        String value = str.substring(par+1);    
        *dval = value.toFloat();
        return true;
     }
  }
  return false;
}

bool handleDSMR( void ) 
{
  long timepart;
  bool result = false;
  static long prevtimepart=0;
  static char buffer[256];
  static int bufpos=0;

  static double Import=0, Export=0;

  if (Serial.available()) 
  {
     char input = Serial.read();
   
    buffer[bufpos] = input&127; // Upper bit is never used
    if( bufpos < 255) bufpos++;

    if (input == '\n' && bufpos > 1) 
    { 
      buffer[bufpos]=0; // Terminate string

      // Timestamp like (200314094022W)
      if( parseTime("0-0:1.0.0",buffer, &timepart) ) 
      {
        if( timepart != prevtimepart ) result = true;
        prevtimepart = timepart;
      }

      // Import power kW
      if( parseDouble("1-0:1.7.0",buffer,1,&Import) && (Import > 0 ) ) powerMeter = (Import-Export);
      // Export power kW
      if( parseDouble("1-0:2.7.0",buffer,1,&Export) && (Export > 0 ) ) powerMeter = (Import-Export);
          
      bufpos=0; // Go to start of buffer
    }
  } 
  return result; // Return true if a second has passed
} 


void sendEVBmaxCurrent( float amp )
{
  char buf[64];
  int  ta = round(10.0 * amp);
  uint8_t csm, csx;
  char hex[] = "0123456789ABCDEF";

  // Template command
  strcpy( buf, "<SECRET>" );

  // Set current values
  buf[8]=buf[12]=buf[16]=hex[(ta >> 4) & 15];
  buf[9]=buf[13]=buf[17]=hex[(ta >> 0) & 15];
  // Calc checksum
  csm = getCheckSumMod256( buf );
  csx = getCheckSum8Xor( buf );
  // Add checksum to buffer
  buf[34]=hex[(csm >> 4) & 15];  buf[35]=hex[(csm >> 0) & 15];
  buf[36]=hex[(csx >> 4) & 15];  buf[37]=hex[(csx >> 0) & 15];
  // EndOfString
  buf[38]=0;

  digitalWrite( 10, HIGH ); // TX mode 
  altSerial.print( char(2) );  altSerial.print( buf );  altSerial.print( char(3) ); altSerial.flush();
  digitalWrite( 10, LOW ); // RX mode

  //Serial.print( "TX: " ); Serial.println( buf );
}

void setup() 
{
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  Serial.println("PeakShaving");
  altSerial.begin(38400);

  // RX mode
  pinMode( 10, OUTPUT ); 
  digitalWrite( 10, LOW ); 

  myPID.SetMode(AUTOMATIC);
}

void loop() {
  char c;
  static int lenbuf=0;
  static char cmdbuf[256];
  static bool reading = false;

  myPID.Compute(); 

  if( handleDSMR() ) 
  {    
    if( outputCurrent < OUTPUT_MIN) outputCurrent = OUTPUT_MIN;
    if( outputCurrent > OUTPUT_MAX) outputCurrent = OUTPUT_MAX;
    
    //Serial.print(": "); Serial.print( outputCurrent ); Serial.print( "A "); Serial.print( powerMeter ); Serial.println( "kW ");
    sendEVBmaxCurrent( outputCurrent );  

  }

  
  if (altSerial.available()) 
  {
    c = altSerial.read();

    if( c == 2 ) 
    {  
       // Message Start  
       reading=true;
       lenbuf=0;
    }
    else if( c == 3 && lenbuf > 8) 
    { 
      // Message End
      reading=false;
      cmdbuf[lenbuf]=0;
      //Serial.print( "RX: " ); Serial.println( cmdbuf );  
      lenbuf=0;     
    }
    else if( reading && c >= 48 && c<= 70 ) 
    {
      // Capture message data
      if( lenbuf<255) cmdbuf[lenbuf++]=c;
    }
    else
    {
      // Invalid data
      lenbuf=0;
    }
  }    
}
