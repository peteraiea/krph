#include <LiquidCrystal.h>   //use LCD library
// 30oct16 : macbook dev platform
// // Rev 1.2.35 30apr17 VCC read function added
//                    use voltages instead of raw to check voltage fluctions
//                    time between shots affect reading and repeat readings cause system to drift!
//                    averaging attempts bias readings in direction of drift (? down) 
//                    Green is not as strong as blue ... may need to move in closer?
// 1.2.38 24may17     moved Green to pin 9 - pin 10 may be weak
//                    HP cuvette holder settins  1.7" path 43.8 mm
// 
// 2.3.1 25may17     changing versioning  to MakeV.swRev
//                    buildV is the electronics buildV == 2
//                    makeV is the holder make
//                    softV is unit is days worked on SW agains6 buildV.makeV  so today is day 1
//                    flash # - the nth flash with a software change - 
//                       all the debugging goes into piling up this # 
//2.3.4.1 29jun17    softV change to 4 - calibration version for cuvette V3
//                   flash 1 updates version# and slope/intercept calibration cuvette V3
//2.3.5.1 21aug17    test out SAMPLE 10 -- the original reading method
//2354               serial command processing loop
//2363 24aug17 command processing for single byte commands sent
//  via serial link. implemented r and b
//      recall previous blank    R
//  git repo created 8/27
// serial processing enabled !
//2382 31oct17 - removed pH display on lcd.
// softV 7 -- fix error  Green Blue reversed

#define ProgName "krph"
#define devSystem "jupiter"
#define Build "wirewrap"
#define Pathlength "35mm"
#define Sample "3 ml"
#define FirmwareDate "31oct17"
#define buildV "2"
#define makeV  "3"
#define softV  "8" // 31oct17
#define flash  "3" // Serial command processing test - continue 

// 7 temp conv - 8 back to 25 N = 20

#define TSL257 3
// v7  GREEN is 8, BLUE is 9  --- this reverses v6 assignment   LOOK
//#define GREEN 9 
//#define BLUE  8
#define GREEN 8
#define BLUE  9
#define BlankButton 7
#define ReadButton  6

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);// LCD pin

//shot parameters
double Samples=100;      //LOOK 2.3.5.1  FROM 200
//double blankSamples = 200; // LOOK 2.3.1.15  --- discontinue blank Samples
int    onDelay = 100;     // turn on LED, onDelay time, read   100 init  500 better
int    ShotDelay = 1000;  // h4 wait between shots 1000   set to 0 in h5  LOOK

// LOOK 2.1.35 change to unsigned int - raw ad reading
unsigned long x1a=0; // Blank
unsigned long y1a=0; // Blank
unsigned long x2a=0; // Read
unsigned long y2a=0; // Read
unsigned long blGreen=0; // backup value  LOOK 2363
unsigned long blBlue=0; // backup value LOOK 2363

// LOOK 2.1.35 voltages of raw ad
double x1aV = 0; //Blank
double x2aV = 0; //Blank
double y1aV = 0; //Read
double y2aV = 0; //Read
double A_1 = 0.000;//absorbance
double A_2 = 0.000;//absorbance
double RB = 0.000; // broadband absorption ratio
double Vcc = 0;
long vala = 0;  //"blank" button value
long valb = 0; //"sample" button value

// VCC compensation for AD v2.1.35
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  //Serial.print("debug readVcc result: ");
  //Serial.println( result );
  return result; // Vcc in millivolts
}

// -- this is terrible but I'm rusty --
char * version()
{
  char Version[15];
  strcpy(Version,buildV);
  strcat(Version,"\.");
  strcat(Version,makeV);
  strcat(Version,"\.");
  strcat(Version,softV);
  strcat(Version,"\.");
  strcat(Version,flash);
  return (Version);
}

void showsetup(){
  char ver[15];  
  strcpy (ver,version());

  lcd.begin(16, 2);  //Initialize LCD
  lcd.clear();
  lcd.print(ProgName);  //sample code signature - not KRDisplay Mini Spectrophotometer
  lcd.print(" ");
  lcd.print(devSystem);
  lcd.setCursor(0, 1) ;
  lcd.print(FirmwareDate);
  lcd.print(" ");
  lcd.print(ver);
  // delay(1000); //Delay1000ms
  lcd.setCursor(0,1);
  // serial debug output
  Serial.print("Program Name:");
  Serial.println(ProgName);
  Serial.print("Dev System: ");
  Serial.println(devSystem);
  Serial.print("Arduino firmwaredate: ");
  Serial.println (FirmwareDate);
  Serial.print("Build: ");
  Serial.println(Build);
  Serial.print("Pathlength:");
  Serial.println(Pathlength);
  Serial.print("Sample Size: ");
  Serial.println(Sample);
  Serial.print("N Shots per Sample: ");
  Serial.println(Samples,0);
  Serial.print("Version: ");
  Serial.println(ver);
  Serial.println("Target pH range: 7.6 - 8.2");
  Serial.print("Vcc: ");
  Serial.println(Vcc);
  
}

void setup()
{
  Serial.begin(9600);
  Vcc = readVcc()/1000.0;
  showsetup();
  pinMode(BlankButton,INPUT); //setup blank button
  pinMode(ReadButton,INPUT);//setup sample button
  pinMode(BLUE,OUTPUT);
  pinMode(GREEN,OUTPUT);
  // green flashes first for second
  digitalWrite(GREEN,HIGH);
  delay (1000);
  digitalWrite(GREEN,LOW);
  digitalWrite(BLUE,HIGH);
  delay (1000);
  digitalWrite(BLUE,LOW);
 
}


void blankProcess()
{
    lcd.clear(); //clear
    lcd.print("Blank Process");
    Serial.println("\nBlank Process Start");
    Serial.println("values should be from 600 to 900");
    Serial.println("adjust with potentiometers as needed");

    //analogWrite(GREEN,10);
    // GREEN =  x
    blGreen = x1a;  // store backup value v2363
    blBlue = y1a;


    x1a = 0;
    x1aV = 0;
    unsigned int x = 0;
    
    digitalWrite(GREEN,HIGH);//Green
    delay (2000);
    for(int i=0; i<Samples; i++){
      x1a += analogRead(TSL257);
    }
    digitalWrite(GREEN,LOW);
    
    x1a /= Samples ; //read the blank - avg
    
    // BLUE == y
    y1a = 0;
    unsigned int y=0;
    y1aV =0;
    
    digitalWrite(BLUE,HIGH);//Blue
    delay (2000);   
    for(int i=0; i<Samples; i++){
      y1a  += analogRead(TSL257);
    }
    digitalWrite(BLUE,LOW);      
    
    y1a /=  Samples;  // avg Read
    y1aV /=  Samples;

    
    Serial.print("GREEN Avg x1a ");
    Serial.print (x1a);

    Serial.print (" BLUE Avg y1a ");
    Serial.println (y1a);

    Serial.print("GBAVG ");

    Serial.print(x1a);
    Serial.print(" ");
    Serial.println(y1a);
    
    double  BluGrnRatio = (double) y1a/ (double) x1a;  // 2355
    Serial.print ("Blue/Green Ratio: ");
    Serial.println (BluGrnRatio);
    Serial.println("Blank Process End\n-----");
    
    
    // LCD Display
    
    lcd.clear(); //clear
    lcd.print("BlankGrn ");
    lcd.print(x1a);
    //lcd.print(" ");
    //lcd.print(x1aV);
    lcd.setCursor(0, 1) ;
    lcd.print("BlankBlu ");
    lcd.print(y1a);
}
// ---------------------end blankProcess-----------------------

void readProcess () 
{
    lcd.clear(); //clear
    lcd.print("Read:green");
    
    Serial.println("\nRead Process Begin\nRead:green");
    x2a = 0;
    x2aV = 0;
    
    digitalWrite(GREEN,HIGH);
    delay(2000);
    for(int i=0; i<Samples; i++){
      x2a += analogRead(TSL257);
    }
    digitalWrite(GREEN,LOW);

    x2a/= Samples; // green
    
    lcd.clear();
    lcd.print("Read:blue");
    Serial.println("Read:blue");
    y2a = 0;
    y2aV =0;
    
    digitalWrite(BLUE,HIGH);
    delay (2000);       
    for(int i=0; i<Samples; i++){
      y2a += analogRead(TSL257);  //look capture ar and monitor
    }
    digitalWrite(BLUE,LOW);
    
    y2a /= Samples; // blue
    y2aV /= Samples;

    A_1 = log((float)x1a/(float)x2a)/(log(10));//calculate the absorbance  green
    A_2 = log((float)y1a/(float)y2a)/(log(10));//calculate the absorbance  blue

    RB = A_1 / A_2;
 
    // -------AVERAGES 2.3.3.2 23jun17 ------
    Serial.print("Green Read Average ar and v : ");
    Serial.println(x2a);

    Serial.print("Blue Read Average  ar and v : ");
    Serial.println(y2a);
    
    Serial.print("GBAVG ");
    Serial.print(x2a);
    Serial.print(" ");
    Serial.println(y2a);
    Serial.println(" ");
    

    Serial.print ("GREEN absp A_1: ");
    Serial.println (A_1);
    Serial.print ("BLUE absp A_2: ");
    Serial.println (A_2);
 
    
    Serial.print("RB Absoption Ratio (A_1/A_2): ");  // Absb ration  of green/blue
    Serial.println(RB); 

/*
    // may 3 2017 calibration
    double slope = 0.456526118;
    double intercept = 0.490052692;
    double r2 = 0.995939;
    
    // use the b yang calibration !! LOOK 
    slope = 1.1892;
    intercept = -0.3079;
*/

    // v4 -- Callibration 6/29/17 -----  
    double slope  = 1.4638237329;
    double intercept = -0.5741381289;
    double r2 = 0.9974126932;
    // LOOK reverse intercept polarity
    // LOOK use Bo Yang see what happens  31octy17  testing
    //slope = 1.1892;
    //intercept = -0.3079;
    
    double RN = slope * RB + intercept;  // RN correction default from EQ 8 B Yang

    // double pH, equConstant,R, e1,e2,e3, e23; // WHOA ERROR --R!
    double pH, equConstant, e1,e2,e3, e23;
    double  T, S, Tc ;  // temp and salinity
    // set default values for S and T
    // for salinity S values from 20 to 40
    // for temperatures T from 278.15 K to 308.15 K
    S =  33.3;
    S = 34.8;

    Tc = 27.7; // pH is .03 units higher than at 25
    Tc = 25.0;      
    
    T = 273.15 + Tc;
    //T =  298.2; // 25C
    // eq 5
    e1 = -0.007762 + (4.5174 / pow(10.0,5.0)) * T;
    e23 = -0.020813 + (2.60262 / pow (10.0,4.0))*T + (1.0436 / pow (10.0,4.0)) * (S-35) ;

    // eq 7   equilibrium constant and components a,b,c,d  equConstant
    double a,b,c,d;

    a = -246.64209 + 0.315971 * S + (2.8855/pow (10.0,4.0)) * S * S ;
    b= 7229.23864 - 7.098137* S - 0.057034 * S * S;
    c = 44.493382-0.052711 * S;
    d = 0.0781344;
    equConstant = a + b / T + c * log(T) - d*T;  // log is the natural log ln

    pH = equConstant +  log10 (( RN - e1)/(1-RN*e23) );  // assume the COMMON LOG

    //lcd.clear();
    //lcd.setCursor(0,0);
    //lcd.print("RB ");  // Absb ration  of green/blue
    //lcd.print(RB,2); 
    //lcd.print(" RN ");
    //lcd.print(RN,2);
        
    //lcd.setCursor(0,1);
    //lcd.print("PH ");
    //lcd.print(pH,3);
    // LOOK New code 2382 31oct17
    lcd.clear();
    lcd.setCursor(0,0); // absorption
    lcd.print("G "); lcd.print(A_1,3); lcd.print(" B ");lcd.print(A_2,3);
    lcd.setCursor(0,1);
    lcd.print("RB ");lcd.print(RB,4); //lcd.print(" pH ");lcd.print(pH,3);

    Serial.print("x1a ");
    Serial.println(x1a);
    Serial.print("x2a ");
    Serial.println(x2a);
    Serial.print("y1a ");
    Serial.println(y1a);
    Serial.print("y2a ");
    Serial.println(y2a);

    Serial.print("RB ");
    Serial.println(RB);
    char buf[25];
    char fpstr[12];
    dtostrf(RB,7,3,fpstr);
    sprintf(buf,"test RB %s",fpstr);
    Serial.println(buf);

    Serial.print("Absorption G ");
    Serial.print(A_1,3);  // green absb
    Serial.print("(");
    Serial.print(x2a);
    Serial.println(")");
    // Serial.setCursor(0, 1) ;
    Serial.print("Absorption B ");
    Serial.print(A_2,3);  // blue absb
    Serial.print("(");
    Serial.print(y2a);
    Serial.println(")\n");
    // delay (2000);
    // Serial.clear();
    
    Serial.print("Temp K: ");
    Serial.println(T);
    Serial.print("Salinity: ");
    Serial.println(S);
    Serial.print("Absorption Ratio RB: ");  // Absb ration  of green/blue
    Serial.println(RB,3); 
    Serial.print("Estimated RN:        ");
    Serial.println(RN,3);
    Serial.print("PH: ");
    Serial.println(pH,3);
    Serial.println(" "); // needed to clear serial buffer 
}
// ---------------------end readProcess-----------------------


// LOOK
char incomingChar = 'S';    // for incoming serial data

void loop ()
{
  double  T, S, Tk, Tc ;  // temp and salinity   
    // set default values for S and T
    // for salinity S values from 20 to 40
    // for temperatures T from 278.15 K to 308.15 K
  S =  33.3;
  S = 34.8;
  Tc = 27.7; // pH is .03 units higher than at 25
  Tc = 25.0;      
  Tk = 273.15 + Tc;
  T = Tk;
 
  // LOOK serial listener 
  if (Serial.available() > 0) {
     incomingChar = Serial.read(); // char at a time
     Serial.print("krph received char: "); 
     Serial.println(incomingChar);
  } 
  
  if (incomingChar == 'S') {
     Serial.println("------- krph Start -----");
     incomingChar = 'A';
  }
     
  if (incomingChar == 'b')
  {
    
    blankProcess();
    incomingChar = 'x';
    
  }
  if (incomingChar == 'B')
  {
     // restore previous Blank
     x1a = blGreen;
     y1a = blBlue;
  }
  if (incomingChar == 'r')
  {
    readProcess();
    incomingChar = 'y';
  }
 
  if (digitalRead(BlankButton) == HIGH)
  {
    blankProcess();
    incomingChar = 'z';
  }  // Blank Button End

  if (digitalRead(ReadButton) == HIGH) 
  {
    readProcess();
    incomingChar ='Z';
  } 

} // end Loop



