#define Rx 17
#define Tx 16
#include <Servo.h>
#include "Adafruit_TCS34725.h"


int pins[] = {47, 49, 51, 52}; //define 4 QTI sensor pins
bool lineReads[4];             //stores [1] for dark, [0] for light per sensor           
bool mirrorReads[4];           //stores [1] for mirror, [0] for no mirror per sensor
long durations[4];             //stores decay duration per QTI sensor
int lineThreshold = 200;       //if decay duration > lineThreshold, read dark
int mirrorThreshold = 75;      //if decay duration < mirrorThreshold, read mirror
int tick = 50;                 //delay between main loop iterations

boolean ownState;              //result of mirror sense task
boolean received[5];           //stores if values have been received from each of the five other bots
boolean info[5];               //stores values received from each of the five other bots

Servo servoLeft;
Servo servoRight;

//color sensor init
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X); 

/*
 * Arrays to store mean ratios and allowed variations between R,G,B readings when over game colors.
 * Ratios stored are r/g, g/b, and r/b, each with six color readings:
 * GREEN, GREY, BLUE, RED, YELLOW, PURPLE
 * These were determined experimentally. "Tolerances" are the maximum deviation from the mean
 * recorded for each R/G/B ratio, for each color
 */
double rg_vals[] = {0.8636380641, 1.284659139, 0.969768167, 4.619912228, 1.381592637, 2.388246845};
//double rg_tol[] = {0.01984019675, 0.1147742883, 0.03023183303, 1.08910654, 0.04060277903, 0.1813502929};
double rg_tol[] = {0.04, 0.22, 0.06, 2.0, 0.08, 0.36};  //EMERGENCY DOUBLE TOLERANCE
double gb_vals[] = {2.380208231, 1.43850934, 0.9406748502, 1.171949175, 2.595074488, 0.992109592};
//double gb_tol[] = {0.08937157732, 0.01416555669, 0.09651523245, 0.08400320598, 0.1487077059, 0.03900151909};
double gb_tol[] = {0.18, 0.028, 0.2, 0.16, 0.3, 0.08}; //EMERGENCY DOUBLE TOLERANCE
double rb_vals[] = {2.055548837, 1.848069159, 0.9122364791, 5.407304038, 3.585255313, 2.368465519};
double rb_tol[] = {0.13, 0.36, 0.17, 2.0, 0.47, 0.18}; //EMERGENCY DOUBLE TOLERANCE

void setup() {
  Serial.begin(9600);  //serial monitor
  Serial2.begin(9600); //XBee
  Serial3.begin(9600); //LCD
  pinMode(4, OUTPUT); //Orange LED
  pinMode(44, OUTPUT); //RGB LED (blue)
  pinMode(45, OUTPUT); //RGB LED (red)
  pinMode(46, OUTPUT); //RGB LED (green)
  servoLeft.attach(12);
  servoRight.attach(11);

  //clear LCD
  for (int i = 0; i <= 62; i++) {
    Serial3.print(" ");
  }
  Serial3.println("");

  //init color sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); //halt program if failure
  }
}

void loop() {
  checkSensors();

  //sensor debug
  for(int i = 0; i < sizeof(pins)/2; i++) {
  /*  Serial.print("#");
    Serial.print(pins[i]);
    Serial.print(" - ");
    Serial.print(lineReads[i]);
    Serial.print(" (");
    Serial.print(durations[i]);
    Serial.println(")");*/
  }
  
  navigate();
  delay(tick);
  clearSensors();
}


/*
 * Called by loop() first.
 * Updates the "line" and "mirror" readings at each
 * QTI sensor through the global arrays.
 * Pings 1ms QTI sensor for 1ms, to pulse infrared
 * as well as charge each capacitor, then measures  
 * how long it takes for capacitor to drain through
 * their photoresistor. This value determines local
 * reflectivity, mapped to durations[].
 */
void checkSensors() {
  for(int i = 0; i < sizeof(pins)/2; i++) {        //for each QTI sensor
    long duration = 0;         
    pinMode(pins[i], OUTPUT);                      //ping QTI sensor
    digitalWrite(pins[i], HIGH); 
    delay(1);                                      //wait for capacitor to charge
    pinMode(pins[i], INPUT);                       //begin draining capacitor
    digitalWrite(pins[i], LOW);
    long dt = micros();                 
    while(digitalRead(pins[i])){                   //measure time until drain voltage < 2.1V
      duration += micros() - dt;
      dt = micros();
    }                                      
    durations[i] = duration;                       //store durations in global array
    lineReads[i] = (duration > lineThreshold);     //update other arrays to indicate what each duration means 
    mirrorReads[i] = (duration < mirrorThreshold);
  }
}

/*
 * Called at end of loop().
 * Likely not the most efficient way to do it, but we erase
 * stored durations and associated readings after each tick.
 */
void clearSensors() {
  for (int i=0; i<3; i++) {
    durations[i] = 0;
    lineReads[i] = 0;
    mirrorReads[i] = 0;
  }
}

/*
 * Called from loop().
 * Uses stored values of QTI sensors to determine the position
 * of the line underneath the bot.
 * Cases are "stop", if it reads the hash,
 * "swivel" left and right, for quickly correcting along a curved line,
 * "corner" left and right, for turning 90deg on a sharp corner,
 * and "go", when parallel with line, or not reading line.
 * 
 * Does all game fuctions once it reaches the hash in "stop" case.
 */
void navigate() {
  if (lineReads[0] && lineReads[1] && lineReads[2]) {
    //Serial.println("stop case: do colorCheck");
    drive(0, 0, 500);
    colorCheck();
    mirrorCheck();
    transmit();
    drive(65, 70, 2*tick);
  }else if(lineReads[0] && !lineReads[1] && !lineReads[2]) {
    //Serial.println("swivel left case");
    drive(-100, 100, 100);
    drive(65, 70, tick);
  /*}else if(lineReads[0] && lineReads[1] && !lineReads[2]) {
    Serial.println("corner left case"); //corner cases not calibrated
    drive(0, 100, 500);*/
  }else if(!lineReads[0] && !lineReads[1] && lineReads[2]) {
    //Serial.println("swivel right case");
    drive(100, -100, 100);
    drive(65, 70, tick);
  /*}else if(!lineReads[0] && lineReads[1] && lineReads[2]) {
    Serial.println("corner right case"); //corner cases not calibrated
    drive(100, 0, 500);*/
  }else if(!lineReads[0] && !lineReads[1] && !lineReads[2]) {
    //Serial.println("go case (no line)");
    drive(65, 70, tick);
  }else if(!lineReads[0] && lineReads[1] && !lineReads[2]) {
    //Serial.println("go case (line)");
    drive(65, 70, tick);
  }
}

/*
 * Helper method for matching servo speed to pulse width,
 * and adding a delay to each "drive" command.
 */
void drive(int leftSpd, int rightSpd, int duration) {
  servoLeft.writeMicroseconds(1500 + leftSpd);
  servoRight.writeMicroseconds(1500 - rightSpd);
  delay(duration);
}


/*
 * Called from navigate() during "stop" case.
 * checks value stored for outermost QTI sensor
 */
boolean mirrorCheck() {
  if (mirrorReads[3]) {
    digitalWrite(4, HIGH);
    ownState = true;
    Serial.println("mirror detected");
  }else{
    digitalWrite(4, LOW);
    ownState = false;
    Serial.println("mirror not detected");
  }
}

/*
 * Called from navigate() during "stop" case.
 * Gets raw RGB from the color sensor and prints,
 * Calculates color ratios,
 * Checks for matching colors,
 * and writes determined color to LED.
 */
void colorCheck() {
  //define and get raw RGB vals
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  //print raw vals (for calibrating ratios)
  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  //Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  //any one, and only one, color condition is satisfied if
  //each color ratio cr is within cr_tol[color] of the mean cr_vals[color]
  //if more than one is satisfied, priority is given to first in order:
  //0 GREEN, 1 GREY, 2 BLUE, 3 RED, 4 YELLOW, 5 PURPLE, 6 ERROR
  //ERROR case 6 is default, if no colors match
  double rg = (double)r / (double)g;
  double gb = (double)g / (double)b;
  double rb = (double)r / (double)b;
  int value = 6;
  for (int i=0; i<6; i++){
    if((rg > rg_vals[i] - rg_tol[i] && rg < rg_vals[i] + rg_tol[i]) &&
       (gb > gb_vals[i] - gb_tol[i] && gb < gb_vals[i] + gb_tol[i]) &&
       (rb > rb_vals[i] - rb_tol[i] && rb < rb_vals[i] + rb_tol[i])) {
       value = i;
       break;
    }
  }
  switch (value) {
    case 0:
      Serial.println("GREEN CASE");
      analogWrite(45, 255);
      analogWrite(46, 0);
      analogWrite(44, 255);
      break;
    case 1:
      Serial.println("GREY CASE");
      analogWrite(45, 128);
      analogWrite(46, 128);
      analogWrite(44, 128);
      break;
    case 2:
      Serial.println("BLUE CASE");
      analogWrite(45, 255);
      analogWrite(46, 255);
      analogWrite(44, 0);
      break;
    case 3:
      Serial.println("RED CASE");
      analogWrite(45, 128);
      analogWrite(46, 255);
      analogWrite(44, 255);
      break;
    case 4:
      Serial.println("YELLOW CASE");
      analogWrite(45, 0);
      analogWrite(46, 0);
      analogWrite(44, 255);
      break;
    case 5:
      Serial.println("PURPLE CASE");
      analogWrite(45, 0);
      analogWrite(46, 255);
      analogWrite(44, 0);
      break;
    case 6:
      Serial.println("ERROR CASE");
      analogWrite(45, 255);
      analogWrite(46, 255);
      analogWrite(44, 255);
      break;
  }
}

/*
 * Called by navigate() during "stop" case.
 * A long script to coordinate a centralized computation of the
 * IDC game objective.
 * 
 */
void transmit() {
  unsigned long startT = micros();
  boolean timeout = false;

  //Listen for data from other bots while missing at least one bot's data,
  //or after time expires.
  while (!isFull() && !timeout) {

    //if there is no data incoming to XBee, wait and increment timer.
    while (!Serial2.available() && !timeout) {
      if (micros() - startT > 20000000) timeout = true;
    }

    //get character from XBee. Each bot is assigned a letter A-E to transmit;
    //uppercase indicates 0, lowercase indicates 1.
    char ch = Serial2.read();
    Serial.print("read character is ");
    Serial.println(ch);

    if (ch < 91) {            //if character is uppercase,
      if (!received[ch-65]) { //and we haven't received this letter before,
        info[ch-65] = 0;      //update that character's bot's data to 0
        received[ch-65] = 1;  //and remember we have received it.
      }
    }else{                    //if character is lowercase,
      if (!received[ch-97]) { //and we haven't received this letter before,
        info[ch-97] = 1;      //update that character's bot's data to 1
        received[ch-97] = 1;  //and remember we have received it.
      }
    }
  }
  //Once all 5 characters are received, or after timeout:
  //calculate the sum of all bot's status (including own game status)
  int sum = 0;
  for (int i=0; i<5; i++) {
    Serial.print("info[");
    Serial.print(i);
    Serial.print("] is ");
    Serial.println(info[i]);
    sum += info[i];
  }
  Serial.print("ownState is ");
  Serial.println(ownState);
  
  if (ownState) sum += 1;
  Serial.print("sum is ");
  Serial.println(sum);

  //in case of timeout, calculates number of bots from whom we missed data.
  //default value for info[] is 0, but this may indicate whether sum is accurate.
  int missing = 0;
  for (int i=0; i<5; i++) {
    if (!received[i]) missing++;
  }
  if (missing > 0) {
    Serial.print("(but we missed ");
    Serial.print(missing);
    Serial.println(" values!)");
  }

  //'Z' indicates to other bots that it is time to receive sum
  //Both 'Z' and sum are printed multiple times to mitigate failure.
  int j = 0;
  while (j < 10) {
    Serial2.print('Z');
    j++;
  }
  j = 0;
  delay(100);
  while (j < 10) {
    Serial2.print(sum);
    j++;
  }
  
  //output the relevant information to LCD
  printLCD(sum, ownState);
  
  //backup to halt execution, just in case printLCD() fails
  //we don't want to do all this twice.
  while (true);
}

/*
 * Called by transmit().
 * Determines if we have received data from all bots.
 */
boolean isFull() {
  for (int i=0; i<5; i++) {
    if (!received[i]) return false;
  }
  return true;
}

/*
 * Called by transmit() at the end.
 * Takes the sum of all bots' status, 0-6,
 * translates it to the Next Grand Challenge,
 * and outputs this to the LCD.
 */
void printLCD(int total, int senseTask) {
  String nextChal = "BAD VAL";
  if(total == 0){
    nextChal = "Manage Nitrogen";
  } else if (total == 1){
    nextChal = "Advance HealthInfo";
  } else if (total == 2){
    nextChal = "Engr Medicine";
  } else if (total == 3){
    nextChal = "Enhance VR";
  } else if (total == 4){
    nextChal = "Engr Scientific Disc";
  } else if (total == 5){
    nextChal = "Rev-Engr Brain";
  } else { //total is 6
    nextChal = "Adv Personal Learning";
  }

  //first prints sum and our own task state,
  //then prints corresponding Grand Challenge
  //cycling every 2s
  //forever.
  while(true){
    for (int i = 0; i <= 62; i++) {  //clears display
      Serial3.print(" ");
    }
    Serial3.println("");
    Serial3.print("The total is: ");
    Serial3.print(total);
    Serial3.print(" ");
    Serial3.print("Sensing is: ");
    Serial3.print(senseTask);
    delay(2000);
    for (int i = 0; i <= 62; i++) {
      Serial3.print(" ");
    }
    Serial3.println("");
    Serial3.print(nextChal);
    delay(2000);
  }
}
