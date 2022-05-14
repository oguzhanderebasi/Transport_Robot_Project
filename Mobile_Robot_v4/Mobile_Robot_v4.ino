/*------------ Libraries ------------*/
#include <math.h>

//RFID and SPI communication lib
#include <SPI.h>
#include <RFID.h>

//Arduino to NodeMCU lib
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

/*------------ DEFINITIONS FOR RFID AND SERIAL ------------*/
//Initialise Arduino to NodeMCU (10=Rx & 11=Tx)
SoftwareSerial arduino_com(10, 11);

//Initialise RFID-Reader
RFID lrt720(53, 4);

/*------------ GLOBALS AND DEFINITIONS FOR RFID ------------*/
int s0; int s1;
int s2; int s3;
int s4;
String tag_data;
String tag_data_old = "";

String station_number;
String gridNom_nodemcu;

int station;
int gridNom_nodemcu_int;

bool stationCheck = true; // Actual Value: false
bool tagCheck = true; // Actual Value: false

char c;
String nodemcu_data_string;
int8_t indexOfA, indexOfB;

StaticJsonBuffer<1000> jsonBuffer2;
JsonObject& data = jsonBuffer2.createObject();
StaticJsonBuffer<1000> jb;
JsonObject& root = jb.createObject();

/*------------ GLOBALS AND DEFINITIONS FOR A* ALGORITHM ------------*/
// Define map size
#define row 6
#define col 6

int Echo = 42;  
int Trig = 43;

float duration, distance; 
int obstacle;

byte goalN = 8; // goal position on grid
byte openList[50]; // contains all the possible paths
byte closedList[50]; // contains the path taken
byte Path[50];
byte oLN=0, cLN=0;// the counters for the openList and closedList
byte curBotPos = 0 ; // holds current bot position
byte curBotDir = 1 ; // holds current bot facing direction (1 up  2 down 3 left  4 right)
byte curBotPos2;
byte curBotDir2;

/*------------ A* Structes ------------*/
struct Node
{
  byte g, h, f;
  byte parent;
  byte index;
  byte gridNom;
};

struct Grid
{
  Node Map[row][col];
} PF ;

/*------------ A* Basic Functions ------------*/
// Returns the number of gride have been traverd
byte G(byte curR, byte curC)
{
  byte gValue, parInd;
  byte rowg, colg;
  parInd = PF.Map[curR][curC].parent;
 
  rowg = (byte)parInd/6;
  colg = parInd%6;
  gValue = PF.Map[rowg][colg].g;
  
  return (gValue+1);
}

// Manhattan distance heauristics function
byte H(byte curR, byte curC, byte goalS)
{
 byte rowg, colg;
 byte manhattan=0;

 
   rowg = (byte)goalS/6;
   colg = goalS%6;
   manhattan += (abs(curR - rowg) + abs(curC - colg));
   
  return manhattan;
}

// the total "cost" of the path taken; adds H and G values for each tile f(n) = g(n) + h(n)
byte FV(byte curG, byte curH) 
{
 byte fValue; 
  
  fValue = curG + curH;
  return fValue;
}

/*------------ PID CLASS ------------*/
class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;
    
  public:
    // Default initialization list (Construction)
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    
    // Set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Evaluate the signal
    void evalu(int value, int target, float deltaT,int &pwr, int &dir){
        
      // error
      int e = target - value;

      // derivative
      float dedt = (e-eprev)/(deltaT);

      // integral
      eintegral = eintegral + e*deltaT;

      // control signal
      float u = kp*e + kd*dedt + ki*eintegral;
    
      // motor power delimitation
      pwr = (int) fabs(u);
      if( pwr > umax ){
        pwr = umax;
      }
           
      // motor direction
      dir = 1;
      if(u<0){
        dir = -1;
      }
            
      // store previous error
      eprev = e;
    }
    
};

/*------------ GLOBALS AND DEFINITIONS FOR PID ------------*/

// Define the motors
#define NMOTORS 2
#define M0 0
#define M1 1

const int enca[] = {21,20};
const int encb[] = {26,27};
const int pwm[] = {3,9};
const int in1[] = {5,7};
const int in2[] = {6,8};

// Global variables
long prevT = 0;
int posPrev[] = {0,0};
int counter;
bool posTrue = false;

// positions
volatile int posi[] = {0,0};
int pos[NMOTORS];

// PID classes
SimplePID pid[NMOTORS];

/*------------ PID Functions ------------*/
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}

// targets
float target_f[] = {0,0};
long target[] = {0,0};

void settarget(int curRobDir){
  float positionChange[2] = {0.0,0.0};
  float pulsesPerTurn = (11*34)+20; // (counts per revolution / 4) * (Metal gearbox)
  float pulsesPerSixtyCm = pulsesPerTurn*1.5915;
  float velocity = 1; // m/s
  
  //positionChange[0] = pulsesPerTurn;
  //positionChange[1] = pulsesPerTurn;
  
  if (curRobDir == 1) { //up
    target[0] = -(pulsesPerTurn+150);
    target[1] = pulsesPerTurn+150;
    Serial.println("Forward");
    Serial.println("=============");
    Serial.print("Target0 for dir=1 = ");
    Serial.println(target[0]);
    Serial.println("=============");
  }
  else if (curRobDir == 3){ //left
    target[0] = -pulsesPerTurn; // pulsesPerTurn
    target[1] = -pulsesPerTurn;
    Serial.println("Left");
    Serial.println("=============");
    Serial.print("Target0 for dir=3 = ");
    Serial.println(target[0]);
    Serial.println("=============");
  }
  else if (curRobDir == 4){ //right
    target[0] = pulsesPerTurn;
    target[1] = pulsesPerTurn; // -pulsesPerTurn
    Serial.println("Right");
    Serial.println("=============");
    Serial.print("Target0 for dir=4 = ");
    Serial.println(target[0]);
    Serial.println("=============");
  }
}

void pidFunc(){
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
  
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
    pos[k] = posi[k];
  }
  interrupts(); // turn interrupts back on
  
  // Loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir); // compute the position
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]); // signal the motor
  }

  long target_value = target[0];
  long pos_value = (long) pos[0];

  for(int i = 0; i<2; i++){
    Serial.print(target[i]);
    Serial.print(" ");
    counter += 1;
  }
  for(int i = 0; i<2; i++){
    Serial.print(posi[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("counter: ");
  Serial.println(counter);
  
  
  if( target_value != 0 && pos_value != 0 ){
    //counter++;
    //Serial.print("counter: ");
    //Serial.println(counter);
    //if( (target_value+6 > pos_value && pos_value > target_value-6) == true ){
    if( counter == 1900 ){
      posTrue = false;
      currT = 0;
      delay(500);
      stop();
      delay(900);
      counter = 0;
    }
    //}
  }
}


/*------------ SETUP ------------*/
void setup() {
  Serial.begin(9600);
  
  // Open serial communications for RFID and wait for port to open:
  Serial.begin(115200);
  SPI.begin();
  lrt720.init();
  arduino_com.begin(9600);
  
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  
  // PID Configurations of the Motors
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
    pid[k].setParams(0.75,0.053,0,60); //0.75,0.053,0,60
  }

  // Encoder interrups of the Motors
  attachInterrupt(digitalPinToInterrupt(enca[M0]),readEncoder<M0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]),readEncoder<M1>,RISING);

  // A* Map Setup
  buildMap();
  printGrid1();
  printGrid2();
  
}

void loop() {
  rfid_get_curBotPos_and_goal(); // new
  
  if (stationCheck && tagCheck && station != -1){
    if (!isGoal(curBotPos) && OLE)
    {
      _loop();                                      // the actual performance of the A* algorithm
    }
    
    else if (isGoal(curBotPos))
    {
      
      PathList();                                   // List the optimal path
      
      Serial.println("Path[i]");
      for(byte i=0;i<PF.Map[closedList[cLN-1]/6][closedList[cLN-1]%6].g;i++) {
      Serial.println(Path[i]);
      }
      delay(300);
      while (1){
        //curBotPos = (byte) gridNom_nodemcu_int;
        movement(curBotPos,curBotDir);
        curBotPos = curBotPos2;
        curBotDir = curBotDir2;
          
        if (!isGoal(curBotPos)){
          break;
        }
        
        Serial.println("Goal Reached");
        stationCheck = false;
        goalN = -1;  // burada kaldık sorun var
        delay(3000000);
        Serial.println("Define new target");
       }   
    }
  }
}

/*------------ RFID Functions ------------*/
void rfid_get_curBotPos_and_goal(){
  if(arduino_com.available() == 0 ){
    if (lrt720.isCard()){
      if (lrt720.readCardSerial()){
        s0 = lrt720.serNum[0];
        s1 = lrt720.serNum[1];
        s2 = lrt720.serNum[2];
        s3 = lrt720.serNum[3];
        s4 = lrt720.serNum[4];
        
        tag_data = String(s0) + "," + String(s1) + "," + String(s2) + "," + String(s3)
                  + "," + String(s4);
        
        if(!(tag_data_old.equals(tag_data))){
          delay(200);
          Serial.println("Card ID : ");
          root["tag_data"] = tag_data;
          Serial.println(tag_data);
          root.printTo(arduino_com);
          //nodemcu.println(tag_data);
          Serial.println("================================================");
          delay(100);
          tagCheck = true;
          tag_data_old = tag_data;
        }
      }
    }
  }
  
  while ( arduino_com.available() > 0){
    c = arduino_com.read();
    
    if(c=='\n') {break;}
    else        {nodemcu_data_string+=c;}    
  }
  
  if(c=='\n'){
    ParseData();
    stationCheck = true;

    station = station_number.toInt();
    gridNom_nodemcu_int = gridNom_nodemcu.toInt();
    
    goalN = (byte) station;
    curBotPos = (byte) gridNom_nodemcu_int;
    
    Serial.print("Gridnom= " +  gridNom_nodemcu);
    Serial.println(", Station= " +  station_number);
    Serial.println("================================================");
    c=0;
    nodemcu_data_string="";
  }  
}

void ParseData(){
  indexOfA = nodemcu_data_string.indexOf("A");
  indexOfB = nodemcu_data_string.indexOf("B");
  
  gridNom_nodemcu = nodemcu_data_string.substring (0, indexOfA);
  station_number = nodemcu_data_string.substring (indexOfA+1, indexOfB);
}

/*------------ A* Functions ------------*/
// Performs the A* algorithm, "main" program
void _loop(){                 
  
  possMov(curBotPos);
  
  AddClosedList();
  
  printGrid2();
  
}

// Builds the 6x6 map grid
void buildMap()
{
  byte gridIn = 0;
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    PF.Map[i][j].gridNom = gridIn;
    PF.Map[i][j].index = 0;
    PF.Map[i][j].parent = 0;
    PF.Map[i][j].h = 0;
    PF.Map[i][j].g = 0;
    PF.Map[i][j].f = 0;
    
    gridIn++;    
   }
  }
}

// Prints the grid, using indices 0 to 35 to represent the possible paths
void printGrid1()
{
  for (byte i = 0; i < row; i++)
  {
    for (byte j = 0; j < col; j++)
    {
      Serial.print(PF.Map[i][j].gridNom);
      if (j != row-1)
      {
        if (PF.Map[i][j].gridNom < row-1)
        {
        Serial.print("  | ");
        }
        else
        Serial.print(" | ");
      }
    }
  
    if (i != row-1)
    {
      Serial.println();
      Serial.print("----------------------------");
      Serial.println();
    }
  }
  Serial.println();
  Serial.println();
  delay(300);
}

// Prints the grid, 0 - untravelled | 1 - travelled | 2 - obstacles | 3 - goal
void printGrid2() 
{
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    Serial.print(PF.Map[i][j].index);
    if (j != row-1)
    {
    Serial.print(" | ");
    } 
   }
  
  if (i != row-1)
    {
      Serial.println();
      Serial.print("----------------------");
      Serial.println();
    }
  }
  Serial.println();
  Serial.println();
  delay(300);
}

// Asks user for input to set the goal state/tile
void setGoal()
{
  byte goal;
  Serial.println("Where do you want to place your goal state?");
  Serial.println("Using the numbers displayed in the earlier grid, enter a number to intialize as your goal state.");
  Serial.println();

  while (!Serial.available() )
  {
     goal = Serial.parseInt();
  }
 
  for (byte i = 0; i < row; i++)
  {
    for (byte k = 0; k < col; k++)
    {
      if (PF.Map[i][k].gridNom == goal)
      {
        
        PF.Map[i][k].index = 3;
        goalN = PF.Map[i][k].gridNom;
      }
      else if (PF.Map[i][k].gridNom == 0)  // initial start point 
      {
        PF.Map[i][k].index = 1;
        curBotPos = PF.Map[i][k].gridNom;
      }
      else if (PF.Map[i][k].gridNom == 3 || PF.Map[i][k].gridNom == 14 || PF.Map[i][k].gridNom == 16 || PF.Map[i][k].gridNom == 18 || PF.Map[i][k].gridNom == 27 || PF.Map[i][k].gridNom == 29 || PF.Map[i][k].gridNom == 31)
      {
        PF.Map[i][k].index = 2;        // initial wall
      }
      else
      PF.Map[i][k].index = 0;          // initial free space
    }
  }
  printGrid2();
}

// Checks the possible moves depending on the location of the current tile the bot is on
void possMov(byte gridNom)
{
  byte rowp = (byte) gridNom / 6;
  byte colp = gridNom % 6;
  if (gridNom == 0) // checks the corner tiles | 2 possible moves
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom == 5)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom == 30)
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    }
  }
  else if (gridNom == 35)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 6);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 1);
    }   
  }
  else if (gridNom > 0 && gridNom < 5) // checks the tiles on the outermost edges of the map | 3 possible moves
  {
   if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }    
  }
  else if (gridNom%6==0)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom%6==5)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    } 
    if (PF.Map[rowp][colp- 1].index != 1 && PF.Map[rowp][colp- 1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
    }
  }
  else if (gridNom > 30 && gridNom < 35)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    } 
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
  }
  else { // checks the remaining tiles | 4 possible moves
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 6);
    }
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
  }
     if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 6);
  }
}

}

// Adds the potential possible moves to the openList
void AddOpenList(byte aol)
{
  
  openList[oLN++] = aol;
  heuristics(aol);
}

// Calculates the "cost" of the tile
void heuristics(byte curIn)
{
  byte hH, gH, fH;
  byte rowh = (byte) curIn / 6;
  byte colh = curIn % 6;

  hH = H(rowh, colh, goalN);
  PF.Map[rowh][colh].h = hH;
  gH = G(rowh, colh);
  PF.Map[rowh][colh].g = gH;
  fH = FV(hH,gH);
  PF.Map[rowh][colh].f = fH;
}

// Returns the best heuristics value restricted by the current path the bot is taking
byte getNextFI() 
{
  byte rowf;
  byte colf;
  byte lowestF;
  byte lowest = openList[0];
  rowf = (byte) lowest / 6;
  colf = lowest % 6;
  lowestF = PF.Map[rowf][colf].f;
  
  for (byte i = 0; i < oLN; i++)
  {
    rowf = (byte) openList[i] / 6;
    colf = openList[i] % 6;
    
    if (PF.Map[rowf][colf].f <= lowestF) 
    {
      lowestF = PF.Map[rowf][colf].f;
      lowest = rowf*6 + colf;
    }
  }
  
  return lowest;
}

// Adds the "best" tile to the closedList
void AddClosedList()
{
  byte low = getNextFI(); 
  byte rowa, cola;

  closedList[cLN++] = low;
  rowa = (byte)low/6;
  cola = low%6;
  PF.Map[rowa][cola].index = 1;
  curBotPos = low;
  removeFOL(low); 
}

// Returns the best heuristics value restricted by the current path the bot is taking
void PathList()  // List the optimal path
{
    for(byte i=1; i < PF.Map[closedList[cLN-1]/6][closedList[cLN-1]%6].g + 1; i++){
      for(byte j=0; j < cLN; j++){
        if(PF.Map[closedList[j]/6][closedList[j]%6].g == i){
          Path[i-1]=closedList[j];
        }
      }
    }
}

// Removes previous potential paths from the openList, in order to get the "best" current path
void removeFOL(byte rfol)
{

  for (byte i = 0; i < oLN-30; i++)
  {
    if (openList[i] == rfol)
    {
      openList[i] = openList[i+1];
    }
    else
      openList[i] = openList[i+1];
  }
    oLN=oLN-1;
}

// Checks if the openList is empty
bool OLE()
{
  if (oLN == 0)
  {
    return true;
  }
  else
  return false;
}

// Checks if the goal has been reached
bool isGoal(byte ig)
{
  if (ig == goalN)
  {
    return true; 
  }
  else
  return false;
}

// Checks if the tile is already on the openList
bool alreadyOnOL(byte rowaol, byte colaol)
{
  byte indexol;
  bool on = false;

  indexol = rowaol*6 + colaol;
  for (byte i = 0; i < oLN; i++)
  {
    if (openList[i] == indexol)
    {
      on = true;
    }
  }
  
  return on;
}

byte movement(byte curBotPos,byte curBotDir) {
  
  curBotPos = PF.Map[Path[0]/6][Path[0]%6].parent;
  Serial.print("curBotPos_beforemovement:  ");
  Serial.println (curBotPos);
  Serial.print("curBotDir_beforemovement:  ");
  Serial.println (curBotDir);
  
  byte rowm, colm, parm;
  byte i = 0;

  while(!isGoal(curBotPos)){
     
      rowm = Path[i]/6;
      colm = Path[i]%6;
      
      posReset();
      
      if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 1){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        settarget(1); //forward
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 1){
        settarget(3); //left();
        pidMove();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        
        posReset();
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 1){
        settarget(4); //right();
        pidMove();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }

        posReset();
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 1){
        settarget(4); // right
        pidMove();
        posReset();   // reset the position
        settarget(4); // right
        pidMove();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;

      }

      else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 2){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 2){
        settarget(4); //right();
        pidMove();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;

      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 2){
        settarget(3); //left();
        pidMove();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        settarget(1); //forward();
        posReset(); // reset the position
        pidMove();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 2){
        settarget(4); //right();
        pidMove();
        posReset(); // reset the position
        settarget(4); //right();
        pidMove();
        curBotDir = 1;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 3){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 3){
        settarget(4); //right();
        pidMove();
        curBotDir = 1;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 3){
        settarget(3); //left();
        pidMove();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 3){
        settarget(4); //right();
        pidMove();
        posReset(); // reset the position
        settarget(4); //right();
        pidMove();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;      
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 4){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 6 && curBotDir == 4){
        settarget(4); //right();
        pidMove();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 6 && curBotDir == 4){
        settarget(3); //left();
        pidMove();
        curBotDir = 1;        
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 4){
        settarget(4); //right();
        pidMove();
        posReset(); // reset the position
        settarget(4); //right();
        pidMove();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        posReset(); // reset the position
        settarget(1); //forward();
        pidMove();
        curBotPos = Path[i];
        i++;
      }
    }
  Serial.print("curBotPos_aftermovement: ");
  Serial.println (curBotPos); 
  Serial.print("curBotDir_aftermovement: ");
  Serial.println (curBotDir);
  curBotPos2 = curBotPos;
  curBotDir2 = curBotDir;
  return curBotPos2,curBotDir2;
  
}

// Re-design the path if encounter obstacles
void rePathPlan(byte curBotPos,byte curBotDir)
{
 
  for (byte i = 0; i < 36; i++){

    if(PF.Map[i/6][i%6].index == 1){
      PF.Map[i/6][i%6].index = 0;
    }
    PF.Map[i/6][i%6].g = 0;
    PF.Map[i/6][i%6].h = 0;
    PF.Map[i/6][i%6].f = 0;
    PF.Map[i/6][i%6].parent = 0;
  }
   PF.Map[curBotPos/6][curBotPos%6].index = 1;
   PF.Map[goalN/6][goalN%6].index = 3;
  if(curBotDir == 1){
   PF.Map[(curBotPos + 6)/6][(curBotPos + 6)%6].index = 2;
  }
  else if(curBotDir == 2){
   PF.Map[(curBotPos - 6)/6][(curBotPos - 6)%6].index = 2;
  }
  else if(curBotDir == 3){
   PF.Map[(curBotPos + 1)/6][(curBotPos + 1)%6].index = 2;
  }
  else if(curBotDir == 4){
   PF.Map[(curBotPos - 1)/6][(curBotPos - 1)%6].index = 2;
  }
  
  oLN=0;
  cLN=0;

  for (byte i = 0; i<50; i++){
    openList[i] = 0; // contains all the possible paths
    closedList[i] = 0; // contains the path taken
    Path[i] = 0 ;   
  }
 Serial.print("curBotPos in re-path: ");
 Serial.println(curBotPos);
 Serial.print("curBotDir in re-path: ");
 Serial.println(curBotDir);
 printGrid2();
 
}

//Ultrasonic distance measurement Sub function
int check_obstacle(){
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW); 

  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;   
  Serial.println(Fdistance);
  
// Measure the response from the echo pin

// Determin distance from duration
// Use 343 metres per second as speed of sound
   if(Fdistance <= 20){
    obstacle = 1;
      Serial.println("Obstacle detected!");
   }
   else{
    obstacle = 0;
   }
   return obstacle;
}

void posReset(){
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
    posi[k] = 0;
  }
  for(int k = 0; k < NMOTORS; k++){
    pos[k] = 0;
  }
  interrupts(); // turn interrupts back on 
}

// PID Move Function
void pidMove(){
  posTrue = true;
  while (posTrue){
  pidFunc();        
  }
}

void stop() {
  digitalWrite(pwm[0], LOW);
  digitalWrite(pwm[1], LOW);
} 
