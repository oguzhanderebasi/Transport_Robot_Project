//RFID and SPI communication lib
#include <SPI.h>
#include <RFID.h>

//Initialise RFID-Reader
RFID lrt720(53, 4);

//Arduino to NodeMCU lib
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

//Initialise Arduino to NodeMCU (10=Rx & 11=Tx)
SoftwareSerial nodemcu(10, 11);

//Motor pins definiton
#define MotorR1 7
#define MotorR2 8
#define MotorRG 9

#define MotorL1 6
#define MotorL2 5
#define MotorLG 3

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  SPI.begin();
  lrt720.init();

  nodemcu.begin(9600);

  pinMode(MotorR1, OUTPUT);
  pinMode(MotorR2, OUTPUT);
  pinMode(MotorRG, OUTPUT);
  
  pinMode(MotorL1, OUTPUT);
  pinMode(MotorL2, OUTPUT);
  pinMode(MotorLG, OUTPUT);

  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Program started");

}

int s0;
int s1;
int s2;
int s3;
int s4;
String tag_data;
String tag_data_old = "";
long int nodemcu_data;

void loop() {
  StaticJsonBuffer<1000> jb;
  JsonObject& root = jb.createObject();
  
  if(nodemcu.available() == 0 ){
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
          Serial.println("Card ID : ");
          root["tag_data"] = tag_data;
          Serial.println(tag_data);
          root.printTo(nodemcu);
          //nodemcu.println(tag_data);
          delay(200);
          tag_data_old = tag_data;
        }
      }
    }
  }

  if ( nodemcu.available() > 0 ){
    nodemcu_data = nodemcu.parseInt();
    delay(100);
    Serial.print("Station= "); Serial.println(nodemcu_data);
    
  }

}
