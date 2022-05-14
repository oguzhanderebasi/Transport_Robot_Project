#include <Firebase.h>
#include <FirebaseCloudMessaging.h>
#include <FirebaseError.h>
#include <FirebaseHttpClient.h>
#include <FirebaseObject.h>

#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <SoftwareSerial.h>
#define PATH "/station"
#define PATHTAG "/Tag_ID"

//D6 = Rx & D5 = Tx
SoftwareSerial nodemcu(D6, D5);

#define FIREBASE_HOST ""
#define FIREBASE_AUTH ""
#define WIFI_SSID "DSH"  // DSH, TAU
#define WIFI_PASSWORD ""

void setup() {
  Serial.begin(115200);
  nodemcu.begin(9600);

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  Firebase.setInt("station", -1);
  if (Firebase.failed()) {
    Serial.print("setting /station failed:");
    Serial.println(Firebase.error());  
    return;
  }

  Firebase.stream(PATH);
  
}

int station;
int station_alt = -1;
const char* myString; // complete message from Arduino, which consists of sensors data
char rdata; // received charactors
int counter = 0;
char Tag_buffer[10];
char database_tag_search[10];
char Tag_search[10];
String Firebase_search;
int check = 0;
byte gridNom;
const String tag_id = "Tag_ID";
String value;
bool first_check = false;
String first_Tag = "Tag_ID/0";
String support_String;

void loop() {
  check = 0;
  //check if streaming failed and try to restart
  if (Firebase.failed()) {
    Serial.println("streaming error");
    Serial.println(Firebase.error());
    delay(1000);
    Firebase.stream(PATH);
    return;
  }

  if(!first_check){
    for(int i=0; i <= 35; i++){
    sprintf(database_tag_search, "Tag_ID/%d", i);
    value = Firebase.getString(database_tag_search);
    support_String = (String) database_tag_search;
    if(support_String.equals(first_Tag)){
      if(value == ""){
        first_check = true;
        break;
      }
      counter = 0;
    } else{
      if(value == ""){
          first_check = true;
          break;
      }
      counter ++;
    }
    
    Serial.print("value: ");
    Serial.println(value);
    Serial.print("counter: ");
    Serial.println(counter);
    } 
  }

  

  // Arduino'dan gelen JSON RFID-Tag verilerini ayrıştırmak obje oluşturuyoruz
  StaticJsonBuffer<1000> jb;
  JsonObject& root = jb.parseObject(nodemcu);

  // Eğer bir veri geldiyse:
  if (root.success()){
    myString = root["tag_data"]; // RFID-Tag verisini kaydedildi
    Serial.println("Received Tag: "); Serial.println(myString);

    String myString2 = String(myString); // const char* tipindeki Tag, String'e dönüştürüldü
    
    for(int i=0; i <= counter; i++){  // Okunan Tag'in daha önce Firebase'de kayıtlı olup olmadığı kontrol ediliyor.
      sprintf(Tag_search, "Tag_ID/%d", i);  // Firebase yolu sırayla taranacak
      Firebase_search = Firebase.getString(Tag_search); // Bulunan Tag_ID'si kaydedildi
      if(myString2.equals(Firebase_search)){  // Firebase'de bulunan ID ile okunan ID eşleşiyor mu?
        check = 1;  // Eğer Firebase'deki herhangi bir ID eşleşiyorsa değer 1 olur.
        gridNom = i;
        //Firebase.stream(PATHTAG);
        grid_station_info();
        Serial.println("This ID already exists.");
        Serial.print("Position of the Robot (GridNom): ");
        Serial.println(gridNom);
        Serial.println("================================================");
      }
    }

    if(check == 0){
      if(counter == 0){
        sprintf(Tag_buffer, "Tag_ID/%d", counter); // RFID-Tag'lerini, Firebase'e sırayla kaydedebilmek için oluşturuldu
        Firebase.setString(Tag_buffer, myString); // Tag set edilir
        grid_station_counter_info();
        Serial.println("RFID-Tag has been successfully saved in Firebase");
        Serial.println("================================================");
        counter ++; // Sonraki Tag için sayaç 1 artırılır
        memset(Tag_buffer, 0, 10);  // Firebase'e kaydetme dizini sonraki aşama için temizlenir
      } else {
        sprintf(Tag_buffer, "Tag_ID/%d", counter+1); // RFID-Tag'lerini, Firebase'e sırayla kaydedebilmek için oluşturuldu
        Firebase.setString(Tag_buffer, myString); // Tag set edilir
        grid_station_counter_info();
        Serial.println("RFID-Tag has been successfully saved in Firebase");
        Serial.println("================================================");
        counter ++; // Sonraki Tag için sayaç 1 artırılır
        memset(Tag_buffer, 0, 10);  // Firebase'e kaydetme dizini sonraki aşama için temizlenir
      }
    }

    
    if (Firebase.failed()) {
      Serial.print("setting /Tag_ID failed:");
      Serial.println(Firebase.error());
    }
  }
  
  station = Firebase.getInt("station");
  
  if(station_alt != station){
    grid_station_info();
    Serial.print("Station: "); Serial.println(station);
    Serial.println("Info of Station has been transported.");
    Serial.println("================================================");
    station_alt = station;
  }
}

void grid_station_info(){
  nodemcu.print(gridNom);
  nodemcu.print("A");
  nodemcu.print(station);
  nodemcu.print("B");
  nodemcu.print("\n");
}

void grid_station_counter_info(){
  nodemcu.print(counter);
  nodemcu.print("A");
  nodemcu.print(station);
  nodemcu.print("B");
  nodemcu.print("\n");
}
