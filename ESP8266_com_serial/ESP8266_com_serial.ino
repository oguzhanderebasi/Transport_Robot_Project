#include <Firebase.h>
#include <FirebaseCloudMessaging.h>
#include <FirebaseError.h>
#include <FirebaseHttpClient.h>
#include <FirebaseObject.h>

#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <SoftwareSerial.h>
#define PATH "/station"

//D6 = Rx & D5 = Tx
SoftwareSerial nodemcu(D6, D5);

// Set these to run example.
#define FIREBASE_HOST "fir-com-c8de4-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "Mr9oVgDez2tNddLNc59GAYVDSp1JRcB1xg4yiJP2"
#define WIFI_SSID "DSH"
#define WIFI_PASSWORD "Wohngemeinschaft"

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

  Firebase.setInt("station", 0);
  if (Firebase.failed()) {
    Serial.print("setting /station failed:");
    Serial.println(Firebase.error());  
    return;
  }

  Firebase.stream(PATH);
  
}

int station;
int station_alt = 0;
const char* myString; // complete message from Arduino, which consists of sensors data
char rdata; // received charactors
int counter = 1;
char Tag_buffer[10];
char Tag_search[10];
String Firebase_search;
int check = 0;

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

  // Arduino'dan gelen JSON RFID-Tag verilerini ayrıştırmak obje oluşturuyoruz
  StaticJsonBuffer<1000> jb;
  JsonObject& root = jb.parseObject(nodemcu);

  // Eğer bir veri geldiyse:
  if (root.success()){
    myString = root["tag_data"]; // RFID-Tag verisini kaydedildi
    Serial.println("Received Tag: "); Serial.println(myString);

    String myString2 = String(myString); // const char* tipindeki Tag, String'e dönüştürüldü
    
    for(int i=1; i <= counter; i++){  // Okunan Tag'in daha önce Firebase'de kayıtlı olup olmadığı kontrol ediliyor.
      sprintf(Tag_search, "Tag_ID/%d", i);  // Firebase yolu sırayla taranacak
      Firebase_search = Firebase.getString(Tag_search); // Bulunan Tag_ID'si kaydedildi
      if(myString2.equals(Firebase_search)){  // Firebase'de bulunan ID ile okunan ID eşleşiyor mu?
        check = 1;  // Eğer Firebase'deki herhangi bir ID eşleşiyorsa değer 1 olur.
        Serial.println("This ID already exists.");
      }
    }

    if(check == 0){
      sprintf(Tag_buffer, "Tag_ID/%d", counter); // RFID-Tag'lerini, Firebase'e sırayla kaydedebilmek için oluşturuldu
      Firebase.setString(Tag_buffer, myString); // Tag set edilir
      Serial.println("RFID-Tag has been successfully saved in Firebase");
      counter ++; // Sonraki Tag için sayaç 1 artırılır
      memset(Tag_buffer, 0, 10);  // Firebase'e kaydetme dizini sonraki aşama için temizlenir
    }

    
    if (Firebase.failed()) {
      Serial.print("setting /Tag_ID failed:");
      Serial.println(Firebase.error());
    }
  }
  
  station = Firebase.getInt("station");
  
  if(station_alt != station){
    nodemcu.print(station);
    Serial.print("Station: "); Serial.println(station);
    Serial.println("Info of Station has been transported.");
    station_alt = station;
  }
  
/*  if (nodemcu.available() > 0 ){
    rdata = nodemcu.read(); 
    myString = myString + rdata; 
    //Serial.println("rdata: "); Serial.println(rdata);
    Serial.println("myString: "); Serial.println(myString);
    //myString = "";
  }*/

}
