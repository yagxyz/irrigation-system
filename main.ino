#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <NTPClient.h>//Biblioteca do NTP.
#include <WiFiUDP.h>

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "WifiF"
#define WLAN_PASS       "12345678" 

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883          
#define AIO_USERNAME    "Esp8266_IO"
#define AIO_KEY         "aio_TWiE27ltLZx979KzPUH9V0RaaBVr"

/*************************Variables********************************************/

int conn = 0; //verificar conexão wifi

// Tempo para conectar
uint64_t uS_TO_S_FACTOR = 1000000;  // Fator de conversão segundos
uint64_t TIME_TO_SLEEP = 60; // Segundos

#define Poten_PIN A0
#define LED_verde_PIN 0
#define Rele_PIN 2


unsigned long time1 = 0,time2 = 0,time3 = 0;
int delta_time = 0, delta_time1 = 0;
int c=0,e=0,e1=0,e2=0,e3=0,e4=0,e5=0,e6=0;

int potenc=1;
int potencant = 1;
int cond=0;
uint16_t cond1=0;
int hora,horant=0;


/************ Global State (you don't need to change this!) ******************/

// Create an ESP32 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

Adafruit_MQTT_Publish plot1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/plot1");
Adafruit_MQTT_Publish Irrigrecib = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Irrigrecib");
Adafruit_MQTT_Publish Irrigevent = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Irrigevent");
Adafruit_MQTT_Publish Irrigled = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Irrigled");
Adafruit_MQTT_Publish Noirrigled = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Noirrigled");
Adafruit_MQTT_Subscribe Irrigsend = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Irrigsend");

WiFiUDP udp;
NTPClient ntp(udp, "a.st1.ntp.br", -3 * 3600, 60000);


/*************************** Sketch Code ************************************/
//MQTT_connect();
// initWifi(); 
//closedcon();
//piscaled();

void setup() {
  pinMode (LED_verde_PIN, OUTPUT);
  pinMode (Rele_PIN, OUTPUT);
  digitalWrite(LED_verde_PIN,HIGH);
  digitalWrite(Rele_PIN,HIGH);
  
  Serial.begin(9600); 
  delay(5000);

  initWifi(); //função para fazer wifi connection, retorn 1 se conectado
 
  Serial.println(F("Sistema de Irrigação"));
  mqtt.subscribe(&Irrigsend);

  ntp.begin();//Inicia o NTP.
  ntp.forceUpdate();//Força o Update.
}

void loop() {
  hora = ntp.getHours();//Armazena na váriavel HORA, o horario atual.
  if(horant!=hora){c=0;}
  
  time1 = millis();
  delta_time = time1-time2;
  delta_time1 = time1-time3;
  
  if(c == 1 && delta_time >= 5*3*1000){
    digitalWrite(Rele_PIN, HIGH); Irrigevent.publish("Irrigação Realizada!");
    c=2; Irrigled.publish(0);e=0;
    e1=0;e2=0;e3=0;e4=0;e5=0;e6=0;
    }
  
  if (conn==1){ //Se conectado
       digitalWrite(LED_verde_PIN, LOW);
       ntp.update();
       
       MQTT_connect();
       
       potenc = analogRead(Poten_PIN);
       potenc = map(potenc, 0, 1023, 2, 6);

        Adafruit_MQTT_Subscribe *subscription;
        while ((subscription = mqtt.readSubscription(500))) {
          if (subscription == &Irrigsend) {Serial.print(F("Got: ")); Serial.println((char *)Irrigsend.lastread);} 
          }

       cond = potencant-potenc;
       if(cond>=1 || cond<=-1 ){Irrigrecib.publish(potenc); piscaled(); potencant = potenc;}
       if(cond1!=atoi((char *)Irrigsend.lastread)){cond1=atoi((char *)Irrigsend.lastread);Irrigrecib.publish((char *)Irrigsend.lastread);}
       

        switch(atoi((char *)Irrigsend.lastread)){
          case 0: if(e==0){if(c==0){digitalWrite(Rele_PIN, LOW); Irrigevent.publish("Irrigação Iniciada..."); time2=time1; c=1; horant=hora;Irrigled.publish(1);e=1;}}                  
          break;
          case 2: if(hora==6||hora==18){e2=1;e5=1;}
          break;
          case 3: if(hora==6||hora==14||hora==18){e2=1;e4=1;e5=1;}
          break;
          case 4: if(hora==4||hora==10||hora==14||hora==22){e1=1;e3=1;e4=1;e6=1;}
          break;
          case 5: if(hora==4||hora==10||hora==14||hora==18||hora==22){e1=1;e3=1;e4=1;e5=1;e6=1;}
          break;
          case 6: e1=1;e2=1;e3=1;e4=1;e5=1;e6=1;
          break;          
        }

        
        //condicionais
        switch(hora){
          case 4: if(c==0&&e1==1){digitalWrite(Rele_PIN, LOW); Irrigevent.publish("Irrigação Iniciada..."); time2=time1; c=1; horant=hora;Irrigled.publish(1);}                  
          break;
          case 6: if(c==0&&e2==1){digitalWrite(Rele_PIN, LOW); Irrigevent.publish("Irrigação Iniciada..."); time2=time1; c=1; horant=hora;Irrigled.publish(1);}
          break;
          case 10: if(c==0&&e3==1){digitalWrite(Rele_PIN, LOW); Irrigevent.publish("Irrigação Iniciada..."); time2=time1; c=1; horant=hora;Irrigled.publish(1);}
          break;
          case 14: if(c==0&&e4==1){digitalWrite(Rele_PIN, LOW); Irrigevent.publish("Irrigação Iniciada..."); time2=time1; c=1; horant=hora;Irrigled.publish(1);}
          break;
          case 18: if(c==0&&e5==1){digitalWrite(Rele_PIN, LOW); Irrigevent.publish("Irrigação Iniciada..."); time2=time1; c=1; horant=hora;Irrigled.publish(1);}
          break;
          case 22: if(c==0&&e6==1){digitalWrite(Rele_PIN, LOW); Irrigevent.publish("Irrigação Iniciada..."); time2=time1; c=1; horant=hora;Irrigled.publish(1);}
          break;
          }
         if(! mqtt.ping()){ mqtt.disconnect();}
         delay(500);                  
       }

       else { //Se não conectado
        potenc = analogRead(Poten_PIN);
        potenc = map(potenc, 0, 1023, 2, 6);

        cond = potencant-potenc;
        if(cond>=1 || cond<=-1 ){Serial.println(potenc); delay(100); piscaled(); potencant=potenc;}
        else{
          digitalWrite (LED_verde_PIN, LOW);
          delay(500);
          digitalWrite (LED_verde_PIN, HIGH);
          delay(500);
          }

        //condicionais
        switch(hora){
          case 3: if(c==0){digitalWrite(Rele_PIN, LOW); Serial.println("Irrigação Ligada"); time2=time1; c=1; horant=hora;}                  
          break;
          case 6: if(c==0){digitalWrite(Rele_PIN, LOW); Serial.println("Irrigação Ligada"); time2=time1; c=1; horant=hora;}
          break;
          case 10: if(c==0){digitalWrite(Rele_PIN, LOW); Serial.println("Irrigação Ligada"); time2=time1; c=1; horant=hora;}
          break;
          case 14: if(c==0){digitalWrite(Rele_PIN, LOW); Serial.println("Irrigação Ligada"); time2=time1; c=1; horant=hora;}
          break;
          case 18: if(c==0){digitalWrite(Rele_PIN, LOW); Serial.println("Irrigação Ligada"); time2=time1; c=1; horant=hora;}
          break;
          case 22: if(c==0){digitalWrite(Rele_PIN, LOW); Serial.println("Irrigação Ligada"); time2=time1; c=1; horant=hora;}
          break;
          }            
        if (delta_time1 >= 10*3600*1000){initWifi();time3 = time1;}        
        }
}

int MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()){conn=1; return conn;}

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0 && retries == 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
  }
 if(!mqtt.connected()){Serial.println("MQTT not Connecting");conn=0;} 
  return conn;
}

// Establish a Wi-Fi connection with your router
int initWifi() {
  Serial.print("Connecting to: "); 
  Serial.print(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);  

  int timeout = 5; // 5 seconds
  while(WiFi.status() != WL_CONNECTED  && (timeout-- > 0)) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");

  if(WiFi.status() != WL_CONNECTED) {
     Serial.println("Failed to connect, going back to sleep");
     conn=0;
  }

if(WiFi.status() == WL_CONNECTED) {
  Serial.print("WiFi connected in: "); 
  Serial.print(millis());
  Serial.print("milisegundos"); 
  Serial.print(", IP address: "); 
  Serial.println(WiFi.localIP());
  conn=1;
  }
return conn;
}
void piscaled()
{int i=0;
  while(i<10){
    digitalWrite (LED_verde_PIN, HIGH);
  delay(map(potenc, 2, 6, 100, 40));
    digitalWrite (LED_verde_PIN, LOW);
  delay(map(potenc, 2, 6, 100, 40));
  i++;
}
i=10;
}