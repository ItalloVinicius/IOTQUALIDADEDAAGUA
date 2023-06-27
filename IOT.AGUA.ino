//Bibliotecas
#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <PubSubClient.h>
#include "dht.h"
#include <OneWire.h>  
#include <DallasTemperature.h>

//Sensor temperatura
#define dados 25

OneWire oneWire(dados);
DallasTemperature sensors(&oneWire);

////Variaveis sensor DHT ----------
const int pinoDHT11 = 33;
dht DHT; 

//LED
int LEDAZUL = 12 ;

//Variaveis sensor TDS ----------
int TdsSensorPin = 34;
#define VREF 5    
#define SCOUNT  3          
int analogBuffer[SCOUNT];   
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0, temperature = 25;
float TdsMQTT;


//Variaveis Sensor PH ----------
#define SensorPin 35  
float PH;
float PHMQTT;

//Variaveis Sensor Turbidez ----------
int sensorPin = A3; 
int leitura; 

//Wifi area
const char* ssid     = "IOT";
const char* password = "redeprimaria";
const char* mqtt_broker = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient MQTT(espClient);

//SETUP ----------
void setup() 
{
  Serial.begin(9600);
  pinMode (LEDAZUL, OUTPUT); 
  pinMode(TdsSensorPin, INPUT);
  sensors.begin();
  //WIFI
  Serial.println("Iniciando WIFI ");
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("CONECTADO!");
  digitalWrite (LEDAZUL, HIGH);
  Serial.println("Endereço IP:  ");
  Serial.print(WiFi.localIP());
  MQTT.setServer(mqtt_broker,1883);
}

//LOOP ----------
void loop()
 
{
 
  delay(5000);
    if (!MQTT.connected()) 
    {
    reconnect_MQTT();
    digitalWrite (LEDAZUL, LOW);
    }
  Sensor_TDS();
  Sensor_PH();
  Sensor_TURBIDEZ();
  Sensor_TEMPERATURA();
  Sensor_DHT();
  publish_MQTT();
  CONSUMO(); 
}

//FUNÇÕES

void Sensor_TDS()
{
static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4095.0;
      float compensationCoefficient=0.8+0.02*(temperature-25.0);    
      float compensationVolatge=averageVoltage/compensationCoefficient;  
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; 

      Serial.println("SENSOR DE TDS");
      Serial.print("TDS Value:");
      TdsMQTT = tdsValue;
      Serial.print(tdsValue);
      Serial.println(" ppm");
      Serial.println("-------------------------------------------");
   }
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void Sensor_PH()
{
    int measure = analogRead(ph_pin);
    Serial.println("SENSOR DE PH");
  Serial.print("Measure: ");
  Serial.print(measure);

  double voltage = 5 / 1024.0 * measure;
  Serial.print("\tVoltage: ");
  Serial.print(voltage, 5);
  float Po = 7 + (( - voltage) / 0.18);
  Serial.print("\tPH: ");
  Serial.print(abs(Po), 5);

  Serial.println("");
  Serial.println("-------------------------------------------");
  delay(500);
}
}

void Sensor_TURBIDEZ()
{
  leitura = analogRead(sensorPin);
  Serial.println("SENSOR DE TURBIDEZ");
  Serial.print("Valor lido de Turbidez: "); 
  Serial.println(leitura); 
  delay(500); 
  Serial.print("Estado da água: "); 

  if (leitura > 900) { 

    Serial.println("LIMPA"); 
     MQTT.publish("IOT.AGUA/TURBIDEZ","Limpa");
  }
  if ((leitura >= 899 ) && (leitura <= 700)) { 

    Serial.println("POUCO SUJA"); 
    MQTT.publish("IOT.AGUA/TURBIDEZ","Pouco Suja");
  }
  if (leitura < 699) { 

    Serial.println("MUITO SUJA"); 
    MQTT.publish("IOT.AGUA/TURBIDEZ","Muito Suja");
  }
   Serial.println("-------------------------------------------");
}

void reconnect_MQTT(){
  while (!MQTT.connected()) {
    MQTT.connect("IOT.AGUA");
    Serial.println("Reconectando...");
  }
}

void publish_MQTT(){

  MQTT.publish("IOT.AGUA/TDS", String(TdsMQTT).c_str(), true);
  MQTT.publish("IOT.AGUA/PH", String(Po)/.c_str(), true);
  MQTT.publish("IOT.AGUA/TEMPERATURAINTERNA", String(DHT.temperature).c_str(), true);
  MQTT.publish("IOT.AGUA/HUMITY", String(DHT.humidity).c_str(), true);
  MQTT.publish("IOT.AGUA/TEMPERATURA", String(sensors.getTempCByIndex(0)).c_str(), true);
  
}

void Sensor_DHT()
{
  DHT.read11(pinoDHT11);
   Serial.println("SENSOR DHT11");
  Serial.print("Umidade: ");
  Serial.print(DHT.humidity);
  Serial.print("%");IME
  Serial.print("Temperatura: ");
  Serial.print(DHT.temperature, 0);
  Serial.println("*C");
  delay(500);
}

void Sensor_TEMPERATURA()
{ 

 Serial.println(" SENSOR DE TEMPERATURA"); 
 sensors.requestTemperatures(); 
 Serial.print("Temperatura: ");
 Serial.print(sensors.getTempCByIndex(0));
 delay(500);
}

void CONSUMO()
{
   if ((TdsMQTT < 100 ) && (leitura > 900)) {

    Serial.println("APROPRIADA"); 
     MQTT.publish("IOT.AGUA/CONSUMO","APROPRIADA");
  }
     if ((TdsMQTT > 100 ) && (leitura < 900)) { 
    Serial.println("INAPROPRIADA"); 
     MQTT.publish("IOT.AGUA/CONSUMO","INAPROPRIADA");
  }
}