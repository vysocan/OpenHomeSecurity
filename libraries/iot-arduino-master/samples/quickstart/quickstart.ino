#include <SPI.h>
#include <Ethernet.h>
#include <EthernetStack.h>
#include <Countdown.h>
#include <MQTTClient.h>


// Update these with values suitable for your network.
byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };
char quickstartMQTTDNS[] = "quickstart.messaging.internetofthings.ibmcloud.com";
int mqttPort = 1883;
//The convention to be followed is d:quickstart:iotsample-arduino:<MAC Address>
char clientId[] = "d:quickstart:iotsample-arduino:00aabbccde02";
char topic[] = "iot-2/evt/status/fmt/json";

#define MQTT_MAX_PACKET_SIZE 100
  
EthernetStack ipstack;  
MQTT::Client<EthernetStack, Countdown, MQTT_MAX_PACKET_SIZE> client(ipstack);

void setup() {
  Serial.begin(9600);
  Ethernet.begin(mac);
  delay(2000);
}

void loop() {
  int rc = -1;
  if (!client.isConnected()) {
    Serial.println("Connecting\n");
    while (rc != 0) {
      rc = ipstack.connect(quickstartMQTTDNS, mqttPort);
    }
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = clientId;
    rc = -1;
    while ((rc = client.connect(data)) != 0)
      ;
    Serial.println("Connected\n");
  }

  char json[56] = "{\"d\":{\"myName\":\"Arduino Uno\",\"temperature\":";
  dtostrf(getTemp(),1,2, &json[43]);
  json[48] = '}';
  json[49] = '}';
  json[50] = '\0';
  Serial.println(json);
  MQTT::Message message;
  message.qos = MQTT::QOS0; 
  message.retained = false;
  message.payload = json; 
  message.payloadlen = strlen(json);
  rc = client.publish(topic, message);
  if (rc != 0) {
    Serial.print("return code from publish was ");
    Serial.println(rc);
  }
  client.yield(1000);
}

/*
This function is reproduced as is from Arduino site => http://playground.arduino.cc/Main/InternalTemperatureSensor
*/
double getTemp(void) {
  unsigned int wADC;
  double t;
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN); // enable the ADC
  delay(20); // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC); // Start the ADC
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;
  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;
  // The returned temperature is in degrees Celcius.
  return (t);
}
