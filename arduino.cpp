#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#define VALVE_OXYGEN 2
#define VALVE_METHANE 3
#define VALVE_DRAIN 4

#define GAS_SENSOR_RX 10
#define GAS_SENSOR_TX 11
SoftwareSerial gasSensorSerial(GAS_SENSOR_RX, GAS_SENSOR_TX);

#define MODBUS_RX 12
#define MODBUS_TX 13
#define MODBUS_DE 8
SoftwareSerial modbusSerial(MODBUS_RX, MODBUS_TX);
ModbusMaster RRG;


#define LEVEL_SENSOR_PIN 5

float methaneConcentration = 0;
int levelSensorState = 0;

#define CONTROL_PANEL_PIN 7
int controlPanelState = LOW;

void preTransmission() {
  digitalWrite(MODBUS_DE, HIGH);
}

void postTransmission() {
  digitalWrite(MODBUS_DE, LOW);
}

void setup() {
  pinMode(VALVE_OXYGEN, OUTPUT);
  pinMode(VALVE_METHANE, OUTPUT);
  pinMode(VALVE_DRAIN, OUTPUT);
  pinMode(LEVEL_SENSOR_PIN, INPUT);
  pinMode(CONTROL_PANEL_PIN, INPUT);
  digitalWrite(MODBUS_DE, LOW);
  pinMode(MODBUS_DE, OUTPUT);

  gasSensorSerial.begin(9600);

  modbusSerial.begin(9600);
  RRG.begin(1, modbusSerial); 
  RRG.preTransmission(preTransmission);
  RRG.postTransmission(postTransmission);

  Serial.begin(9600); 

  digitalWrite(VALVE_OXYGEN, LOW);
  digitalWrite(VALVE_METHANE, LOW);
  digitalWrite(VALVE_DRAIN, LOW);
}

void loop() {
  if (gasSensorSerial.available()) {
    String gasData = gasSensorSerial.readStringUntil('\n');
    methaneConcentration = gasData.toFloat();
    Serial.print("Methane Concentration: ");
    Serial.println(methaneConcentration);
  }

  levelSensorState = digitalRead(LEVEL_SENSOR_PIN);
  Serial.print("Level Sensor State: ");
  Serial.println(levelSensorState);

  controlPanelState = digitalRead(CONTROL_PANEL_PIN);
  if (controlPanelState == HIGH) {
    digitalWrite(VALVE_OXYGEN, HIGH);
    digitalWrite(VALVE_METHANE, LOW);
  } else {
    digitalWrite(VALVE_OXYGEN, LOW);
    digitalWrite(VALVE_METHANE, LOW);
  }

  uint8_t result;
  result = RRG.readHoldingRegisters(0x0000, 2); 
  if (result == RRG.ku8MBSuccess) {
    uint16_t flowRate = RRG.getResponseBuffer(0);
    uint16_t pressure = RRG.getResponseBuffer(1);
    Serial.print("Flow Rate: ");
    Serial.println(flowRate);
    Serial.print("Pressure: ");
    Serial.println(pressure);
  }

  if (levelSensorState == HIGH) {
    digitalWrite(VALVE_DRAIN, HIGH);
  } else {
    digitalWrite(VALVE_DRAIN, LOW);
  }

  delay(1000); 
}
