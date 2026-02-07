#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// =====================
// WIFI CONFIG
// =====================
const char* ssid = "XXXX";
const char* password = "YYYYYYYYY";

// =====================
// UDP CONFIG
// =====================
WiFiUDP udp;
const unsigned int localPort = 5005;
char packetBuffer[350];

// =====================
// LED GPIOs (4 LEDs)
// =====================
#define LED_LAMP     4    // D2  White lamp (PWM)
#define LED_OCC      12   // D6  White occupancy
#define LED_DROWSY   5    // D1  Red drowsy
#define LED_ENG      13   // D7  Red engagement

// =====================
// STATE
// =====================
unsigned long lastPacketTime = 0;
const unsigned long UDP_TIMEOUT = 5000;

int restlessCount = 0;

// Lamp brightness levels (tuned for 220Ω)
const int LAMP_OFF  = 0;
const int LAMP_DIM  = 180;
const int LAMP_FULL = 450;

// =====================
void allOff() {
  digitalWrite(LED_OCC, LOW);
  digitalWrite(LED_DROWSY, LOW);
  digitalWrite(LED_ENG, LOW);
  analogWrite(LED_LAMP, LAMP_OFF);
  restlessCount = 0;
}

// =====================
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP UDP LED Controller (Final, Comfort Tuned)");

  pinMode(LED_LAMP, OUTPUT);
  pinMode(LED_OCC, OUTPUT);
  pinMode(LED_DROWSY, OUTPUT);
  pinMode(LED_ENG, OUTPUT);

  allOff();

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.print("UDP listening on port ");
  Serial.println(localPort);
}

// =====================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }

  int packetSize = udp.parsePacket();
  if (!packetSize) {
    // ---- FAILSAFE ----
    if (millis() - lastPacketTime > UDP_TIMEOUT) {
      allOff();
    }
    return;
  }

  int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
  if (len <= 0) return;
  packetBuffer[len] = '\0';

  String msg = String(packetBuffer);
  lastPacketTime = millis();

  Serial.print("UDP >>> ");
  Serial.println(msg);

  // =====================
  // OCCUPANCY MASTER
  // =====================
  bool occupied = msg.indexOf("OCC=1") >= 0;

  if (!occupied) {
    allOff();
    return;
  }

  // Room occupied
  digitalWrite(LED_OCC, HIGH);

  // =====================
  // DROWSY
  // =====================
  bool drowsy = msg.indexOf("DROWSY=1") >= 0;
  digitalWrite(LED_DROWSY, drowsy ? HIGH : LOW);

  // =====================
  // ENGAGEMENT (HYSTERESIS)
  // =====================
  if (msg.indexOf("ENG=RESTLESS") >= 0 ||
      msg.indexOf("ENG=PASSIVE") >= 0) {
    restlessCount++;
  } else {
    restlessCount = 0;
  }
  digitalWrite(LED_ENG, restlessCount >= 2 ? HIGH : LOW);

  // =====================
  // PSEUDO LAMP CONTROL
  // =====================
  if (drowsy) {
    analogWrite(LED_LAMP, LAMP_DIM);    // dim, calming light
  } else {
    analogWrite(LED_LAMP, LAMP_FULL);   // comfortable brightness
  }
}
