// IOT 라이브러리 포함 (2.4GHz만 연결됨: ESP32는 5GHz Wi-Fi 안 됨)
#include <WiFi.h>
#include <PubSubClient.h>

// ===== 상수설정 =====
const intSENSOR_PIN      = 36;// GPIO36(VP), wifi사용시 ADC1채널만 사용
const floatVREF            = 3.3;// esp32 3.3v 전원
const intADC_RESOLUTION  = 4095;   
const floatBURDEN_RES      = 4700;// 버든저항
const floatTURNS_RATIO     = 2000.0;// ct센서 내부 변압비
const floatLINE_VOLTAGE    = 220.0;// 가정용 콘센트 전압
const intSAMPLE_COUNT    = 2000;// 샘플 수
// ===== Wi-Fi =====
const char* WIFI_SSID = "iptime";
const char* WIFI_PASS = "";
// ===== MQTT =====
const char* MQTT_HOST = "broker.hivemq.com";// MQTT브로커 서버주소
const uint16_tMQTT_PORT = 1883;// 포트
const char* MQTT_CLIENT_ID = "esp32-ct-project";// ESP32 고유 ID
const char* MQTT_TOPIC = "home/power";// ESP32가 발행할 topic
// ===== TCP설정 =====
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
// ===== WIFI연결함수정의 =====
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while(WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\nWiFi OK, IP: "); Serial.println(WiFi.localIP());
}
// ===== MQTT연결함수정의 =====
void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  Serial.print("MQTT connecting");
  while(!mqtt.connected()) {
    if(mqtt.connect(MQTT_CLIENT_ID)) { Serial.println("\nMQTT OK"); break; }
    Serial.print("."); delay(1000);
}
}
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);// 현재 연결된 전원이 3.3V일 때 아날로그핀은 0~3.3V를 0~2^12=4095의 디지털 수로 치환해서 읽겠다는 선언
  
  connectWiFi();// WiFi,MQTT 연결함수 사용 (하드웨어-->네트워크 연결순서가 안전하고 직관적이기 때문에 set up 부분에서 가장 마지막에 사용)
  connectMQTT();
}
void loop() {
  if(WiFi.status() != WL_CONNECTED) connectWiFi();// WiFi,MQTT연결유지 (안정적인 네트워크 연결 유지를 위해 loop에서는 iot의 연결 유지 부분을 가장 먼저 사용)
  if(!mqtt.connected()) connectMQTT();
  mqtt.loop();
 // ===== 1) 오프셋노드 DC전압 먼저 추정 =====
  floatsumCounts = 0.0f;
  for(inti = 0; i < SAMPLE_COUNT; i++) {
sumCounts += analogRead(SENSOR_PIN);// 센서핀에 측정되는 위아래로 진동하는 AC전압은 2000번의 샘플링으로 인해 상쇄, 오프셋노드의 DC전압만 남게됨
    delayMicroseconds(200);
}
  floatmeanCounts = sumCounts / SAMPLE_COUNT;
  // ===== 2) AC RMS전압 추정 =====
  floatssqCounts = 0.0f;
  for(inti = 0; i < SAMPLE_COUNT; i++) {
    floatx = analogRead(SENSOR_PIN) - meanCounts;// x = DC전압 제거 후 나온 순간 AC전압
ssqCounts += x * x;
    delayMicroseconds(200);
}
  floatrmsCounts = sqrt(ssqCounts / SAMPLE_COUNT);// RMS공식을 이용해 AC전압의 RMS값 도출
  
  floatvRMS = (rmsCounts * VREF) / ADC_RESOLUTION;// 디지털 값 --> 아날로그 값
  floatiRMS = vRMS / ( BURDEN_RES / TURNS_RATIO );// V=I2*R, I2=I1/N       
  floatCAL_I = iRMS + 0.06;// 보정계수 적용
  // ===== 노이즈 컷 ======
  if(CAL_I < 0.09f) CAL_I = 0.0f;  
  floatpW = CAL_I * LINE_VOLTAGE;
  
  // ===== Serial =====
  Serial.print("V="); Serial.print(vRMS, 3);   Serial.print(" V ");
  Serial.print("I="); Serial.print(CAL_I, 2); Serial.print(" A  ");
  Serial.print("P="); Serial.print(pW, 2);   Serial.println(" W");
  // ===== MQTT Publish =====
  char payload[96];// payload에 발행할 내용 담기 (센서 측정이 끝난 최신 데이터를 발행하기 위해 가장 마지막에 사용)
  snprintf(payload, sizeof(payload),
           "{\"I\":%.2f,\"P\":%.2f}", CAL_I, pW);// 발행내용
  mqtt.publish(MQTT_TOPIC, payload, false);
  delay(500);
}
