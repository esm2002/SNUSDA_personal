const int potPin = A0; // 포텐시오미터 연결 핀

void setup() {
  Serial.begin(57600); // 시리얼 통신 초기화
}

void loop() {
  int sensorValue = analogRead(potPin); // 포텐시오미터의 아날로그 값을 읽기
  Serial.print("Analog 값: ");
  Serial.println(sensorValue); // 시리얼 모니터에 출력
  delay(100); // 0.5초 딜레이
}
