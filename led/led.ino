const char serialStart = 'z';
const int serialPort = 9600;

const int ledBlue = 2;
const int ledYerrow = 3;
const int ledRed = 4;

const int patternBlue = 1;
const int patternYerrow = 1;
const int patternRed = 1;

const int maxDistance = 100;

const int delayTime = 1;

int distances[20];
int i = 0;

void setLed(int distanceBlue, int distanceYerrow, int distanceRed) {
  if(distanceBlue < 0) distanceBlue = 0;
  if(distanceBlue > maxDistance) distanceBlue = maxDistance;
  if(distanceYerrow < 0) distanceYerrow = 0;
  if(distanceYerrow > maxDistance) distanceYerrow = maxDistance;
  if(distanceRed < 0) distanceRed = 0;
  if(distanceRed > maxDistance) distanceRed = maxDistance;

  distances[ledBlue] = distanceBlue;
  distances[ledYerrow] = distanceYerrow;
  distances[ledRed] = distanceRed;
}

void serialRead() {
  if((Serial.available() > 3) && (Serial.read() == serialStart)) {
    setLed(Serial.read(), Serial.read(), Serial.read());
    Serial.print("!");
  }
  Serial.print(distances[ledBlue]);
  Serial.print("|");
  Serial.print(distances[ledYerrow]);
  Serial.print("|");
  Serial.print(distances[ledRed]);
  Serial.print("|");
  Serial.print(i);
  Serial.print("\n");
}

bool _lightPattern(int distance, int i, int pattern) {
  switch(pattern) {
  case 1:
    return distance < i;
  case 2:
    return distance/3 < i%(maxDistance/3);
  case 3:
    return (i%2 == 0) && (distance/3 < i%(maxDistance/3));
  default:
    return false;
  }
  return false;
}

void lightLed(int led, int i, int pattern) {
  if(_lightPattern(distances[led], i, pattern)) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }
}

void setup() {
  Serial.begin(serialPort);
  
  pinMode(ledBlue, OUTPUT);
  pinMode(ledYerrow, OUTPUT);
  pinMode(ledRed, OUTPUT);
  
  distances[ledBlue] = maxDistance;
  distances[ledYerrow] = maxDistance;
  distances[ledRed] = maxDistance;
}

void loop() {
  serialRead();
  
  i++;
  if(i > maxDistance) i = 0;

  lightLed(ledBlue, i, patternBlue);
  lightLed(ledYerrow, i, patternYerrow);
  lightLed(ledRed, i, patternRed);

  delay(delayTime);
}

