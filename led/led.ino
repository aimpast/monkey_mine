const int serialPort = 9600;

const int ledBlue = 0;
const int ledYerrow = 1;
const int ledRed = 2;

const int patternBlue = 1;
const int patternYerrow = 3;
const int patternRed = 2;

const int maxDistance = 100;

const int delayTime = 10;

int distance[3];

void setLed(int distanceBlue, int distanceYerrow, int distanceRed) {
  if(distanceBlue < 0) distanceBlue = 0;
  if(distanceBlue > maxDistance) distanceBlue = maxDistance;
  if(distanceYerrow < 0) distanceYerrow = 0;
  if(distanceYerrow > maxDistance) distanceYerrow = maxDistance;
  if(distanceRed < 0) distanceRed = 0;
  if(distanceRed > maxDistance) distanceRed = maxDistance;

  distance[ledBlue] = distanceBlue;
  distance[ledYerrow] = distanceYerrow;
  distance[ledRed] = distanceRed;
}

void serialRead() {
  while(Serial.read() == 255);
  setLed(Serial.read(), Serial.read(), Serial.read());
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
  if(_lightPattern(distance[led], i, pattern)) {
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
  
  distance[ledBlue] = maxDistance;
  distance[ledYerrow] = maxDistance;
  distance[ledRed] = maxDistance;
}

void loop() {
  serialRead();
  
  static int i = 0;
  i++;
  if(i > maxDistance) i = maxDistance;

  lightLed(ledBlue, i, patternBlue);
  lightLed(ledYerrow, i, patternYerrow);
  lightLed(ledRed, i, patternRed);

  delay(delayTime);
}

