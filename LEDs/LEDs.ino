// #include <string>

int powerPin = 3, rPin = 9, gPin = 10, bPin = 11;

String gold = "#FFD700";
String green = "#61B077";
String red = "#FF0000";
String currentColour = "";

bool power = false;

unsigned long previousMillis = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(powerPin, OUTPUT);
  pinMode(rPin, OUTPUT);
  pinMode(gPin, OUTPUT);
  pinMode(bPin, OUTPUT);

  Serial.begin(9600);

  digitalWrite(powerPin, HIGH);
  power = true;
  // digitalWrite(rPin, LOW);
  analogWrite(rPin, -255);
  analogWrite(gPin, -190);
  analogWrite(bPin, 255);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

void setColour(String colour) {
  // string hexstring = "#FF3Fa0";
  currentColour = colour;

  // Convert it to a long
  long number = strtol(colour.substring(1).c_str(), NULL, 16);

  // Split them up into r, g, b values
  long r = number >> 16;
  long g = number >> 8 & 0xFF;
  long b = number & 0xFF;

  Serial.println("R: " + String(r) + ", G: " + String(g) + ", B: " + String(b));

  long rgb[] = { r, g, b };
  for (int i = 0; i < sizeof(rgb) / sizeof(rgb[0]); i++) {
    if (rgb[i] <= 0) {
      rgb[i] = 255;
    } else {
      rgb[i] *= -1;
    }
  }

  digitalWrite(powerPin, HIGH);
  power = true;
  analogWrite(rPin, rgb[0]);
  analogWrite(gPin, rgb[1]);
  analogWrite(bPin, rgb[2]);
}

void TeleOp() {
  setColour(gold);
  if (millis() - previousMillis >= 700) {
    previousMillis = millis();
    if (currentColour.equals(gold)) {
      setColour(green);
    } else {
      setColour(gold);
    }
  }
}

void Auto() {
  setColour(gold);
  if (millis() - previousMillis >= 450) {
    previousMillis = millis();
    if (currentColour.equals(gold)) {
      setColour(green);
    } else {
      setColour(gold);
    }
  }
}

void EndOfMatch() {
  setColour(gold);
  if (millis() - previousMillis >= 700 && power) {
    previousMillis = millis();
    digitalWrite(powerPin, LOW);
    power = false;
  }
  if (millis() - previousMillis >= 200 && !power) {
    previousMillis = millis();
    digitalWrite(powerPin, HIGH);
    power = true;
  }
}

void EStop() {
  setColour(red);
  if (millis() - previousMillis >= 500) {
    previousMillis = millis();
    if (power) {
      digitalWrite(powerPin, LOW);
      power = false;
    } else {
      digitalWrite(powerPin, HIGH);
      power = true;
    }
}

void ClimbCorrectAngle() {
  setColour("#00FF00");
}