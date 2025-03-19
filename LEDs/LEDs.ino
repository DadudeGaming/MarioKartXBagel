// #include <string>

int powerPin = 3, rPin = 9, gPin = 10, bPin = 11;

String gold = "#FFD700";
String green = "#61B077";
String red = "#FF0000";
String white = "#FFFFFF";
String currentColour = "";

bool power = false;

int numberCode = -1;

unsigned long previousMillis = 0;

bool fading = false;
String fadeFrom;
String fadeTo;
int fadeSteps = 0;
int currentFadeStep = 0;
unsigned long fadeStepDelay = 0;
unsigned long fadeLastUpdate = 0;

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
  if (Serial.available()) {
    String serialString = Serial.readString();
    numberCode = serialString.toInt();
  }

  if (numberCode == 0) {
    Disabled();
  } else if (numberCode == 1) {
    TeleOp();
  } else if (numberCode == 2) {
    Auto();
  } else if (numberCode == 3) {
    EndOfMatch();
  } else if (numberCode == 4) {
    EStop();
  } else if (numberCode == 5) {
    ClimbCorrectAngle();
  }
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

void startFade(String fromColour, String toColour, int steps, int stepDelay) {
  fadeFrom = fromColour;
  fadeTo = toColour;
  fadeSteps = steps;
  fadeStepDelay = stepDelay;
  currentFadeStep = 0;
  fadeLastUpdate = millis();
  fading = true;
}

// This function should be called frequently (in loop) to update the fade.
void updateFade() {
  if (!fading) return;

  if (millis() - fadeLastUpdate >= fadeStepDelay) {
    
    long fromNumber = strtol(fadeFrom.substring(1).c_str(), NULL, 16);
    int fromR = fromNumber >> 16;
    int fromG = (fromNumber >> 8) & 0xFF;
    int fromB = fromNumber & 0xFF;

    long toNumber = strtol(fadeTo.substring(1).c_str(), NULL, 16);
    int toR = toNumber >> 16;
    int toG = (toNumber >> 8) & 0xFF;
    int toB = toNumber & 0xFF;

    float factor = (float)currentFadeStep / fadeSteps;
    int currentR = fromR + (toR - fromR) * factor;
    int currentG = fromG + (toG - fromG) * factor;
    int currentB = fromB + (toB - fromB) * factor;

    int rgb[3] = { currentR, currentG, currentB };
    for (int i = 0; i < 3; i++) {
      if (rgb[i] <= 0) {
        rgb[i] = 255;
      } else {
        rgb[i] = -rgb[i];
      }
    }

    digitalWrite(powerPin, HIGH);
    power = true;
    analogWrite(rPin, rgb[0]);
    analogWrite(gPin, rgb[1]);
    analogWrite(bPin, rgb[2]);

    currentFadeStep++;
    fadeLastUpdate = millis();


    if (currentFadeStep > fadeSteps) {
      fading = false;
      currentColour = fadeTo;
    }
  }
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
}

void Disabled() {
  updateFade();
  if (currentColour.equals(gold)) {
    startFade(gold, green, 20, 500);
  }
  if (currentColour.equals(green)) {
    startFade(green, white, 20, 500);
  }
  if (currentColour.equals(white)) {
    startFade(white, gold, 20, 500);
  }
}

void ClimbCorrectAngle() {
  setColour("#00FF00");
}