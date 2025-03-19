// #include <string>

int powerPin = 3, rPin = 9, gPin = 10, bPin = 11;

String gold = "#FFC200";
String green = "#018900";
String red = "#FF0000";
String white = "#FFFFFF";
String currentColour = "";

bool power = false;

int numberCode = -1;

unsigned long previousMillis = 0;

bool fading = false;
String fadeFrom;
String fadeTo;

// steps between the two fade colours
int fadeSteps = 200;
// the delay between steps
unsigned long fadeStepDelay = 20;

int currentFadeStep = 0;
unsigned long fadeLastUpdate = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(powerPin, OUTPUT);
  pinMode(rPin, OUTPUT);
  pinMode(gPin, OUTPUT);
  pinMode(bPin, OUTPUT);

  Serial.begin(9600);

  digitalWrite(powerPin, LOW);
  power = false;

  // setColour(gold);
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
    // setColour(gold);
  } else if (numberCode == 2) {
    Auto();
    // setColour(green);
  } else if (numberCode == 3) {
    EndOfMatch();
    // setColour(white);
  } else if (numberCode == 4) {
    EStop();
    // setColour(red);
  } else if (numberCode == 5) {
    ClimbCorrectAngle();
  } else if (numberCode == 6) {
    Intake();
  } else {
    digitalWrite(powerPin, LOW);
    power = false;
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
      rgb[i] = -rgb[i];
    }
  }

  digitalWrite(powerPin, HIGH);
  power = true;
  analogWrite(rPin, rgb[0]);
  analogWrite(gPin, rgb[1]);
  analogWrite(bPin, rgb[2]);
}


void startFade(String fromColour, String toColour /*, int steps, int stepDelay*/) {
  fadeFrom = fromColour;
  fadeTo = toColour;
  // fadeSteps = steps;
  // fadeStepDelay = stepDelay;
  currentFadeStep = 0;
  fadeLastUpdate = millis();
  fading = true;
  currentColour = "fading";
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
  if (!currentColour.equals(gold) && !currentColour.equals(green)) {
    setColour(gold);
  }
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
  if (!currentColour.equals(gold) && !currentColour.equals(green)) {
    setColour(gold);
  }
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
  if (!currentColour.equals(gold)) {
    setColour(gold);
  }
  if (millis() - previousMillis >= 800 && power) {
    previousMillis = millis();
    digitalWrite(powerPin, LOW);
    power = false;
  }
  if (millis() - previousMillis >= 400 && !power) {
    previousMillis = millis();
    digitalWrite(powerPin, HIGH);
    power = true;
  }
}

void EStop() {
  if (!currentColour.equals(red)) {
    setColour(red);
  }
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
    startFade(gold, green);
  } else if (currentColour.equals(green)) {
    startFade(green, white);
  } else if (currentColour.equals(white)) {
    startFade(white, gold);
  } else if (currentColour.equals("fading")) {
  } else {
    setColour(gold);
    startFade(gold, green);
  }
}

void Intake() {
  if (!currentColour.equals(gold)) {
    setColour(gold);
  }
}

void ClimbCorrectAngle() {
  if (!currentColour.equals(green)) {
    setColour(green);
  }
}