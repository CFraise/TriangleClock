#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiiChuck.h>

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

Accessory nunchuck;

#define BUTTON_A  0
#define BUTTON_B 16
#define BUTTON_C  2


void setup() {
  Serial.begin(115200);

  nunchuck.begin();
  if (nunchuck.type == Unknown) {
    nunchuck.type = NUNCHUCK;
  }
  

  Serial.println("128x64 OLED FeatherWing test");
  display.begin(0x3C, true); // Address 0x3C default

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);
  Serial.println("Button test");

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
}

void loop() {
  nunchuck.readData();
  updateScreen();
  yield();
  delay(50);
}

void updateScreen() {
  display.setCursor(0,0);
  display.clearDisplay();
  display.setTextSize(1);

  display.print("X: "); display.print(nunchuck.getAccelX());
  display.print(" Y: "); display.print(nunchuck.getAccelY()); 
  display.print(" Z: "); display.println(nunchuck.getAccelZ()); 

  display.setTextSize(2);
  display.print("("); 
  display.print(nunchuck.getJoyX());
  display.print(", "); 
  display.print(nunchuck.getJoyY());
  display.println(")");

  display.print("Buttons:\n"); 
  if (nunchuck.getButtonZ()) display.print("Z "); 
  if (nunchuck.getButtonC()) display.print("C1 ");

  if(!digitalRead(BUTTON_A)) display.print("A ");
  if(!digitalRead(BUTTON_B)) display.print("B ");
  if(!digitalRead(BUTTON_C)) display.print("C ");
  
  display.display();  
}
