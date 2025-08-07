#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define NUMPIXELS 30
#define BUFFER_SIZE 10

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

char receive_cmd[BUFFER_SIZE];
uint8_t cmd_idx = 0;

bool robot_state = false;
bool led_state = false;
unsigned long prev_millis = 0;
const long interval = 500; // 500ms
bool led_on = false;

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.clear();
  pixels.show();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      receive_cmd[cmd_idx] = '\0';
      processCommand(receive_cmd);
      cmd_idx = 0;
    } else if (cmd_idx < BUFFER_SIZE - 1) {
      receive_cmd[cmd_idx++] = c;
    }
  }

  unsigned long curr_millis = millis();
  if (robot_state) { // ON
    if (led_state) { // FLASH
      if (curr_millis - prev_millis >= interval) {
        prev_millis = curr_millis;
        led_on = !led_on;
        updateLED(led_on);
      }
    } else { // SOLID
      updateLED(true);
    }
  } else { // OFF
    updateLED(false);
  }
}

void processCommand(const char* command) {
  if (strcmp(command, "ON") == 0) {
    robot_state = true;
  } else if (strcmp(command, "OFF") == 0) {
    robot_state = false;
  } else if (strcmp(command, "FLASH") == 0) {
    led_state = true;
  } else if (strcmp(command, "SOLID") == 0) {
    led_state = false;
  }
}

void updateLED(bool state) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, state ? pixels.Color(55, 0, 0) : pixels.Color(0, 0, 0));
  }
  pixels.show();
}
