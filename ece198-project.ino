#include <Wire.h>            
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobot_BMX160.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Standard hardware I2C constructor (uses A4 and A5 automatically)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int trig = 7;
const int echo = 8;

DFRobot_BMX160 bmx160;

float lastDist = 0.0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Start the physical I2C hardware bus
  Wire.begin(); 
  delay(100);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Initialize OLED (Address 0x3C on shared hardware pins A4/A5)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("OLED allocation failed"));
    while(1);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("System Initializing...");
  display.display();
  delay(1000);

  // Initialize IMU (Also on shared hardware pins A4/A5)
  if (bmx160.begin() != true) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("BMX160 Init Failed!");
    display.println("Check A4/A5 split.");
    display.display();
    while(1);
  }

  // Prime the tracking metrics
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Speed * time / 2 (divide by 2 because travel there and back, 0.034 is cm/microsecond)
  lastDist = pulseIn(echo, HIGH) * 0.034 / 2;
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float timeDelta = (currentTime - lastTime) / 1000.0; 

  // 1. Measure Ultrasonic Distance
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH);
  float currentDist = dur * 0.034 / 2;

  // 2. Calculate Downward Velocity
  float velocity = 0.0;
  if (timeDelta > 0 && currentDist > 0 && lastDist > 0) {
    velocity = (lastDist - currentDist) / timeDelta; 
  }

  lastDist = currentDist;
  lastTime = currentTime;

  // 3. Fetch IMU Data from Shared Bus
  sBmx160SensorData_t Omagn, Ogyro, Oaccel;
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);

  // Calculate the squared horizontal components (x^2 + y^2)
  float xSquared = pow(Oaccel.x, 2);
  float ySquared = pow(Oaccel.y, 2);

  // Combine X and Y to get the total horizontal magnitude (hypotenuse)
  float horizontalMagnitude = sqrt(xSquared + ySquared);

  // Calculate the tilt angle in radians relative to the Z-axis
  float tiltAngleRad = atan2(horizontalMagnitude, Oaccel.z);

  // Convert the radian angle into degrees
  float tiltAngle = tiltAngleRad * 180.0 / PI;

  // 4. Update the OLED Screen Interface
  display.clearDisplay();
  
  display.setCursor(0, 0);
  display.print("TILT ANGLE: ");
  display.print(tiltAngle, 1);
  display.println(" deg");

  display.setCursor(0, 10);
  if (tiltAngle > 15.0) {  
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
    display.println(" !! EXCESSIVE LEAN !! ");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.println(" Position: Stable     ");
  }

  display.drawFastHLine(0, 24, 128, SSD1306_WHITE); 
  
  display.setCursor(0, 30);
  display.print("Current Dist: ");
  display.print(currentDist, 1);
  display.println(" cm");

  display.setCursor(0, 44);
  display.print("Tempo Vel: ");
  display.print(velocity, 1);
  display.println(" cm/s");

  display.setCursor(0, 56);
  if (velocity > 3.0) {
    display.println("Phase: -> DOWNWARD <-");
  } else if (velocity < -3.0) {
    display.println("Phase: <- UPWARD ->  ");
  } else {
    display.println("Phase: Static/Pause  ");
  }

  display.display(); 
  delay(150); 
}
