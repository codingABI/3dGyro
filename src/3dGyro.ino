/*
 * Project: 3dGyro
 * Description: 3d object rotated by a gyro sensor MPU6050 on an Arduino Uno/Nano with a SSD1306 OLED 128x64 pixel display.
 * License: MIT License
 * 
 * created by codingABI https://github.com/codingABI/3dGyro
 * 
 * History:
 * 28.04.2022, Initial version
 */

#define DEBUG false  // true für Serial.print
#define SERIALDEBUG if (DEBUG) Serial

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <I2Cdev.h>

#include <MPU6050_6Axis_MotionApps20.h> // older, but smaller (~1k) binary than <MPU6050_6Axis_MotionApps612.h>

// 3d object (we use no ordered display list, so only simple convex 3d objects works)

// original points x,y,z
#define P1 0
#define P2 1
#define P3 2
#define P4 3
#define P5 4
#define P6 5
#define P7 6
#define P8 7
#define P9 8
#define P10 9
#define P11 10
#define P12 11
#define P13 12
#define P14 13
#define P15 14
#define P16 15
#define P17 16
#define P18 17
#define MAXPOINTS 14
const PROGMEM signed char points3d[MAXPOINTS][3] = {
  {32,-16,-32},
  {32,16,-32},
  {-32,16,-32},
  {-32,-16,-32},
  {32,-16,32},
  {32,16,32},
  {-32,16,32},
  {-32,-16,32},
  {90,0,32},
  {-90,0,32},
  {4,-4,-64},
  {4,4,-64},
  {-4,4,-64},
  {-4,-4,-64},
};

// build mesh from points (all polygons must be defined counterclockwise, otherwise hiding of backsides will not work)

// List of ordered points for triangles and quadrangles 
#define MAXTRIANGLES 8
const PROGMEM byte triangleList[MAXTRIANGLES][3] {
  {P1, P2, P9 },
  {P6, P5, P9 },
  {P2, P6, P9 },
  {P5, P1, P9 },
  {P3, P4, P10 },
  {P4, P8, P10 },
  {P8, P7, P10 },
  {P7, P3, P10 }
};

#define MAXQUADRANGLES 8
const PROGMEM byte quadrangleList[MAXQUADRANGLES][4] {
  { P14, P13, P12, P11 },
  { P14, P4, P3, P13 },
  { P13, P3, P2, P12 },
  { P12, P2, P1, P11 },
  { P11, P1, P4, P14 },
  { P5, P6, P7, P8 },
  { P2, P3, P7, P6 },
  { P4, P1, P5, P8 }
};

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// SSD1306 I2C 
#define OLED_RESET -1 // no reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu;

#define X 0
#define Y 1
#define Z 2

#define TYPE_TRIANGLE 2
#define TYPE_QUADRANGLE 1

// global variables
int viewerDistance = 200; // z-value for viewer/camera 
int viewerScale = 60; // 2d scale

// transformed points
signed char pointsTransformed3d[MAXPOINTS][3];

#define INTERRUPT_PIN 2
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// projection for x to 2d
int x3dTo2D (signed char x, signed char z) {
  if (z-viewerDistance != 0) {
    return (float) SCREEN_WIDTH/2 + viewerScale * x/(z-viewerDistance);
  } else return 0;
}

// projection for y to 2d
int y3dTo2D (signed char y, signed char  z) {
  if (z-viewerDistance != 0) {
    return (float) SCREEN_HEIGHT/2 - viewerScale * y/(z-viewerDistance);
  } else return 0;
}

// detect backsides for polygons (clockwise = backside, based on idea from https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order)
bool isBackface(byte type, int i) {
  long sum=0;
  switch (type) {
    case TYPE_TRIANGLE:
      for (byte j=0;j<3;j++) {
        sum+=(x3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][(j+1)%3]))][X],
          pointsTransformed3d[pgm_read_byte(&(triangleList[i][(j+1)%3]))][Z])-
          x3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][j]))][X],
          pointsTransformed3d[pgm_read_byte(&(triangleList[i][j]))][Z]))*
          (y3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][(j+1)%3]))][Y],
          pointsTransformed3d[pgm_read_byte(&(triangleList[i][(j+1)%3]))][Z])+
          y3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][j]))][Y],
          pointsTransformed3d[pgm_read_byte(&(triangleList[i][j]))][Z]));
      }
    break;
    case TYPE_QUADRANGLE:
      for (byte j=0;j<4;j++) {
        sum+=(x3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][X],
          pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][Z])-
          x3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][X],
          pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][Z]))*
          (y3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][Y],
          pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][Z])+
          y3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][Y],
          pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][Z]));
      }
    break;
  }
  // sum < 0 is counterclockwise in xy-coordinatesystem, but clockwise in screen-coordinatesystem
  // sum = 0 is when the polygon collaps just to a single line
  return (sum <= 0);
}

void setup(void) {
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

  pinMode(LED_BUILTIN,OUTPUT);

  SERIALDEBUG.begin(9600);

  //Start I2C
  Wire.begin();
  
  // MPU6050 init
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  if (!mpu.testConnection()) SERIALDEBUG.println(F("MPU6050 connection failed"));

  SERIALDEBUG.println(F("Initializing DMP..."));
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  // Calibration based on IMU_ZERO
  mpu.setXGyroOffset(1907);
  mpu.setYGyroOffset(130);
  mpu.setZGyroOffset(-1);
  mpu.setXAccelOffset(-647);
  mpu.setYAccelOffset(-3985);  
  mpu.setZAccelOffset(-4111);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    SERIALDEBUG.print(F("DMP Initialization failed (code "));
    SERIALDEBUG.print(devStatus);
    SERIALDEBUG.println(F(")"));
  }

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Default Address is 0x3D for 128x64, but my oled uses 0x3C 
    SERIALDEBUG.println(F( "SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Font settings
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
}

void loop(void) {
  char strData[7];
  static int pitch = 0;
  static int roll = 0;
  static int yaw = 0;
  signed char temp3d[2][3];
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint8_t rc;
  
  // clear display buffer
  display.clearDisplay();

  // get pitch, roll and yaw from MPU6050
  if (dmpReady) {
    // read a packet from FIFO
    digitalWrite(LED_BUILTIN,HIGH);
    rc = mpu.dmpGetCurrentFIFOPacket(fifoBuffer); 
    digitalWrite(LED_BUILTIN,LOW);

    if (rc) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      pitch = -ypr[1] * 180/M_PI;
      roll = -ypr[2] * 180/M_PI;
      yaw = -ypr[0] * 180/M_PI;
    }

  }

  for (byte i=0;i<MAXPOINTS;i++) {
    // rotate by x-axis
    temp3d[0][X] = (signed char)pgm_read_byte(&(points3d[i][X]));
    temp3d[0][Y] = (signed char)pgm_read_byte(&(points3d[i][Y]))*cos(radians(pitch)) - (signed char)pgm_read_byte(&(points3d[i][Z]))*sin(radians(pitch));
    temp3d[0][Z] = (signed char)pgm_read_byte(&(points3d[i][Z]))*cos(radians(pitch)) + (signed char)pgm_read_byte(&(points3d[i][Y]))*sin(radians(pitch)); 

    // rotate by y-axis
    temp3d[1][X] = temp3d[0][X]*cos(radians(yaw)) + temp3d[0][Z]*sin(radians(yaw));
    temp3d[1][Y] = temp3d[0][Y];
    temp3d[1][Z] = temp3d[0][Z]*cos(radians(yaw)) - temp3d[0][X]*sin(radians(yaw)); 

    // rotate by z-axis
    pointsTransformed3d[i][X] = temp3d[1][X]*cos(radians(roll)) - temp3d[0][Y]*sin(radians(roll));
    pointsTransformed3d[i][Y] = temp3d[1][X]*sin(radians(roll)) + temp3d[0][Y]*cos(radians(roll));
    pointsTransformed3d[i][Z] = temp3d[1][Z];
  }

  // draw triangles
  for (byte i=0;i<MAXTRIANGLES;i++) {
   if (!isBackface(TYPE_TRIANGLE,i)) { // only front sides  
    // draw outer border
    display.drawTriangle(
      x3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][0]))][X],pointsTransformed3d[pgm_read_byte(&(triangleList[i][0]))][Z]),
      y3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][0]))][Y],pointsTransformed3d[pgm_read_byte(&(triangleList[i][0]))][Z]),
      x3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][1]))][X],pointsTransformed3d[pgm_read_byte(&(triangleList[i][1]))][Z]),
      y3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][1]))][Y],pointsTransformed3d[pgm_read_byte(&(triangleList[i][1]))][Z]),
      x3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][2]))][X],pointsTransformed3d[pgm_read_byte(&(triangleList[i][2]))][Z]),
      y3dTo2D(pointsTransformed3d[pgm_read_byte(&(triangleList[i][2]))][Y],pointsTransformed3d[pgm_read_byte(&(triangleList[i][2]))][Z]),
      SSD1306_WHITE);
    }
  }
  // draw quadrangles
  for (byte i=0;i<MAXQUADRANGLES;i++) {
    if (!isBackface(TYPE_QUADRANGLE,i)) { // only front sides  
      // draw outer border
      for (byte j=0;j<4;j++) {
        display.drawLine(
          x3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][X],pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][Z]),
          y3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][Y],pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][j]))][Z]),
          x3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][X],pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][Z]),
          y3dTo2D(pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][Y],pointsTransformed3d[pgm_read_byte(&(quadrangleList[i][(j+1)%4]))][Z]), 
          SSD1306_WHITE);
      }
    }
  }

  snprintf(strData,7,"x%4d%c",pitch,(char)247);
  display.setCursor(0,35);
  display.println(strData);
  snprintf(strData,7,"y%4d%c",yaw,(char)247);
  display.setCursor(0,45);
  display.println(strData);
  snprintf(strData,7,"z%4d%c",roll,(char)247);
  display.setCursor(0,55);
  display.println(strData);

  // show display buffer on screen
  display.display();
}
