
// MKR Family boards available attach interrupt pins - 0, 1, 4, 5, 6, 7, 8, 9, A1, A2

// Physical Pin conversions - Drone
// Uno      MKR1010   Function
// 4        16        ESC1 - FR > CCW
// 5        17        ESC2 - RR > CW
// 6        18        ESC2 - RL > CCW
// 7        19        ESC2 - FL > CW
// 8        A1        CH1 - Roll
// 9        A2        CH2 - Throttle
// 10       0         CH3 - Pitch
// 11       7         CH4 - Yaw
// 12       2         Status LED 1 - Errors, calibration
// 13       3         Status LED 2 - TBD
// A0       A0        Battery Monitor from Voltage Divider
// SDA      SDA       Data with MPU, Barometer
// SLC      SLC       Clock with MPU, Barometer

#include <global.h>
#include <base64.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WebSocketClient.h>
#include <WebSocketServer.h>
#define _WIFININA_LOGLEVEL_       1
#include <WiFiNINA_Generic.h>
#include "arduino_secrets.h"

// global var
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
const byte IPLastByte  = 99;
const short webPort     = 80;
const short socketPort  = 8080;

// OBJECTS
// laser servo objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// WiFi
WiFiServer      webServer(webPort);
WiFiServer      socketServer(socketPort);
WebSocketServer webSocketServer;
WiFiClient      socketClient;

// Console Attached
#ifndef TerminalAttached
    // true = terminal attached (send serial messages), false = no terminal attached no messages 
    #define TerminalAttached  true
#endif


// PIN ASSIGNMENTS
// Battery Monitor
#ifndef VoltageMonitorPin
    // A6 / D21
    #define VoltageMonitorPin  A6
#endif
#ifndef LowVoltageCutoff
    // the value we want keep the system from starting and/or when to shutdown
    #define LowVoltageCutoff  10.5
#endif
#ifndef LowVoltagePin
    // D6 - Red LED
    #define LowVoltagePin  7
#endif


// Motor Control
#ifndef Rec5_tank_lr
    // A1 - Receiver output 5
    #define Rec5_tank_lr  A1
#endif
#ifndef Rec1_tank_speed
    // A2 - Receiver output 1
    #define Rec1_tank_speed  A2
#endif
#ifndef Rec4_move_laser_ud
    // 0 - Receiver output 4
    #define Rec4_move_laser_ud  0
#endif
#ifndef Rec2_move_laser_lr
    // 7 - Receiver output 2
    #define Rec2_move_laser_lr  7
#endif
#ifndef MRen1
    // A3/D18 - Motor 1 PWM
    #define MRen1  18
#endif
#ifndef MRin1
    // 1 - Motor 1 Direction 1
    #define MRin1  1
#endif
#ifndef MRin2
    // 2 - Motor 1 Direction 2
    #define MRin2  2
#endif
#ifndef MLen1
    // A4/D19 - Motor 2 PWM
    #define MLen1  19
#endif
#ifndef MLin1
    // 3 - Motor 1 Direction 1
    #define MLin1  3
#endif
#ifndef MLin2
    // 4 - Motor 2 Direction 2
    #define MLin2  4
#endif
#ifndef TankSwitchTime
    // Delay time to Switch Direction, allow motors to slow down first so they are not hammered by the direction switch
    #define TankSwitchTime  500
#endif
#ifndef TrimAdjustment
    // This is the trim adjustment to compensate for the different motor speeds
    #define TrimAdjustment  3
#endif


// Laser
#ifndef LASER_LR_SERVO_MIN_PULSE
    // PCA9685 minimum pulse with to servos
    #define LASER_LR_SERVO_MIN_PULSE   130
#endif
#ifndef LASER_LR_SERVO_MAX_PULSE
    // PCA9685 minimum pulse with to servos
    #define LASER_LR_SERVO_MAX_PULSE   540
#endif
#ifndef LASER_UD_SERVO_MIN_PULSE
    // PCA9685 maximum pulse with to servos
    #define LASER_UD_SERVO_MIN_PULSE   110
#endif
#ifndef LASER_UD_SERVO_MAX_PULSE
    // PCA9685 maximum pulse with to servos
    #define LASER_UD_SERVO_MAX_PULSE   300
#endif
#ifndef SERVO_FREQUENCY
    // PCA9685 frequency of pulses to servos
    #define SERVO_FREQUENCY 50
#endif
#ifndef SERVO_OSCILLATORFREQUENCY
    // PCA9685 oscillator frequency
    #define SERVO_OSCILLATORFREQUENCY 25000000
#endif
#ifndef LASERDIGITALPWM
    // D5 - PWM Out to TTL of Laser
    #define LASERDIGITALPWM  5
#endif
#ifndef LASERSTARTLR
    // the center of the left/right position
    #define LASERSTARTLR 330
#endif
#ifndef LASERSTARTUD
    // the center of the left/right position
    #define LASERSTARTUD 110
#endif


// ISR
// call ISR - TC4_Handler 20000 times per second
// an interrupt is called every 50 microseconds so to get:
// count 1 interrupts = 50us
// count 20 interrupts = 1ms
// count 100 interrupts = 5ms
// count 200 interrupts = 10ms
// count 20000 interrupts = 1s
// count 60000 interrupts = 3s
// count 100000 interrupts = 5s
#ifndef ISR_1MSECS
    // number of 50usecs we need to get 1msec for the laser fire length,of the user defined time in msecs
    #define ISR_1MSECS 20
#endif
#ifndef ISR_5MSECS
    // pulse the laser every 5 milliseconds
    #define ISR_5MSECS 100
#endif
#ifndef ISR_1SECS
    // check the battery
    #define ISR_1SECS 20000
#endif
#ifndef ISR_3SECS
    // check the battery
    #define ISR_3SECS 60000
#endif
#ifndef ISR_5SECS
    #define ISR_5SECS 100000
#endif


// Motor
// true = forward, false =  reverse
boolean tankdirection = true;


// Laser servos
int LaserLR = 0;
int LaserUD = 1;


// Battery
short raw_read;
byte BatteryAverageCount = 0;
float BatteryVoltage = 0;
float BatteryAverageBuild = 0;
float BatteryAverageFinal = 0;


// Background
boolean Backgroundinit = true;
boolean BackgroundHearBeat = false;


// ISR vars
volatile int ISR_HeartBeat = 0;
volatile int ISR_BatteryVoltage = 0;
volatile boolean ISR_LaserOn = false;
volatile int ISR_LaserTargetCount = 0;
volatile boolean ISR_LaserFire = false;
volatile short Laserlrpos = LASERSTARTLR;
volatile short Laserudpos = LASERSTARTUD;

// Receiver Processing
unsigned long current_time_1, current_time_2, current_time_3, current_time_4;
int receiver_input[5];


// remote control webpage, gzipped and base64 encoding to save space
char webpage_base64[] = "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";


// move the select servo (SeroNum) to the requested position (pulseSize)
void moveLaser( int ServoNum, int pulseSize)
{
  //Control Motor
  pwm.setPWM(ServoNum, 0, pulseSize);
}

// set the motor direction
// Forward
// MRin1 = LOW      MLin1 = LOW
// MRin2 = HIGH     MLin2 = HIGH
// Reverse
// MRin1 = HIGH     MLin1 = HIGH
// MRin2 = LOW      MLin2 = LOW
void setMotorDirection(boolean tankdirection)
{
    if(tankdirection) {
        // Forward
        digitalWrite(MRin1, LOW);
        digitalWrite(MRin2, HIGH);
        digitalWrite(MLin1, LOW);
        digitalWrite(MLin2, HIGH);
    } else {
        // Reverse
        digitalWrite(MRin1, HIGH);
        digitalWrite(MRin2, LOW);
        digitalWrite(MLin1, HIGH);
        digitalWrite(MLin2, LOW);
    }
}

// stop the tank
void stopTank()
{
    analogWrite(MRen1, 0); // Motor 1 - Send PWM signal to L293D Enable pin
    analogWrite(MLen1, 0); // Motor 2 - Send PWM signal to L293D Enable pin
}

// set the motor speed
// receiver_input[2] = 1000 - 2000 , 1500 is mid range WITH motor PWM range of 0 - 250
// 1000 - full reverse
// 1400 - 1600 stop
// 2000 - full forward
void setMotorSpeed()
{
    int tempSpeed; // get a snap shot of the tanks speed
    tempSpeed = receiver_input[2];

    // see if we are stopped
    if((tempSpeed > 1450 && tempSpeed < 1550) || tempSpeed <= 0) {
        stopTank();
    } else {
        // set the direction
        // see if we are going forward or reverse
        if(tempSpeed >= 1550) {
            // are we already going reverse
            if(!tankdirection) {
                // first stop the tank
                stopTank();
                delay(TankSwitchTime);

                // second set the direction to forward
                tankdirection = true;
                setMotorDirection(tankdirection);
            }
            // map(value, fromLow, fromHigh, toLow, toHigh)
            tempSpeed = map(tempSpeed, 1550, 1900, 50, 250);
        } else {
            // are we already going forward
            if(tankdirection) {
                // first stop the tank
                stopTank();
                delay(TankSwitchTime);

                // second set the direction to reverse
                tankdirection = false;
                setMotorDirection(tankdirection);
            }
            // map(value, fromLow, fromHigh, toLow, toHigh)
            tempSpeed = map(tempSpeed, 1080, 1450, 250, 50);
        }

        analogWrite(MLen1, tempSpeed);                        // Motor 2 - Send PWM signal to L293D Enable pin
        analogWrite(MRen1, (tempSpeed - TrimAdjustment));    // Motor 1 - Send PWM signal to L293D Enable pin
        
        if(TerminalAttached) {
          Serial.println("MLen1 " + String(tempSpeed));
          Serial.println("MRen1 " + String((tempSpeed - TrimAdjustment)));
        }  
    }
}

void readBatteryStatus()
{ 
    // read the Battery Voltage
    raw_read = analogRead(VoltageMonitorPin);
    BatteryVoltage = (((float)raw_read) * 12.6 / 4095);
}

void sendBatteryStatus()
{ 
    // read the Battery Voltage
    readBatteryStatus();
    BatteryAverageBuild += BatteryVoltage;

    // make the average
    // average is the last 10 samples
    BatteryAverageCount++;
    if(BatteryAverageCount == 10) {
        BatteryAverageFinal = (BatteryAverageBuild / 10);
        BatteryAverageBuild = BatteryAverageCount = 0;
      
        // send it to the client
        webSocketServer.sendData("S:C = " + String(BatteryVoltage) + "v, A = " + String(BatteryAverageFinal) + "v, Raw = " + String(raw_read));

        // send an error message if the battery is below the error threshold
        if(BatteryAverageFinal <= LowVoltageCutoff) {
            // turn on Low Battery Light
            digitalWrite(LowVoltagePin, HIGH);
            webSocketServer.sendData("E:LOW BATTERY, Please Shout Down Now!");  
        } else {
            // turn off Low Battery Light
            digitalWrite(LowVoltagePin, LOW);
        }
    }
}

void printWifiStatus() 
{
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("Signal strength (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("Gateway: "); Serial.println(WiFi.gatewayIP());
    Serial.print("Netmask: "); Serial.println(WiFi.subnetMask());
    Serial.print("Webpage is at http://"); Serial.print(WiFi.localIP()); Serial.println("/");
    Serial.print("Websocket is at http://"); Serial.print(WiFi.localIP()); Serial.println(":" + (String)socketPort + "/");
}

void WiFiConnect() 
{
  while (WiFi.status() != WL_CONNECTED) 
  {
    if(TerminalAttached) {
        Serial.println("Connecting to " + (String)ssid + " ...");
    }
    WiFi.begin(ssid, pass);
    delay(5000);
  }
  
  IPAddress IP = WiFi.localIP();
  IP[3] = IPLastByte;
  
  WiFi.config(IP, WiFi.gatewayIP(), WiFi.gatewayIP(), WiFi.subnetMask());
  if(TerminalAttached) {
    Serial.println("Connected to " + (String)ssid);
  }
  
  webServer.begin();
  socketServer.begin();
  if(TerminalAttached) {
    printWifiStatus();
  }
  WiFi.lowPowerMode();
}

//Read the move tank left or right
void read_ch5_move_tank_lr() {
  unsigned long temp_1;
    
  if(digitalRead(Rec5_tank_lr) == HIGH) {
    current_time_1 = micros();   
  } else {
    temp_1 = micros() - current_time_1;  
    if(temp_1 >= 990 && temp_1 <= 2010) {
      if(temp_1 < 1000) {
        temp_1 = 1000;
      } else if (temp_1 > 2000) {
        temp_1 = 2000;
      }
      receiver_input[1] = temp_1;   
    }
  }
}

//Read Tank Speed
void read_ch1_tank_speed() {
  unsigned long temp_2;
    
  if(digitalRead(Rec1_tank_speed) == HIGH) {
    current_time_2 = micros();   
  } else {
    temp_2 = micros() - current_time_2;  
    if(temp_2 >= 990 && temp_2 <= 2010) {
      if(temp_2 < 1000) {
        temp_2 = 1000;
      } else if (temp_2 > 2000) {
        temp_2 = 2000;
      }
      receiver_input[2] = temp_2;   
    }
  }
}

//Read the Laser U/D
void read_ch4_move_laser_ud() {
  unsigned long temp_3;
    
  if(digitalRead(Rec4_move_laser_ud) == HIGH) {
    current_time_3 = micros();   
  } else {
    temp_3 = micros() - current_time_3;  
    if(temp_3 >= 990 && temp_3 <= 2010) {
      if(temp_3 < 1000) {
          temp_3 = 1000;
        } else if (temp_3 > 2000) {
          temp_3 = 2000;
        }
        receiver_input[3] = temp_3;   
     }
  }
}

//Read the Laser l/R
void read_ch2_move_laser_lr() {
  unsigned long temp_4;
    
  if(digitalRead(Rec2_move_laser_lr) == HIGH) {
    current_time_4 = micros();   
  } else {
    temp_4 = micros() - current_time_4;  
    if(temp_4 >= 990 && temp_4 <= 2010) {
      if(temp_4 < 1000) {
        temp_4 = 1000;
      } else if (temp_4 > 2000) {
        temp_4 = 2000;
      }
      receiver_input[4] = temp_4;   
    }
  }
}

// Start MKR1010 software interrupt functions **********
void setup_timer4()
{
    // Set up the generic clock (GCLK4) used to clock timers
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);              // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |           // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |     // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);            // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Feed GCLK4 to TC4 and TC5
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |       // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;       // Feed the GCLK4 to TC4 and TC5
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT8;          // Set the counter to 8-bit mode
    while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

    REG_TC4_COUNT8_CC0 = 150;                       // Set the TC4 CC0 value calculated for 50usec
    while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

    NVIC_SetPriority(TC4_IRQn, 0);                  // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
    NVIC_EnableIRQ(TC4_IRQn);                       // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

    REG_TC4_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
    REG_TC4_INTENSET = TC_INTENSET_OVF;             // Enable TC4 interrupts

    // value needed for 50usec isr
    uint16_t prescale=TC_CTRLA_PRESCALER(4);
    Serial.println("prescale " + (String)prescale);

    REG_TC4_CTRLA |= prescale | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_ENABLE;  // Enable TC4
    while (TC4->COUNT8.STATUS.bit.SYNCBUSY);                              // Wait for synchronization
}

// Interrupt Service Routine (ISR)
void TC4_Handler()
{
  // check for overflow (OVF) interrupt
  if (TC4->COUNT8.INTFLAG.bit.OVF && TC4->COUNT8.INTENSET.bit.OVF)
  {

    // see if Targeting is active and/or we were asked to Fire the Laser
    if(ISR_LaserFire) {
        // Fire
        digitalWrite(LASERDIGITALPWM, HIGH);
    } else if (ISR_LaserOn) {
        // Targeting
        // generate a 50 microsecond digital pulse every 5 milliseconds
        ISR_LaserTargetCount++;
        if(ISR_LaserTargetCount == 1) {
            digitalWrite(LASERDIGITALPWM, HIGH);
        }
        // 50 microseconds each interrupt
        if(ISR_LaserTargetCount == 2) {
            digitalWrite(LASERDIGITALPWM, LOW);
        }
        // do this every 5 milliseconds
        if(ISR_LaserTargetCount == ISR_5MSECS) {
            ISR_LaserTargetCount = 0;
        }
    } else {
        digitalWrite(LASERDIGITALPWM, LOW);
    }
    
    // see if it is time to send the battery status - every 3 seconds
    // if so tell the background to do it
    if(ISR_HeartBeat < ISR_1SECS) {
      ISR_HeartBeat++;
    }

    // see if it is time to send the battery status - every 3 seconds
    // if so tell the background to do it
    if(ISR_BatteryVoltage < ISR_3SECS) {
      ISR_BatteryVoltage++;
    }

    // clear interrupt - clear the MC1 interrupt flag
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;
  }
}
// End MKR1010 software interrupt functions **********

void setup()
{
    // we use this led to show the background is running
    // flased on and off every time we go through the background loop
    pinMode(LED_BUILTIN, OUTPUT);

    // setup the low voltage pin
    pinMode(LowVoltagePin, OUTPUT);
    digitalWrite(LowVoltagePin, LOW);

    // Digital PWM Pin
    pinMode(LASERDIGITALPWM, OUTPUT);
    digitalWrite(LASERDIGITALPWM, LOW); // No PWM

    // set up the Motor Pins
    // Motor 1
    pinMode(MRen1, OUTPUT);
    pinMode(MRin1, OUTPUT);
    pinMode(MRin2, OUTPUT);
    // Motor 2
    pinMode(MLen1, OUTPUT);
    pinMode(MLin1, OUTPUT);
    pinMode(MLin2, OUTPUT);
    // stop the tank and set the direction
    stopTank();
    setMotorDirection(tankdirection);

    // Start up the laser servos
    pwm.begin();
    pwm.setOscillatorFrequency(SERVO_OSCILLATORFREQUENCY);
    pwm.setPWMFreq(SERVO_FREQUENCY);
    delay(10);

    // set left/right laser servo to center
    moveLaser(LaserLR, Laserlrpos);
    delay(100);
    // set up/down laser servo to down
    moveLaser(LaserUD, Laserudpos);
    delay(100);
    
    // done initilization so start the interrupts
    // call ISR - TC4_Handler 20000 times per second
    // an interrupt is called every 50 microseconds
    setup_timer4();
  
    //Set up Pin Change Interrupts
    attachInterrupt(digitalPinToInterrupt(Rec5_tank_lr), read_ch5_move_tank_lr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Rec1_tank_speed), read_ch1_tank_speed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Rec4_move_laser_ud), read_ch4_move_laser_ud, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Rec2_move_laser_lr), read_ch2_move_laser_lr, CHANGE);

    // Serial port initialization
    if(TerminalAttached) {
        // Start the serial communication
        Serial.begin(57600);
        delay(100);

        Serial.println("\nStart MultiServers");
        Serial.println("Version " + String(WIFININA_GENERIC_VERSION));

        // check and make sure we are using the lastest Firmware
        String fv = WiFi.firmwareVersion();
        if (fv < WIFI_FIRMWARE_LATEST_VERSION)
        {
            Serial.print("Your current firmware NINA FW v");
            Serial.println(fv);
            Serial.print("Please upgrade the firmware to NINA FW v");
            Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
        }    
    }
           
    // Get the Initial Battery Status so we can preset the battery average until we have one (every 30 seconds)
    // also check and make sure we are good to go else set the Low Voltage LED and wait
    analogReadResolution(12);
    while(BatteryAverageFinal <= LowVoltageCutoff) {
      readBatteryStatus();

      // check to make sure we got enough battery to continue
      if(BatteryVoltage <= LowVoltageCutoff) {
        // turn on Low Battery Light and do nothing else
        digitalWrite(LowVoltagePin, HIGH);
      } else {
          BatteryAverageFinal = BatteryVoltage;
          digitalWrite(LowVoltagePin, LOW);
      }
   }

   // setup complete
   digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
    if(WiFi.status() != WL_CONNECTED)
    {
        if(TerminalAttached) {
            Serial.println("Lost WiFi connection");
        }
        WiFi.end();
        WiFiConnect();
    }

    WiFiClient webClient = webServer.available();
  
    if(webClient.connected())
    {
        if(TerminalAttached) {  
            Serial.print("New client: "); Serial.print(webClient.remoteIP()); Serial.print(":"); Serial.println(webClient.remotePort());
        }
        String header = "";
  
        while(webClient.available())
        {
            char ch = webClient.read();
  
            if (ch != '\r') 
            {
            header += ch;
            }
        
            if (header.substring(header.length() - 2) == "\n\n") 
            {
            if(TerminalAttached) {  
                Serial.print(header.substring(0, header.length() - 1));
            }
          
            if (header.indexOf("GET / HTTP") > -1) 
            {
                const int webpage_base64_length = sizeof(webpage_base64);
                const int webpage_gz_length = base64_dec_len(webpage_base64, webpage_base64_length);
                char webpage_gz[webpage_gz_length];
                base64_decode(webpage_gz, webpage_base64, webpage_base64_length);
                int packetsize = 1024;
                int done = 0;
                
                webClient.println("HTTP/1.1 200 OK\nContent-Type: text/html\nContent-Encoding: gzip\n");
            
                while (webpage_gz_length > done) 
                {
                    if (webpage_gz_length - done < packetsize) {
                
                    packetsize = webpage_gz_length - done;
                    }
              
                    webClient.write(webpage_gz + done, packetsize * sizeof(char));
                    done = done + packetsize;
                }
                if(TerminalAttached) {
                    Serial.println("--Interface webpage sent");
                }
            } 
            else 
            {
                webClient.println("HTTP/1.1 404 Not Found\nContent-Type: text/plain; charset=utf-8\n\n404 Not Found\n");
                if(TerminalAttached) {
                    Serial.println("--Page not found");
                }
            }
          
            webClient.stop();
            if(TerminalAttached) {
                Serial.println("--Client disconnected");
            }
        }
      }
    }

    if(!socketClient.connected()) {
        socketClient = socketServer.available();
        if (socketClient.connected() && webSocketServer.handshake(socketClient))  {
            if(TerminalAttached) {
                Serial.print("\n--Websocket connected to: ");
                Serial.print(socketClient.remoteIP());
                Serial.print(":");
                Serial.println(socketClient.remotePort());
            }
        } else {
            socketClient.stop();
            delay(100);
        }
    }

    if(socketClient.connected())  {
        // Background Init - setup the background tasks, runs only once
        if(Backgroundinit == true) {
            Backgroundinit = false;
            String data = webSocketServer.getData();
            if(TerminalAttached) {
                Serial.println("Websocket Flushed");
            }
        }

        // Background Process 1
        // see if we have a command/request from the user
        String data = webSocketServer.getData();
        if (data.length() > 0)
        {
            String cmd = data.substring(0, data.indexOf(":"));
            String setting = data.substring(data.indexOf(":") + 1);

            // process command
            switch (cmd.toInt()) {
                case 1:
                    // this is button #1
                    webSocketServer.sendData("R:Button #1");
                break;
                case 2:
                    // this is the -Battery+ button
                    webSocketServer.sendData("R:CBattery = " + String(BatteryVoltage) + "V, ABattery = " + String(BatteryAverageFinal) + "V, Raw = " + String(raw_read));
                break;
                case 3:
                    // process the command
                    if (setting.charAt(0) == 'D') {
                        webSocketServer.sendData("D:receiver1: " + String(receiver_input[1]));
                        webSocketServer.sendData("D:receiver2: " + String(receiver_input[2]));
                        webSocketServer.sendData("D:receiver3: " + String(receiver_input[3]));
                        webSocketServer.sendData("D:receiver4: " + String(receiver_input[4]));
                        webSocketServer.sendData("D:EndDebug:");
                    }
                break;
                default:
                    webSocketServer.sendData("E:" + cmd + " - " + setting);
                break;
            }
        }

        // Background Process 2
        // see if it's time to send the battery status
        // the function will send a status update every 10 times we call it so we send the status every 30 seconds
        if(ISR_BatteryVoltage == ISR_3SECS) {
          ISR_BatteryVoltage = 0;
          sendBatteryStatus();
        }

        // Background Process 3
        // show the background heart
        if(ISR_HeartBeat == ISR_1SECS) {
            BackgroundHearBeat = !BackgroundHearBeat;
            if(BackgroundHearBeat) {
                digitalWrite(LED_BUILTIN, LOW);
            } else {
                digitalWrite(LED_BUILTIN, HIGH);
                if(TerminalAttached) {
                    Serial.println("");
                    Serial.println("Receiver 1 " + String(receiver_input[1]));
                    Serial.println("Receiver 2 " + String(receiver_input[2]));
                    Serial.println("Receiver 3 " + String(receiver_input[3]));
                    Serial.println("Receiver 4 " + String(receiver_input[4]));
                    //setMotorSpeed();
                }
            }
            ISR_HeartBeat = 0;
        }

        // Background Process 4
        // process the receiver outputs
        // set the speed
        setMotorSpeed();


    }
}
