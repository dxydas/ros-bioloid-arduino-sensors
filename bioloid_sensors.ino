// Hardware:
// Pololu A-Star 32U4 Mini SV
// Pololu MinIMU-9 v3 (L3GD20H and LSM303D)
// Interlink FSR 400 Short (x6)


// Important! Define this before #include <ros.h>
#define USE_USBCON

#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <AStar32U4Prime.h>
#include <LSM303.h>
#include <L3G.h>

// ROS node and publishers
ros::NodeHandle nh;
std_msgs::Int16MultiArray msg_fsrs;
std_msgs::MultiArrayDimension fsrsDim;
ros::Publisher pub_fsrs("fsrs", &msg_fsrs);
geometry_msgs::Vector3 msg_accel;
ros::Publisher pub_accel("accel", &msg_accel);
geometry_msgs::Vector3 msg_magnet;
ros::Publisher pub_magnet("magnet", &msg_magnet);
std_msgs::Float32 msg_heading;
ros::Publisher pub_heading("heading", &msg_heading);
geometry_msgs::Vector3 msg_gyro;
ros::Publisher pub_gyro("gyro", &msg_gyro);
unsigned long pubTimer = 0;

const int numOfFSRs = 6;
const int FSRPins[] = {A0, A1, A2, A3, A4, A5};
int FSRValue = 0;
LSM303 compass;
L3G gyro;

const int numOfBlueLEDs = 4;
const int blueLEDPins[] = {5, 6, 9, 10};
int blueLEDBright = 0;
int blueLEDDim = 5;
unsigned long blueLEDTimer = 0;

const int yellowLEDPin = IO_C7;  // 13
int yellowLEDBright = 0;
int yellowLEDDim = 5;
unsigned long yellowLEDTimer = 0;

const int numOfBarLEDs = 10;
const int barLEDPins[] = {0, 1, 4, 7, 8, 11, 12, 14, 16, 15};
int barLEDOn = 2;
unsigned long barLEDTimer = 0;
bool countUp = true;
int trailingLEDSign = -1;


void setup()
{
    delay(2000);
    Serial.begin(57600);

    // Blue LEDs used for lighting
    for (int i=0; i<numOfBlueLEDs; ++i)
    {
        pinMode(blueLEDPins[i], OUTPUT);
        analogWrite(blueLEDPins[i], 0);
    }
    
    // Yellow LED on A-Star pulses when ROS loop is working
    pinMode(yellowLEDPin, OUTPUT);
    
    // Bar LEDs used for status feedback
    for (int i=0; i<numOfBarLEDs; ++i)
    {
        pinMode(barLEDPins[i], OUTPUT);
        digitalWrite(barLEDPins[i], LOW);
    }
        
    // Array for FSRs
    msg_fsrs.layout.dim = &fsrsDim;
    msg_fsrs.layout.dim[0].label = "fsrs";
    msg_fsrs.layout.dim[0].size = numOfFSRs;
    msg_fsrs.layout.dim[0].stride = 1*numOfFSRs;
    msg_fsrs.layout.dim_length = 1;
    msg_fsrs.layout.data_offset = 0;
    msg_fsrs.data_length = numOfFSRs;
    msg_fsrs.data = (int16_t *)malloc(sizeof(int16_t)*numOfFSRs);

    nh.initNode();
    nh.advertise(pub_fsrs);
    nh.advertise(pub_accel);
    nh.advertise(pub_magnet);
    nh.advertise(pub_heading);
    nh.advertise(pub_gyro);
    
    // Wait until connected
    while (!nh.connected())
    {
        // Blink the bar LEDs
        if (millis() > barLEDTimer)
        {
            for (int i=0; i<numOfBarLEDs; ++i)
                digitalWrite(barLEDPins[i], LOW);

            digitalWrite(barLEDPins[barLEDOn+trailingLEDSign*2], HIGH);
            delay(50);
            digitalWrite(barLEDPins[barLEDOn+trailingLEDSign*1], HIGH);
            delay(50);
            digitalWrite(barLEDPins[barLEDOn], HIGH);
            
            if (countUp)
                barLEDOn = ++barLEDOn;
            else
                barLEDOn = --barLEDOn;

            if (barLEDOn > 9)
            {
                barLEDOn = 7;
                countUp = false;
                trailingLEDSign = 1;            
            }
            if (barLEDOn < 0)
            {
                barLEDOn = 2;
                countUp = true;
                trailingLEDSign = -1;
            }
            
            barLEDTimer = millis() + 150;
        }
        
        // Pulse the blue LEDs
        if (millis() > blueLEDTimer)
        {
            blueLEDBright += blueLEDDim;
            
            for (int i=0; i<numOfBlueLEDs; ++i)
                analogWrite(blueLEDPins[i], blueLEDBright);
            
            if ( (blueLEDBright == 0) || (blueLEDBright == 255) )
                blueLEDDim = -blueLEDDim;
            
            // 10 msec increments, 500 msec wait after each full cycle
            if (blueLEDBright != 0)
                blueLEDTimer = millis() + 10;
            else
                blueLEDTimer = millis() + 500;
        }
        
        nh.spinOnce();
    }
    nh.loginfo("ROS startup complete");
    
    // Blue LEDs on, about 75% brightness
    for (int i=0; i<numOfBlueLEDs; ++i)
        analogWrite(blueLEDPins[i], 191);
    
    // Bar LEDs off
    for (int i=0; i<numOfBarLEDs; ++i)
        digitalWrite(barLEDPins[i], LOW);
    
    Wire.begin();
    
    // Enable pullup resistors
    for (int i=0; i<numOfFSRs; ++i)
        pinMode(FSRPins[i], INPUT_PULLUP);
    
    if (!compass.init())
    {
        nh.logerror("Failed to autodetect compass type!");
    }
    compass.enableDefault();
    
    // Compass calibration values
    compass.m_min = (LSM303::vector<int16_t>){-3441, -3292, -2594};
    compass.m_max = (LSM303::vector<int16_t>){+2371, +2361, +2328};
    
    if (!gyro.init())
    {
        nh.logerror("Failed to autodetect gyro type!");
    }
    gyro.enableDefault();
    
    pubTimer = millis();
}


void loop()
{
    if (millis() > pubTimer)
    {
        for (int i=0; i<numOfFSRs; ++i)
        {        
            FSRValue = analogRead(FSRPins[i]);
            msg_fsrs.data[i] = FSRValue;
            delay(2);  // Delay (in msec) to allow ADC VRef to settle
        }
        
        compass.read();
        gyro.read();
        
        // Compass - accelerometer:
        // 16-bit, default range +-2 g, sensitivity 0.061 mg/digit
        // 1 g = 9.80665 m/s/s
        // e.g. value for z axis in m/s/s will be: compass.a.z * 0.061 / 1000.0 * 9.80665
        //      value for z axis in g will be: compass.a.z * 0.061 / 1000.0
        // Gravity is measured as an upward acceleration:
        // Stationary accel. shows +1 g value on axis facing directly "upwards"
        // Convert values to g
        msg_accel.x = (float)(compass.a.x)*0.061/1000.0;
        msg_accel.y = (float)(compass.a.y)*0.061/1000.0;
        msg_accel.z = (float)(compass.a.z)*0.061/1000.0;
        
        // Compass - magnetometer:
        // 16-bit, default range +-2 gauss, sensitivity 0.080 mgauss/digit
        msg_magnet.x = (float)(compass.m.x);
        msg_magnet.y = (float)(compass.m.y);
        msg_magnet.z = (float)(compass.m.z);
        // Heading from the LSM303D library is the angular difference in
        // the horizontal plane between the x axis and North, in degrees.
        // Convert value to rads, and change range to +-pi
        msg_heading.data = ( (float)(compass.heading())*M_PI/180.0 );
        
        // Gyro:
        // 16-bit, default range +-245 dps (deg/sec), sensitivity 8.75 mdps/digit
        // Convert values to rads/sec
        msg_gyro.x = (float)(gyro.g.x)*0.00875*M_PI/180.0;
        msg_gyro.y = (float)(gyro.g.y)*0.00875*M_PI/180.0;
        msg_gyro.z = (float)(gyro.g.z)*0.00875*M_PI/180.0;
        
        pub_fsrs.publish(&msg_fsrs);
        pub_accel.publish(&msg_accel);
        pub_magnet.publish(&msg_magnet);
        pub_heading.publish(&msg_heading);
        pub_gyro.publish(&msg_gyro);
        
        pubTimer = millis() + 10;  // wait at least 10 msecs between publishing
    }
    
    // Pulse the LED
    if (millis() > yellowLEDTimer)
    {
        yellowLEDBright += yellowLEDDim;
        analogWrite(yellowLEDPin, yellowLEDBright);
        
        if ( (yellowLEDBright == 0) || (yellowLEDBright == 255) )
            yellowLEDDim = -yellowLEDDim;
        
        // 50 msec increments, 2 sec wait after each full cycle
        if (yellowLEDBright != 0)
            yellowLEDTimer = millis() + 50;
        else
            yellowLEDTimer = millis() + 2000;
    }
    
    nh.spinOnce();
}

