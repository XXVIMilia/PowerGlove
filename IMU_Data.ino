#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (20)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
//  Serial.println("------------------------------------");
//  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
//  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
//  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
//  Serial.println("------------------------------------");
//  Serial.println("");
  delay(200);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
//  Serial.println("");
//  Serial.print("System Status: 0x");
//  Serial.println(system_status, HEX);
//  Serial.print("Self Test:     0x");
//  Serial.println(self_test_results, HEX);
//  Serial.print("System Error:  0x");
//  Serial.println(system_error, HEX);
//  Serial.println("");
  delay(200);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  //Serial.print("\t");
  if (!system)
  {
    //Serial.print("! ");
  }
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
sensors_event_t  accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  int8_t boardTemp = bno.getTemp();

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
       
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  
    //Serial.print("Accl:");
    x = accelerometerData.acceleration.x;
    y = accelerometerData.acceleration.y;
    z = accelerometerData.acceleration.z;
 
//  Serial.print("x_accel= ");
//  Serial.print(x);
//  Serial.print(" |\ty_accel= ");
//  Serial.print(y);
//  Serial.print(" |\tz_Accel= ");
//  Serial.println(z);
  double accell[]={x,y,z};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  //Serial.print("X_oriet: ");
  //x_oreint = map(event.orientation.x, 0, 
//  Serial.print(event.orientation.x, 4);
//  Serial.print("\tY_oriet: ");
//  Serial.print(event.orientation.y, 4);
//  Serial.print("\tZ_oriet: ");
  int z_oreint;
  if(event.orientation.z <0){
     z_oreint = map(event.orientation.z, -360, -0.001, 180, 360);
  }
  else{
    z_oreint = event.orientation.z;
  }
// z_oreint = map(z_oreint, 0, 360, 0, 255);
//int  y_orient = map(event.orientation.y, -360, 360, 99,99.1);
//  int x_orient = map(event.orientation.x, 0, 360, 0, 255);

  
  //Serial.print(z_oreint);
  //double arrayy[6]={x_orient, y_orient, z_oreint, x,y,z};
  
    //Serial.println(x_orient, y_orient, z_oreint, x,y,z);
    
  String x_ori = String(event.orientation.x);
  String y_ori = String(event.orientation.y);
  String z_ori = String(z_oreint);
  String x_acc = String(x);
  String y_acc = String(y);
  String z_acc = String(z);

 String outputt = x_ori+"!" +y_ori+"!" +z_ori+"!" +x_acc+"!" + y_acc+"!" +z_acc;
 Serial.println(outputt);
  /* Optional: Display calibration status */
  displayCalStatus();
  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();
  /* New line for the next sample */
  //Serial.println("");
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

//void printEvent(sensors_event_t* event) {
//  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//    //Serial.print("Accl:");
//    x = event->acceleration.x;
//    y = event->acceleration.y;
//    z = event->acceleration.z;
//  }
//
//  Serial.print("x_accel= ");
//  Serial.print(x);
//  Serial.print(" |\ty_accel= ");
//  Serial.print(y);
//  Serial.print(" |\tz_Accel= ");
//  Serial.println(z);
//  float accel[]={x,y,z};
//  return(accel);
//}
