#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

uint16_t errorsAndWarnings = 0;

LSM6DS3Core myIMU( I2C_MODE, 0x6B );
//LSM6DS3Core myIMU( SPI_MODE, 10 );

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  //Call .beginCore() to configure the IMU
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("Error at beginCore().\n");
  }
  else
  {
    Serial.print("\nbeginCore() passed.\n");
  }
  
  uint8_t dataToWrite = 0;  //Temporary variable

  //Setup the accelerometer******************************
  dataToWrite = 0; //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

  //Now, write the patched together data
  errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Set the ODR bit
  errorsAndWarnings += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

}


void loop(){
  int16_t temp;
  //Get all parameters
  Serial.print("\nAccelerometer Counts:\n");
  Serial.print(" X = ");
  
  //Read a register into the temp variable.
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.println(temp);
  Serial.print(" Y = ");

  //Read a register into the temp variable.
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.println(temp);
  Serial.print(" Z = ");

  //Read a register into the temp variable.
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.println(temp);
  
  Serial.println();
  Serial.print("Total reported Errors and Warnings: ");
  Serial.println(errorsAndWarnings);
  
  delay(1000);
}