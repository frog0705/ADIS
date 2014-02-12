/******************************************************ADIS16488*******************************************/
/*

*/

#define NSS 10
#define SCK 6
#define MOSI 4

#define OUTPUT_BAUD_RATE 115200
#define OUTPUT_DATA_INTERVAL 20 

#define ACCEL_X_MIN (-32767.0f)
#define ACCEL_X_MAX (32767.0f)
#define ACCEL_Y_MIN (-32767.0f)
#define ACCEL_Y_MAX (32767.0f)
#define ACCEL_Z_MIN (-32767.0f)
#define ACCEL_Z_MAX (32767.0f)

#define MAGN_X_MIN (-32767.0f)
#define MAGN_X_MAX (32767.0f)
#define MAGN_Y_MIN (-32767.0f)
#define MAGN_Y_MAX (32767.0f)
#define MAGN_Z_MIN (-32767.0f)
#define MAGN_Z_MAX (32767.0f)

#define GYRO_X_OFFSET (0.0f)
#define GYRO_Y_OFFSET (0.0f)
#define GYRO_Z_OFFSET (0.0f)

#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (0.8f)
#define ACCEL_Y_SCALE (0.8f)
#define ACCEL_Z_SCALE (0.8f)

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (0.1f)
#define MAGN_Y_SCALE (0.1f)
#define MAGN_Z_SCALE (0.1f)

#define GYRO_GAIN_X (0.02f)
#define GYRO_GAIN_Y (0.02f)
#define GYRO_GAIN_Z (0.02f)

#define GYRO_X_SCALE (TO_RAD(GYRO_GAIN_X))
#define GYRO_Y_SCALE (TO_RAD(GYRO_GAIN_Y))
#define GYRO_Z_SCALE (TO_RAD(GYRO_GAIN_Z))

#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

#define GRAVITY (1250.0f) 
#define TO_RAD(x) (x * 0.01745329252)  
#define TO_DEG(x) (x * 57.2957795131) 

float accel[3];  
float magnetom[3];
float gyro[3];

int prod;
float temperature;
float pressure;

// DCM variables
float MAG_Heading;
float Magn_Vector[3]= {0, 0, 0}; 
float Accel_Vector[3]= {0, 0, 0};
float Gyro_Vector[3]= {0, 0, 0}; 
float Omega_Vector[3]= {0, 0, 0};
float Omega_P[3]= {0, 0, 0}; 
float Omega_I[3]= {0, 0, 0}; 
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

float yaw, pitch, roll;
long timestamp;
long timestamp_old;
float G_Dt; 

int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  ReadSensors();  
  ApplySensorMapping();
  timestamp = millis();
  pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));
  Vector_Cross_Product(temp1, Accel_Vector, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  roll = atan2(temp2[1], temp2[2]);
  Compass_Heading();
  yaw = MAG_Heading;
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

void ApplySensorMapping()
{
   
    Magn_Vector[1] = -magnetom[0];
    Magn_Vector[0] = -magnetom[1];
    Magn_Vector[2] = -magnetom[2];

    Magn_Vector[0] -= MAGN_X_OFFSET;
    Magn_Vector[0] *= MAGN_X_SCALE;
    Magn_Vector[1] -= MAGN_Y_OFFSET;
    Magn_Vector[1] *= MAGN_Y_SCALE;
    Magn_Vector[2] -= MAGN_Z_OFFSET;
    Magn_Vector[2] *= MAGN_Z_SCALE;
  
    Accel_Vector[1] = accel[0];
    Accel_Vector[0] = accel[1];
    Accel_Vector[2] = accel[2];

    Accel_Vector[0] -= ACCEL_X_OFFSET;
    Accel_Vector[0] *= ACCEL_X_SCALE;
    Accel_Vector[1] -= ACCEL_Y_OFFSET;
    Accel_Vector[1] *= ACCEL_Y_SCALE;
    Accel_Vector[2] -= ACCEL_Z_OFFSET;
    Accel_Vector[2] *= ACCEL_Z_SCALE;
    
    Gyro_Vector[1] = -gyro[0];
    Gyro_Vector[0] = -gyro[1];
    Gyro_Vector[2] = -gyro[2];

    Gyro_Vector[0] -= GYRO_X_OFFSET;
    Gyro_Vector[0] *= GYRO_X_SCALE;
    Gyro_Vector[1] -= GYRO_Y_OFFSET;
    Gyro_Vector[1] *= GYRO_Y_SCALE;
    Gyro_Vector[2] -= GYRO_Z_OFFSET;
    Gyro_Vector[2] *= GYRO_Z_SCALE;
}

HardwareSPI spi(1);
void setup()
{
  //SerialUSB.begin(OUTPUT_BAUD_RATE);
  SPI_Init();
  reset_sensor_fusion();
}


void loop()
{
  if ((millis() - timestamp) >= OUTPUT_DATA_INTERVAL) 
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f;
    else
      G_Dt = 0;

    ReadSensors();
    ApplySensorMapping();
    Compass_Heading(); 
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    if(SerialUSB.isConnected() && (SerialUSB.getDTR() || SerialUSB.getRTS()) && (prod ==16488 || prod==0))
    {    
      SerialUSB.print(prod); SerialUSB.print(",");
      SerialUSB.print(TO_DEG(yaw)); SerialUSB.print(",");
      SerialUSB.print(TO_DEG(pitch)); SerialUSB.print(",");
      SerialUSB.print(TO_DEG(roll)); SerialUSB.print(",");

      SerialUSB.print(accel[0]); SerialUSB.print(",");
      SerialUSB.print(accel[1]); SerialUSB.print(",");
      SerialUSB.print(accel[2]); SerialUSB.print(",");

      SerialUSB.print(magnetom[0]); SerialUSB.print(",");
      SerialUSB.print(magnetom[1]); SerialUSB.print(",");
      SerialUSB.print(magnetom[2]); SerialUSB.print(",");
  
      SerialUSB.print(gyro[0]); SerialUSB.print(",");
      SerialUSB.print(gyro[1]); SerialUSB.print(",");
      SerialUSB.print(gyro[2]); SerialUSB.print(",");

      SerialUSB.print(temperature); SerialUSB.print(",");
      SerialUSB.print(pressure); SerialUSB.println();
    }
  }
}
