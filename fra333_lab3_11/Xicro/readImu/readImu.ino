#include <Arduino_LSM6DS3.h>
#include <Xicro_read_imu_ID_3.h>
Xicro xicro;

float ax, ay, az;
float gx, gy, gz;

float orientation[4] = {0};
float orientation_covariance[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float angular_velocity[3] = {0};
float angular_velocity_covariance[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float linear_acceleration[3] = {0};
float linear_acceleration_covariance[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
int count = 0;
float data_buffer[6][100];

float mexn[6] = {0, 0, 0,0, 0, 0};

float covariance(float *array, int n);
float mean(float *array, int n);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(57600);
  xicro.begin(&Serial);
  IMU.begin();
  // Set timer
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  // TCB0.CCMP  = 12500;   //20 Hz.
  TCB0.CCMP = 25000; // 10 Hz.
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
  // end of set timer
}

void read_imu_Sendros2()
{
  static bool first_time = true;
  if (IMU.readAcceleration(gx, gy, gz) && IMU.readGyroscope(ax, ay, az))
  {
    angular_velocity[0] = ax * (3.141592 / 180.00) ;
    angular_velocity[0] = angular_velocity[0] - mexn[0];
    angular_velocity[1] = ay * (3.141592 / 180.00);
    angular_velocity[1] = angular_velocity[1] - mexn[1];
    angular_velocity[2] = az * (3.141592 / 180.0);
    angular_velocity[2] = angular_velocity[2] - mexn[2];
    linear_acceleration[0] = gx * (9.80665);
    linear_acceleration[0] = linear_acceleration[0] - mexn[3];
    linear_acceleration[1] = gy * (9.80665);
    linear_acceleration[1] = linear_acceleration[1] - mexn[4];
    linear_acceleration[2] = gz * (-9.80665);
    xicro.publish_Imu_arduino((int32_t)micros() / 1000000.00, (uint32_t)micros() / 1000000.00, (String) "from arduino", 
    orientation[0], orientation[1], orientation[2], orientation[3], orientation_covariance, 
    angular_velocity[0], angular_velocity[1], angular_velocity[2], angular_velocity_covariance, 
    linear_acceleration[0], linear_acceleration[1], linear_acceleration[2], linear_acceleration_covariance);
    if (first_time)
    {
      data_buffer[0][count] = angular_velocity[0];
      data_buffer[1][count] = angular_velocity[1];
      data_buffer[2][count] = angular_velocity[2];
      data_buffer[3][count] = linear_acceleration[0];
      data_buffer[4][count] = linear_acceleration[1];
      data_buffer[5][count] = linear_acceleration[2];
      count++;
    }
    if (count == 100)
    {
      first_time = false;
      angular_velocity_covariance[0] = covariance(data_buffer[0], 100);
      angular_velocity_covariance[4] = covariance(data_buffer[1], 100);
      angular_velocity_covariance[8] = covariance(data_buffer[2], 100);
      linear_acceleration_covariance[0] = covariance(data_buffer[3], 100);
      linear_acceleration_covariance[4] = covariance(data_buffer[4], 100);
      linear_acceleration_covariance[8] = covariance(data_buffer[5], 100);
      
      mexn[0] = mean(data_buffer[0], 100);
      mexn[1] = mean(data_buffer[1], 100);
      mexn[2] = mean(data_buffer[2], 100);
      mexn[3] = mean(data_buffer[3], 100);
      mexn[4] = mean(data_buffer[4], 100);

      count = 0;
    }
  }
}
void loop()
{
  // put your main code here, to run repeatedly:
  xicro.Spin_node();
}

ISR(TCB0_INT_vect)
{
  read_imu_Sendros2();
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

// function calculate covariance of array
float covariance(float *array, int n)
{
  float sum = 0.0, mean, standardDeviation = 0.0;
  int i;
  for (i = 0; i < n; ++i)
  {
    sum += array[i];
  }
  mean = sum / n;
  for (i = 0; i < n; ++i)
    standardDeviation += array[i] - mean;
  return standardDeviation / n;
}

float mean(float *array, int n)
{
  float sum = 0.0, mean;
  int i;
  for (i = 0; i < n; ++i)
  {
    sum += array[i];
  }
  mean = sum/n;
  return mean;
}
