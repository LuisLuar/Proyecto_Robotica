bool IMU_begin() {
  Wire.begin();
  Wire.setClock(400000);

  //imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  while (!imu.begin()) {
    return false;
    delay(10);
  }

  imu.setAccelerometerRange(MPU6050_RANGE_2_G);
  imu.setGyroRange(MPU6050_RANGE_250_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);


 

  /*if (!imu.ConfigSrd(19)) {
    return false;
  }*/

  Serial.println("Calibrando... No muevas el sensor.");
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);

    accX_offset += a.acceleration.x;
    accY_offset += a.acceleration.y;
    accZ_offset += a.acceleration.z;//- 9.80665; // quitar gravedad

    gyroX_offset += g.gyro.x;  
    gyroY_offset += g.gyro.y;  
    gyroZ_offset += g.gyro.z;  

    delay(10);
  }

  accX_offset /= samples;
  accY_offset /= samples;
  accZ_offset /= samples;
  gyroX_offset /= samples;
  gyroY_offset /= samples;
  gyroZ_offset /= samples;

  Serial.println("Calibracion Exitosa");

  return true;

}

bool IMU_update() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  //______________ACELERACION____________
  ax = a.acceleration.x - accX_offset;
  ay = a.acceleration.y - accY_offset;
  az = a.acceleration.z - accZ_offset;

  if (abs(ax) < 0.07) ax = 0;
  if (abs(ay) < 0.2) ay = 0;
  if (abs(az) < 0.07) az = 0;

  //accX = alpha * ax_uf + (1.0 - alpha) * accX;

  //__________VELOCIDAD ANGULAR_____________
  gx = g.gyro.x - gyroX_offset;
  gy  = g.gyro.y - gyroY_offset;
  gz  = g.gyro.z - gyroZ_offset;

  if (abs(gx) < 0.3) gx = 0;
  if (abs(gy) < 0.3) gy = 0;
  if (abs(gz) < 0.05) gz = 0;

  return true;
}
