bool IMU_begin() {
  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    return false;
  }

  bool ok1 = imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_2G);
  bool ok2 = imu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS);
  bool ok3 = imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);


  if (!(ok1 && ok2 && ok3)) {
    return false;
  }

  if (!imu.ConfigSrd(19)) {
    return false;
  }

  Serial.println("Calibrando... No muevas el sensor.");
  for (int i = 0; i < samples; i++) {
    imu.Read();

    accX_offset += imu.accel_x_mps2();
    accY_offset += imu.accel_y_mps2();
    accZ_offset += imu.accel_z_mps2();//- 9.80665; // quitar gravedad

    gyroX_offset += imu.gyro_x_radps();
    gyroY_offset += imu.gyro_y_radps();
    gyroZ_offset += imu.gyro_z_radps();

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
  if (!imu.Read()) {
    return false;
  }

  //______________ACELERACION____________
  ax = imu.accel_x_mps2() - accX_offset;
  ay = imu.accel_y_mps2() - accY_offset;
  az = imu.accel_z_mps2() - accZ_offset;

  if (abs(ax) < 0.07) ax = 0;
  if (abs(ay) < 0.2) ay = 0;
  if (abs(az) < 0.07) az = 0;

  //accX = alpha * ax_uf + (1.0 - alpha) * accX;

  //__________VELOCIDAD ANGULAR_____________
  gx = imu.gyro_x_radps() - gyroX_offset;
  gy  = imu.gyro_y_radps() - gyroY_offset;
  gz  = imu.gyro_z_radps() - gyroZ_offset;

  if (abs(gx) < 0.3) gx = 0;
  if (abs(gy) < 0.3) gy = 0;
  if (abs(gz) < 0.05) gz = 0;

  return true;
}
