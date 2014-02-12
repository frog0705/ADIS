/* This file is part of the ADIS16488 code */

// spi code to read the sensors


void SPI_Init() {
  pinMode(NSS, OUTPUT);
  digitalWrite(NSS, HIGH);
  spi.begin(SPI_562_500KHZ, MSBFIRST, SPI_MODE_3);
  delay(500);
}

void ReadSensors() {
  digitalWrite(NSS, LOW);
  int prodH = spi.transfer(0x0E) << 8;
  int prodL = spi.transfer(0x00);
  prod = (prodH + prodL) > 32767 ? (prodH + prodL)-65536 : (prodH + prodL);
  
  

  int tempH = spi.transfer(0x12) << 8;
  int tempL = spi.transfer(0x00);  
  temperature = (tempH + tempL) > 32767 ? ((tempH + tempL)-65536)*0.00565 + 25 : (tempH + tempL)*0.00565 + 25;
  
  

  int gyro_xH = spi.transfer(0x16) << 8;
  int gyro_xL = spi.transfer(0x00);
  gyro[0] = (gyro_xH + gyro_xL) > 32767 ? (gyro_xH + gyro_xL)-65536 : (gyro_xH + gyro_xL);
  
  

  int gyro_yH = spi.transfer(0x1A) << 8;
  int gyro_yL = spi.transfer(0x00);
  gyro[1] = (gyro_yH + gyro_yL) > 32767 ? (gyro_yH + gyro_yL)-65536 : (gyro_yH + gyro_yL);
  
  

  int gyro_zH = spi.transfer(0x1E) << 8;
  int gyro_zL = spi.transfer(0x00);
  gyro[2] = (gyro_zH + gyro_zL) > 32767 ? (gyro_zH + gyro_zL)-65536 : (gyro_zH + gyro_zL);
  
  

  int accel_xH = spi.transfer(0x22) << 8;
  int accel_xL = spi.transfer(0x00);
  accel[0] = (accel_xH + accel_xL) > 32767 ? (accel_xH + accel_xL)-65536 : (accel_xH + accel_xL);
  
  

  int accel_yH = spi.transfer(0x26) << 8;
  int accel_yL = spi.transfer(0x00);
  accel[1] = (accel_yH + accel_yL) > 32767 ? (accel_yH + accel_yL)-65536 : (accel_yH + accel_yL);
  
  

  int accel_zH = spi.transfer(0x28) << 8;
  int accel_zL = spi.transfer(0x00);
  accel[2] = (accel_zH + accel_zL) > 32767 ? (accel_zH + accel_zL)-65536 : (accel_zH + accel_zL);
  
  

  int magn_xH = spi.transfer(0x2A) << 8;
  int magn_xL = spi.transfer(0x00);
  magnetom[0] = (magn_xH + magn_xL) > 32767 ? (magn_xH + magn_xL)-65536 : (magn_xH + magn_xL);
  
  

  int magn_yH = spi.transfer(0x2C) << 8;
  int magn_yL = spi.transfer(0x00);
  magnetom[1] = (magn_yH + magn_yL) > 32767 ? (magn_yH + magn_yL)-65536 : (magn_yH + magn_yL);
  
 

  int magn_zH = spi.transfer(0x30) << 8;
  int magn_zL = spi.transfer(0x00);
  magnetom[2] = (magn_zH + magn_zL) > 32767 ? (magn_zH + magn_zL)-65536 : (magn_zH + magn_zL);
  
  

  int barH= spi.transfer(0x7E) << 8;
  int barL = spi.transfer(0x00);
  pressure = (barH + barL) > 32767 ? ((barH + barL)-65536)*0.00004 : (barH + barL)*0.00004;
  digitalWrite(NSS, HIGH);

}

