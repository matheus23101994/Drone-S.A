#include "MPU6050.h"
#include "Wire.h"
#include "Kalman.h"
#include <Arduino.h>

uint8_t MPU::i2c_Write(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop)
{
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}
uint8_t MPU::i2c_Write(uint8_t registerAddress, uint8_t data, bool _sendStop)
{
  return i2c_Write(registerAddress, &data, 0x01, _sendStop); // Returns 0 on success
}
uint8_t MPU::i2c_Read(uint8_t registerAddress, uint8_t *data, uint8_t nbytes)
{
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
void MPU::configMPU()
{
  Wire.begin(); //Init i2c
  pinMode(2, OUTPUT); 
  /*---------Definindo freq i2c 400k Hz---------*/
  #if ARDUINO >= 157 //Versão mais atual
  Wire.setClock(400000UL); // Freq = 400kHz.
  #else  //Versão mais antiga
  TWBR = ((F_CPU / 400000UL) - 16) / 2;
  #endif
  /*-------------Config inicial MPU-------------*/
  i2c_data[0] = 7;      /* 0x19 - Taxa de amostragem  8kHz/(7 + 1) = 1000Hz */
  i2c_data[1] = 0x00;   /* 0x1A - Desabilitar FSYNC, Configurar o Filtro de ACC 260Hz, Configurar Filtro de Gyro 256Hz, Amostragem de 8Khz */
  i2c_data[2] = 0x00;   /* 0x1B - Configurar o fundo de escala do Gyro ±250deg/s - Faixa */
  i2c_data[3] = 0x00;   /* 0x1C - Configurar o fundo de escala do Acelerômetro para ±2g - Faixa */
  while (i2c_Write(0x19, i2c_data, 4, false)); //Ficará aqui até a função terminar a escrita nos 4 endereços
  while (i2c_Write(0x6B, 0x01, true)); //PLL tenha como referência o gyro de eixo X, Desabilitando Sleep Mode
  while (i2c_Read(0x75, i2c_data, 1));
  if (i2c_data[0] != 0x68) 
  {
    Serial.print("Erro. Placa desconhecida\n");
    while (1) 
    {
      Serial.print("Erro. Conecte a MPU6050 no barramento i2c\n");
    }
  }
  while(millis() <= 10000)
  {
    digitalWrite(2,HIGH);
    delay(100);
    digitalWrite(2,LOW);
    delay(100);
  }
}
void MPU::initKalman()
{
  while (i2c_Read(0x3B, i2c_data, 14)); //Leitura dos dados de Acc XYZ
  /* 2 - Organizar os dados de Acc XYZ */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])
  /* 3 - Calculo de Roll, Pitch, Yaw */
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  /* 4 - Inicialização do Filtro de Kalman XYZ. Para fazer essa inicialização, necessito seguir os passos 1 a 3*/
  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);

  gyroXangle = roll;
  gyroYangle = pitch;
}
void MPU::lerAcelGyro()
{
  /*---------------Ler registradores---------------*/
  while (i2c_Read(0x3B, i2c_data, 14));
  /*-------Tratamendo do valor da aceleração-------*/
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MostSB ] [ LeastSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MostSB ] [ LeastSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MostSB ] [ LeastSB ])
  // Serial.print(accY);
  // Serial.print("\n");
  /*----------Tratamendo do valor do gyro----------*/
  gyroX = (int16_t)((i2c_data[8] << 8) | i2c_data[9]);   // ([ MostSB ] [ LeastSB ])
  gyroY = (int16_t)((i2c_data[10] << 8) | i2c_data[11]); // ([ MostSB ] [ LeastSB ])
  gyroZ = (int16_t)((i2c_data[12] << 8) | i2c_data[13]); // ([ MostSB ] [ LeastSB ])
}
void MPU::settingMPU()
{
  configMPU(); //Init MPU
  initKalman(); //Init Kalman
  tempoAnterior = millis();
}
void MPU::looppingMPU()
{
  lerAcelGyro();
  
  double dt = (double)(millis() - tempoAnterior);
  tempoAnterior = millis();

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw   = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  gyroXangle = gyroX / 131.0; 
  gyroYangle = gyroY / 131.0;
  gyroZangle = gyroZ / 131.0;

  KalAngleX = KalmanX.getAngle(roll, gyroXangle, dt / 1000);
  KalAngleY = KalmanY.getAngle(pitch, gyroYangle, dt / 1000);

  if((millis() - lasTime) >= 5)
  {
    variacaoZ +=gyroZ;
    if(variacaoZ > 360.0 * 100000.0)
    {
      variacaoZ = 360.0 * 1000000.0;
    }
    else if(variacaoZ < -360.0 * 1000000.0)
    {
      variacaoZ = -360.0 * 1000000.0;
    }
    lasTime = millis();
  }  
  KalAngleZ = KalmanZ.getAngle(variacaoZ, gyroZangle, dt / 1000);

  Serial.print(KalAngleX);Serial.print("\t");
  Serial.print(KalAngleY);Serial.print("\t");
  Serial.print((KalAngleZ / 100000));Serial.print("\n");

}
 void MPU::readAng(float *angX, float *angY, float *angZ)
{
    *angX = (float)KalAngleY;
    *angY = (float)KalAngleX;
    *angZ = (float)KalAngleZ;
}



































































// void MPU::initialize()
// {
//     //Activate the MPU-6050
//     Wire.begin();
//     Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
//     Wire.write(0x6B);                                                    //Send the requested starting register
//     Wire.write(0x00);                                                    //Set the requested starting register
//     Wire.endTransmission(true);                                          //End the transmission
//     //Configure the accelerometer (+/-8g)
//     Wire.begin();
//     Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
//     Wire.write(0x1C);                                                    //Send the requested starting register
//     Wire.write(0x10);                                                    //Set the requested starting register
//     Wire.endTransmission(true);                                          //End the transmission
//     //Configure the gyro (500dps full scale)
//     Wire.begin();
//     Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
//     Wire.write(0x1B);                                                    //Send the requested starting register
//     Wire.write(0x08);                                                    //Set the requested starting register
//     Wire.endTransmission(true);     
// }
// void MPU::calibration()
// {
//     initialize();
//     pinMode(2, OUTPUT);                                                  //Set output 2 (LED) as output
//     digitalWrite(2, HIGH);                                               //Set digital output 13 high to indicate startup 
//     Serial.print("Calibrando MPU6050");                                  //Print text to screen
//     for (int32_t cal_int = 0; cal_int < 2000 ; cal_int ++)               //Run this code 2000 times
//     {                 
//         if(cal_int % 125 == 0)Serial.print(".");                         //Print a dot on the LCD every 125 readings
//         update();                                                      //Read the raw acc and gyro data from the MPU-6050
//         gyro_x_cal += gx;                                                //Add the gyro x-axis offset to the gyro_x_cal variable
//         gyro_y_cal += gy;                                                //Add the gyro y-axis offset to the gyro_y_cal variable
//         gyro_z_cal += gz;                                                //Add the gyro z-axis offset to the gyro_z_cal variable
//         delay(3);                                                        //Delay 3us to simulate the 250Hz program loop
//     }
//     gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
//     gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
//     gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
//     digitalWrite(2, LOW);                                                //All done, turn the LED off
// }
// void MPU::update()
// {
//     Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
//     Wire.write(0x3B);                                                    //Send the requested starting register
//     Wire.endTransmission(false);                                         //End the transmission
//     Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
//     ax = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the ax variable
//     ay = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the ay variable
//     az = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the acc_z variable
//     temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
//     gx = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the gx variable
//     gy = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the gy variable
//     gz = Wire.read()<<8|Wire.read(); 
// }
// void MPU::Atualiza()
// {
//     if((millis() - RunTime) >= cycle)                                    // 250Hz
//     {
//         unsigned long cT = millis();                                     // contar tempo de loop  
//         unsigned long dT = cT - pT;
//         pT = cT;        
//         update();
//         acelx = ax * const_gravid / const_calib;
//         acely = ay * const_gravid / const_calib;
        // acelz = az * const_gravid / const_calib;

        // AccXangle = (atan2(ax, sqrt(pow(ay,2) + pow(az,2)))*180) / 3.14;
        // AccYangle = (atan2(ay, sqrt(pow(ax,2) + pow(az,2)))*180) / 3.14;
        // AccZangle = (atan2(az, sqrt(pow(ax,2) + pow(ay,2)))*180) / 3.14;       
        
//         gx -= gyro_x_cal;
//         gy -= gyro_y_cal;
//         gz -= gyro_z_cal;

//         gyro_yaw = gz;

//         if(gyro_yaw < 0)gyro_yaw *=-1;

//         // Converte valor do acelerometro com base nos 3 eixos
//         // Converte valor do giro em graus por seg
//         // multiplicando uma contante relacionada à taxa de amostragem do sensor
//         // nesse caso, a taxa é +-250g -> 0.00875
//         rate_gyr_x = gx*G_GAIN;
//         rate_gyr_y = gy*G_GAIN;
//         rate_gyr_z = gz*G_GAIN;

//         // Calcula a distância percorrida por integração simples
//         // com base no tempo de loop (dT = cT – pT)
//         gyroXangle+=rate_gyr_x*dT;
//         gyroYangle+=rate_gyr_y*dT;
//         gyroZangle+=rate_gyr_z*dT;

//         // Fusão dos dados: giro + accel
//         // Métodos: filtro complementar ou filtro de kalman
//         // Optei pelo Filtro Complementar por ser mais simples de se aplicar do que o Filtro de Kalman.
//         // Eficiencia bastante satisfatoria, segundo teoria
//         // Atribui peso de 0.98 ao valor do giro e 0.02 ao acelerometro
//         // O giroscópio tem leitura muito boa, mas também apresenta oscilação do valor.
//         // Acelerômetro, por outro lado, é muito ruidoso, mas o desvio é zero.

//         CFangleX=AA*(CFangleX+rate_gyr_x*(dT/1000000)) +(1 - AA) * AccXangle;
//         CFangleY=AA*(CFangleY+rate_gyr_y*(dT/1000000)) +(1 - AA) * AccYangle;
//         CFangleZ=AA*(CFangleZ+rate_gyr_z*(dT/1000000)) +(1 - AA) * AccZangle;


        
//         gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

//         RunTime = millis();
//     }
// }
//
