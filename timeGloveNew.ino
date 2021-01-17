#include <Wire.h>
#include "Kalman.h" // Kalman Filter Library

const int mpu_addr = 0x68; // I2C address of the MPU-6050
uint8_t light = 200;			 //задержка, свет включен, микросекунд
uint8_t dark;							 //свет выключен, микросекунды
uint8_t min_dark = 1;			 //миминмальная задержка темноты
uint8_t max_dark = 50;		 //максимальная задержка темноты
byte light_pin = 2;				 //сюда подключен свет
byte potent_pin = 6;			 //аналоговый пин потенциометра
int16_t angle;

int16_t accZ, accX, accY, gyroX, gyroZ;
double accZangle, accXangle;
double kalAngleX, kalAngleZ;

uint32_t timer;

Kalman kalmanX;
Kalman kalmanZ;

void setup()
{
	Wire.begin();
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
	Wire.beginTransmission(mpu_addr);
	Wire.write(0x6B);
	Wire.write(0); // wake up the mpu6050
	Wire.endTransmission(true);
	Serial.begin(9600);
	pinMode(light_pin, OUTPUT);
	pinMode(potent_pin, INPUT);
	kalmanX.setAngle(180); // Set starting angle
	kalmanZ.setAngle(180);
	timer = micros();
}
void loop()
{
	measure();

	angle = kalAngleZ;																							 //вычисляем угол, сделав поправку на 250 градусов
	dark = map(analogRead(potent_pin), 0, 1024, min_dark, max_dark); //расчет времени темноты как сумму угла с датчика угла и значения с потенциометра
	digitalWrite(light_pin, 1);																			 // Включаем свет
	delayMicroseconds(light);																				 // ждем
	digitalWrite(light_pin, 0);																			 // выключаем
	delay(dark);
	delayMicroseconds(2000 + angle * 10); // ждем
}

void measure()
{
	accX = Acc_X();
	accY = Acc_Y();
	accZ = Acc_Z();
	gyroX = Gyro_X();
	gyroZ = Gyro_Z();
	/* Calculate the angls based on the different sensors and algorithm */
	accZangle = (atan2(accX, accY) + PI) * RAD_TO_DEG;
	accXangle = (atan2(accY, accX) + PI) * RAD_TO_DEG;
	double gyroXrate = (double)gyroX / 131.0;
	double gyroZrate = -((double)gyroZ / 131.0);
	kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
	kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (double)(micros() - timer) / 1000000);
	timer = micros();
}

int16_t Gyro_X()
{
	Wire.beginTransmission(mpu_addr);
	Wire.write(0x43);
	Wire.endTransmission(false);
	Wire.requestFrom(mpu_addr, 2);
	int16_t GyX = Wire.read() << 8 | Wire.read();
	Wire.endTransmission(true);
	return GyX;
}

int16_t Gyro_Z()
{
	Wire.beginTransmission(mpu_addr);
	Wire.write(0x47);
	Wire.endTransmission(false);
	Wire.requestFrom(mpu_addr, 2);
	int16_t GyZ = Wire.read() << 8 | Wire.read();
	Wire.endTransmission(true);
	return GyZ;
}

int16_t Acc_Y()
{
	Wire.beginTransmission(mpu_addr);
	Wire.write(0x3D);
	Wire.endTransmission(false);
	Wire.requestFrom(mpu_addr, 2);
	double AccY = Wire.read() << 8 | Wire.read();
	Wire.endTransmission(true);
	return AccY;
}

int16_t Acc_X()
{
	Wire.beginTransmission(mpu_addr);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(mpu_addr, 2);
	int16_t AccX = Wire.read() << 8 | Wire.read();
	Wire.endTransmission(true);
	return AccX;
}

int16_t Acc_Z()
{
	Wire.beginTransmission(mpu_addr);
	Wire.write(0x3F);
	Wire.endTransmission(false);
	Wire.requestFrom(mpu_addr, 2);
	int16_t AccZ = Wire.read() << 8 | Wire.read();
	Wire.endTransmission(true);
	return AccZ;
}
