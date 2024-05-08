#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

#define BNO08X_RESET -1

/**
 * @brief IMU structure (only acceleration and gyroscope)
 */
typedef struct ImuMessage
{	
	uint32_t deltaT;
	float accX;
	float accY;
	float accZ;
	float gyrX;
	float gyrY;
	float gyrZ;
} imuMessage;

/****************** OBJECTS AND VARIABLES ******************/
imuMessage imu;
esp_now_peer_info_t peerInfo;
// Slave MAC address 58:BF:25:82:6B:F4
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x82, 0x6B, 0xF4};
// Adafruit_MPU6050 mpu;
Adafruit_BNO08x  bno(BNO08X_RESET);
// sensors_event_t a, g, temp;
sh2_SensorValue_t sensorValue;

uint32_t lastSendTime = 0;

union flags_t
{	
	uint8_t raw;
	struct
	{
		uint8_t acc : 1;
		uint8_t gyr : 1;
	};
};

flags_t Flags;

/****************** ADDITIONAL FUNCTIONS ******************/
// callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// {
// 	Serial.print("\r\nLast Packet Send Status:\t");
// 	Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
// }

/****************** MAIN FUNCTIONS ******************/
void setup()
{
	// Init Serial Monitor
	Serial.begin(115200);

	// Set device as a Wi-Fi Station
	WiFi.mode(WIFI_STA);

	// Init ESP-NOW
	if (esp_now_init() != ESP_OK)
	{
		Serial.println("Error initializing ESP-NOW");
		return;
	}

	// Once ESPNow is successfully Init, we will register for Send CB to
	// get the status of Trasnmitted packet
	// esp_now_register_send_cb(OnDataSent);

	// Register peer
	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
	peerInfo.channel = 0;
	peerInfo.encrypt = false;

	// Add peer
	if (esp_now_add_peer(&peerInfo) != ESP_OK)
	{
		Serial.println("Failed to add peer");
		return;
	}

	// Try to initialize!

	// if (!mpu.begin())
	// {
	// 	Serial.println("Failed to find MPU6050 chip");
	// 	while (1)
	// 	{
	// 		delay(10);
	// 	}
	// }
	// mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
	// mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
	// mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

	if (!bno.begin_I2C(0x4B)) {
		Serial.println("Failed to find BNO08x chip");
		while (1)
		{
			delay(10);
		}
	}

	// Serial.println("Setting desired reports");
	// if (!bno.enableReport(SH2_RAW_ACCELEROMETER)) {
	// 	Serial.println("Could not enable accelerometer");
	// }
	// if (!bno.enableReport(SH2_RAW_GYROSCOPE)) {
	// 	Serial.println("Could not enable gyroscope");
	// }	
	if (!bno.enableReport(SH2_ACCELEROMETER)) {
		Serial.println("Could not enable accelerometer");
	}
	if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
		Serial.println("Could not enable gyroscope");
	}

	Serial.println("");
	delay(100);
}

void loop()
{
	// if((millis() - lastSendTime) >= 10) {

		if (!bno.getSensorEvent(&sensorValue)) {
			return;
		}

		switch (sensorValue.sensorId) {
			case SH2_ACCELEROMETER:
				Flags.acc = true;
				imu.accX = -sensorValue.un.accelerometer.y; // 9.81;
				imu.accY = sensorValue.un.accelerometer.x; // 9.81;
				imu.accZ = sensorValue.un.accelerometer.z; // 9.81;
				break;
			case SH2_GYROSCOPE_CALIBRATED:
				Flags.gyr = true;
				imu.gyrX = -sensorValue.un.gyroscope.y; //* (180 / PI);
				imu.gyrY = sensorValue.un.gyroscope.x; //* (180 / PI);
				imu.gyrZ = sensorValue.un.gyroscope.z; //* (180 / PI);
				break;
		}
		// Send message via ESP-NOW
		if (Flags.raw = 0x3) {
			imu.deltaT = millis() - lastSendTime;
			esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&imu, sizeof(imu));
			Flags.raw = 0;
			lastSendTime = millis();
		}
	// }
}
