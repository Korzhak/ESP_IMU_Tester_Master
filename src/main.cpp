#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

/**
 * @brief IMU structure (only acceleration and gyroscope)
 */
typedef struct ImuMessage
{
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

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

uint32_t lastSendTime = 0;

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
	if (!mpu.begin())
	{
		Serial.println("Failed to find MPU6050 chip");
		while (1)
		{
			delay(10);
		}
	}

	mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
	mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
	Serial.println("");
	delay(100);
}

void loop()
{
	if((millis() - lastSendTime) >= 10) {
		mpu.getEvent(&a, &g, &temp);

		imu.accX = a.acceleration.x;
		imu.accY = a.acceleration.y;
		imu.accZ = a.acceleration.z;
		imu.gyrX = g.gyro.x;
		imu.gyrY = g.gyro.y;
		imu.gyrZ = g.gyro.z;

		// Send message via ESP-NOW
		esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&imu, sizeof(imu));
		lastSendTime = millis();
	}
}
