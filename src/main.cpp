#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define DHT_PIN 26
#define noise_pin 32
#define gas_sensor_pin 35
#define DHTTYPE DHT11

// WiFi AP SSID
#define WIFI_SSID "cloudflight-guest"
// WiFi password
#define WIFI_PASSWORD "digitalfuture"

#define INFLUXDB_URL "https://eu-central-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "token-key"
#define INFLUXDB_ORG "008207ced3014295"
#define INFLUXDB_BUCKET "Polihackv16"

// Time zone info
#define TZ_INFO "UTC3"

// WiFi for ESP32
#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensor("Sensor");

DHT dht(DHT_PIN, DHTTYPE);
Adafruit_MPU6050 vibro_sensor;
float sound_level = 0.0;
float umiditate = 0.0;
float temperatura = 0.0;
bool gasdetected = false;
bool gasflag = false;

float vibration = 0.0;
float accelX, accelY, accelZ;

float simulated_co2 = 0.0;
float co2Min = 400;
float co2Max = 600;
float co2ChangeLimit = 10;

float pm1 = 0.0;
float pmMin = 5.0;
float pmMax = 30;
float pmChangeLimit = 2.0;

volatile bool has_interrupted = false;

void IRAM_ATTR gas_sensor_interupt()
{
  has_interrupted = true;
}

void simulate_sensor(float *sensor, float min, float max, float limit)
{
  float change = random(-limit * 10, limit * 10) / 10.0;
  *sensor += change;
  if (*sensor > max)
    *sensor = max;
  else if (*sensor < min)
    *sensor = min;
}

void noise_sensor()
{
  sound_level = analogRead(noise_pin);
  Serial.print(sound_level);
  Serial.println(" ");
  delay(250);
}

void DHT_sensor()
{
  delay(1000);
  umiditate = dht.readHumidity();
  temperatura = dht.readTemperature();
  Serial.print(temperatura);
  Serial.print(" ");
  Serial.print(umiditate);
  Serial.println("");
  if (isnan(umiditate) || isnan(temperatura))
  {
    Serial.println("err");
    return;
  }
}

void vibration_sensor()
{
  if (vibro_sensor.getMotionInterruptStatus())
  {
    /* Get new sensor events with the readings */
    sensors_event_t acc, giro, temp;
    vibro_sensor.getEvent(&acc, &giro, &temp);

    // Extract acceleration data
    accelX = acc.acceleration.x;
    accelY = acc.acceleration.y;
    accelZ = acc.acceleration.z;

    /* Print out the values */
    Serial.print("AccelX:");
    Serial.print(acc.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(acc.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(acc.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(giro.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(giro.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(giro.gyro.z);
    Serial.println("");
  }

  delay(10);
}

void setup()
{
  Serial.begin(9600);

  pinMode(DHT_PIN, INPUT);
  pinMode(gas_sensor_pin, INPUT);
  delay(3000);

  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Accurate time is necessary for certificate validation and writing in batches
  // We use the NTP servers in your area as provided by: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (client.validateConnection())
  {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  }
  else
  {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  dht.begin();

  if (!vibro_sensor.begin())
  {
    Serial.println("error not found");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  vibro_sensor.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  vibro_sensor.setMotionDetectionThreshold(1);
  vibro_sensor.setMotionDetectionDuration(20);
  vibro_sensor.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  vibro_sensor.setInterruptPinPolarity(true);
  vibro_sensor.setMotionInterrupt(true);

  attachInterrupt(digitalPinToInterrupt(gas_sensor_pin), gas_sensor_interupt, RISING);
}

void loop()
{
  sensor.clearFields();

  DHT_sensor();
  noise_sensor();
  vibration_sensor();
  simulate_sensor(&simulated_co2, co2Min, co2Max, co2ChangeLimit);
  simulate_sensor(&pm1, pmMin, pmMax, pmChangeLimit);

  if (has_interrupted == true)
  {

    Serial.println("Gas Detected");

    gasdetected = true;
  }

  vibration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  sensor.addField("temperature", temperatura);
  sensor.addField("humidity", umiditate);
  sensor.addField("noise", sound_level / 6);
  sensor.addField("vibration", vibration * 10);
  sensor.addField("gas_detection", gasdetected);
  sensor.addField("CO2", simulated_co2);
  sensor.addField("pm", pm1);

  if (!client.writePoint(sensor))
  {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  delay(1000);
}