#include <Arduino.h>

#include <SparkFun_u-blox_GNSS_v3.h> // https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SERIAL myGNSS;        // SFE_UBLOX_GNSS_SERIAL uses Serial (UART). For I2C or SPI, see Example1 and Example3
#define mySerial Serial2             // Use Serial1 to connect to the GNSS module. Change this if required. must use HardwareSerial

#include <QMC5883LCompass.h> //https://github.com/mprograms/QMC5883LCompass
QMC5883LCompass compass;

#define HORN 12
#define FUEL 13
#define ACCS 14
#define STRT 15

#define ACCUV_SENS 36 // ADC1_CH0
#define ACCUV_SENS_R1 8298.3
#define ACCUV_SENS_R2 912

#define BATTV_SENS 39 // ADC1_CH3
#define BATTV_SENS_R1 9904.8
#define BATTV_SENS_R2 3250

#define R502_TOUCH 23
#define R502_RX 25
#define R502_TX 4

#define SIM_TX 5
#define SIM_RX 18
#define SIM_RST 19

#include <Adafruit_Fingerprint.h>
#include <SoftwareSerial.h>
EspSoftwareSerial::UART mySerial2;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial2);

static const int SIM_RXPin = SIM_RX, SIM_TXPin = SIM_TX;
static const uint8_t SIM_ResetPin = SIM_RST;
static const uint32_t SIMBaud = 9600;
#define SerialSIM Serial1
#define delayCommand 100

uint8_t array[4] = {HORN, FUEL, ACCS, STRT}; // OUTPUT
uint8_t indekusu = 0;

uint32_t recTime_relay = 0;

uint32_t recTime_analogRead = 0;
uint8_t total_irritation = 0;
uint8_t reading_irritation = 10;
float avg_accuv = 0.00;
float avg_battv = 0.00;
uint32_t accuv = 0;
uint32_t battv = 0;

uint32_t recTime_GPS = 0;

uint32_t recTime_R305 = 0;
int getFingerprintIDez();
uint8_t getFingerprintEnroll();
uint8_t readnumber(void);
uint8_t id;

uint32_t recTime_SIM = 0;
String inData = "";

void setup()
{
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(array[i], OUTPUT);
  }

  mySerial.begin(115200);
  while (myGNSS.begin(mySerial) == false) // Connect to the u-blox module using mySerial (defined above)
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay(1000);
  }
  myGNSS.setUART1Output(COM_TYPE_UBX); // Set the UART1 port to output UBX only (turn off NMEA noise)

  compass.init();
  compass.setCalibrationOffsets(729.00, 335.00, -446.00);
  compass.setCalibrationScales(0.80, 0.77, 2.22);

  pinMode(SIM_ResetPin, OUTPUT);
  digitalWrite(SIM_ResetPin, HIGH);

  SerialSIM.setRxBufferSize(1024);
  SerialSIM.begin(SIMBaud, SERIAL_8N1, SIM_TXPin, SIM_RXPin);

  // finger.begin(57600);
  mySerial2.begin(57600, EspSoftwareSerial::SWSERIAL_8N1, R502_TX, R502_RX, false);
  delay(1000);

  if (finger.verifyPassword())
  {
    Serial.println("Found fingerprint sensor!");
  }
  else
  {
    Serial.println("Did not find fingerprint sensor :(");
    while (1)
    {
    }
  }

  /* Serial.println("Ready to enroll a fingerprint!");
  Serial.println("Please type in the ID # (from 1 to 127) you want to save this finger as...");
  id = readnumber();
  if (id == 0)
  { // ID #0 not allowed, try again!
    return;
  }
  Serial.print("Enrolling ID #");
  Serial.println(id);

  while (!getFingerprintEnroll())
    ; */
}

void loop()
{
  if (millis() - recTime_R305 >= 100)
  {
    recTime_R305 = millis();
    getFingerprintIDez();
  }

  if (millis() - recTime_SIM >= 2000)
  {
    recTime_SIM = millis();
    SerialSIM.println("AT");
  }

  while (SerialSIM.available() > 0)
  {
    char recieved = SerialSIM.read();
    if (recieved != '\n')
    {
      inData += recieved;
    }

    if (recieved == '\n')
    {
      if (inData.length() != 1)
      {
        inData.remove(inData.length() - 1);
        Serial.println(inData);
      }

      inData = "";
    }
  }

  if (millis() - recTime_relay >= 500)
  {
    recTime_relay = millis();
    if (!digitalRead(array[indekusu]))
    {
      digitalWrite(array[indekusu], 1);
    }
    else
    {
      digitalWrite(array[indekusu], 0);
      indekusu++;
    }

    if (indekusu >= 4)
    {
      indekusu = 0;
    }
  }

  if (millis() - recTime_analogRead >= 50)
  {
    recTime_analogRead = millis();
    if (total_irritation > reading_irritation)
    {
      total_irritation = 0;
      avg_accuv = accuv / float(reading_irritation);
      avg_battv = battv / float(reading_irritation);

      /* Serial.print("ACCU: ");
      Serial.print(avg_accuv);
      Serial.print(" mV, BATT: ");
      Serial.print(avg_battv);
      Serial.print(" mV, "); */

      avg_accuv = avg_accuv / (ACCUV_SENS_R2 / (ACCUV_SENS_R1 + ACCUV_SENS_R2));
      avg_battv = avg_battv / (BATTV_SENS_R2 / (BATTV_SENS_R1 + BATTV_SENS_R2));

      Serial.print("ACCU: ");
      Serial.print(avg_accuv);
      Serial.print(" mV, BATT: ");
      Serial.print(avg_battv);
      Serial.println(" mV");

      accuv = 0;
      battv = 0;
    }
    else
    {
      accuv += analogReadMilliVolts(ACCUV_SENS) - 44;
      battv += analogReadMilliVolts(BATTV_SENS) - 24;
      total_irritation++;
    }
  }

  if (myGNSS.getPVT() == true && millis() - recTime_GPS >= 500)
  {
    recTime_GPS = millis();

    int32_t latitude = myGNSS.getLatitude();
    double lat = latitude / 10000000.0;
    Serial.print(F("Lat: "));
    Serial.print(lat, 7);

    int32_t longitude = myGNSS.getLongitude();
    double lon = longitude / 10000000.0;
    Serial.print(F(" Long: "));
    Serial.print(lon, 7);

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.print(F(" Time: "));
    Serial.print(myGNSS.getHour() + 7);
    Serial.print(F(":"));
    Serial.print(myGNSS.getMinute());
    Serial.print(F(":"));
    Serial.print(myGNSS.getSecond());

    compass.read();
    int16_t a = compass.getAzimuth();
    Serial.print(F(" (A):"));
    Serial.print(a < 0 ? a + 360 : a);

    char myArray[3];
    compass.getDirection(myArray, a);
    Serial.print(F(" "));
    Serial.print(myArray[0]);
    Serial.print(myArray[1]);
    Serial.print(myArray[2]);

    Serial.println();
  }
}

int getFingerprintIDez()
{
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)
    return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)
    return -1;

  p = finger.fingerSearch();
  if (p != FINGERPRINT_OK)
    return -1;

  // found a match!
  Serial.println();
  Serial.print("Found ID #");
  Serial.print(finger.fingerID);
  Serial.print(" with confidence of ");
  Serial.println(finger.confidence);
  return finger.fingerID;
}

uint8_t getFingerprintEnroll()
{

  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #");
  Serial.println(id);
  while (p != FINGERPRINT_OK)
  {
    p = finger.getImage();
    switch (p)
    {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    Serial.println("Image too messy");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_FEATUREFAIL:
    Serial.println("Could not find fingerprint features");
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    Serial.println("Could not find fingerprint features");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  Serial.println("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER)
  {
    p = finger.getImage();
  }
  Serial.print("ID ");
  Serial.println(id);
  p = -1;
  Serial.println("Place same finger again");
  while (p != FINGERPRINT_OK)
  {
    p = finger.getImage();
    switch (p)
    {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p)
  {
  case FINGERPRINT_OK:
    Serial.println("Image converted");
    break;
  case FINGERPRINT_IMAGEMESS:
    Serial.println("Image too messy");
    return p;
  case FINGERPRINT_PACKETRECIEVEERR:
    Serial.println("Communication error");
    return p;
  case FINGERPRINT_FEATUREFAIL:
    Serial.println("Could not find fingerprint features");
    return p;
  case FINGERPRINT_INVALIDIMAGE:
    Serial.println("Could not find fingerprint features");
    return p;
  default:
    Serial.println("Unknown error");
    return p;
  }

  // OK converted!
  Serial.print("Creating model for #");
  Serial.println(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK)
  {
    Serial.println("Prints matched!");
  }
  else if (p == FINGERPRINT_PACKETRECIEVEERR)
  {
    Serial.println("Communication error");
    return p;
  }
  else if (p == FINGERPRINT_ENROLLMISMATCH)
  {
    Serial.println("Fingerprints did not match");
    return p;
  }
  else
  {
    Serial.println("Unknown error");
    return p;
  }

  Serial.print("ID ");
  Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK)
  {
    Serial.println("Stored!");
  }
  else if (p == FINGERPRINT_PACKETRECIEVEERR)
  {
    Serial.println("Communication error");
    return p;
  }
  else if (p == FINGERPRINT_BADLOCATION)
  {
    Serial.println("Could not store in that location");
    return p;
  }
  else if (p == FINGERPRINT_FLASHERR)
  {
    Serial.println("Error writing to flash");
    return p;
  }
  else
  {
    Serial.println("Unknown error");
    return p;
  }

  return true;
}

uint8_t readnumber(void)
{
  uint8_t num = 0;

  while (num == 0)
  {
    while (!Serial.available())
      ;
    num = Serial.parseInt();
  }
  return num;
}