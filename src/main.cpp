// ============================================ //
// ================= Includes ================= //
// ============================================ //

/**
 * Includes for default arduino
 * WiFi 
 * Non_volatile storage
 */
#include <Arduino.h>
#include <WiFi.h>
#include <nvs.h>
#include <nvs_flash.h>

/**
 * Includes for JSON object handling
 * Requires ArduinoJson library
 * https://arduinojson.org
 * https://github.com/bblanchon/ArduinoJson
 */
#include <ArduinoJson.h>

/** Includes for Bluetooth Serial */
#include "BluetoothSerial.h"

/** Includes persist after restart */
#include <Preferences.h>

/** 
 * Disable brownout detector
 * ESP32 CAM include
*/
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

/** Include http requests */
#include "HTTPClient.h"

// ============================================ //
// ================= Variables ================ //
// ============================================ //

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;

/** Unique device name from MAC */
char apName[] = "ESP32-xxxxxxxxxxxx";

/** WifiClient class */
WiFiClient client;
/** HttpClient class */
HTTPClient http;
/** SerialBT class */
BluetoothSerial SerialBT;

/** Buffer for JSON string */
StaticJsonDocument<200> jsonBuffer;
/** Create camera_config_t objet */
camera_config_t config;

/** SSIDs of local WiFi networks */
String ssidPrim;
String ssidSec;
/** Password for local WiFi network */
String pwPrim;
String pwSec;

/** 
 * Selected network 
  		true = use primary network
  			false = use secondary network
*/
bool usePrimAP = true;
/** Flag if stored AP credentials are available */
bool hasCredentials = false;
/** Connection status */
volatile bool isConnected = false;
/** Connection change status */
bool connStatusChanged = false;

/** Server domain */
String serverName = "pinware.tech";
/** Server path to img upload */
String serverPath = "/host_files-main/upload.php";
/** Server port */
const int serverPort = 80;
/** Device MAC name string */

/** Global variables */
String final_uid = "";
String final_token = "";

/** CAMERA_MODEL_AI_THINKER define pins */
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ============================================ //
// ============== Routine methods ============= //
// ============================================ //

/** Create unique device name from MAC address */
void createName()
{
	uint8_t baseMac[6];
	/** Get MAC address for WiFi station */
	esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
	/** Write unique name into apName */
	sprintf(apName, "ESP32-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

/**
 * initBTSerial
 * Initialize Bluetooth Serial
 * Start BLE server and service advertising
 * @return <code>bool</code>
 * 			true if success
 * 			false if error occured
 */
bool initBTSerial()
{
	if (!SerialBT.begin(apName))
	{
		Serial.println("Failed to start BTSerial");
		return false;
	}
	Serial.println("BTSerial active, device name: " + String(apName));
	return true;
}

/**
 * readBTSerial
 * read all data from BTSerial receive buffer
 * parse data for valid WiFi credentials
 */
void readBTSerial()
{
	uint64_t startTimeOut = millis();
	String receivedData;
	int msgSize = 0;
	/** Read RX buffer to String */
	while (SerialBT.available() != 0)
	{
		receivedData += (char)SerialBT.read();
		msgSize++;
		/** Check for timeout condition */
		if ((millis() - startTimeOut) >= 5000)
			break;
	}
	SerialBT.flush();
	Serial.println("Received message " + receivedData + " over Bluetooth");

	/** Decode the message from BT */
	int keyIndex = 0;
	for (int index = 0; index < receivedData.length(); index++)
	{
		receivedData[index] = (char)receivedData[index] ^ (char)apName[keyIndex];
		keyIndex++;
		if (keyIndex >= strlen(apName))
			keyIndex = 0;
	}

	Serial.println("Received message " + receivedData + " over Bluetooth");

	/** Dump incoming data into Json object */
	auto error = deserializeJson(jsonBuffer, receivedData);
	if (!error)
	{
		if (jsonBuffer.containsKey("ssidPrim") &&
			jsonBuffer.containsKey("pwPrim") &&
			jsonBuffer.containsKey("ssidSec") &&
			jsonBuffer.containsKey("pwSec"))
		{
			ssidPrim = jsonBuffer["ssidPrim"].as<String>();
			pwPrim = jsonBuffer["pwPrim"].as<String>();
			ssidSec = jsonBuffer["ssidSec"].as<String>();
			pwSec = jsonBuffer["pwSec"].as<String>();

			/** Create preferences object to save credentials after shutdown */
			Preferences preferences;
			preferences.begin("WiFiCred", false);
			preferences.putString("ssidPrim", ssidPrim);
			preferences.putString("ssidSec", ssidSec);
			preferences.putString("pwPrim", pwPrim);
			preferences.putString("pwSec", pwSec);
			preferences.putBool("valid", true);
			preferences.end();

			Serial.println("Received over bluetooth:");
			Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
			Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
			connStatusChanged = true;
			hasCredentials = true;
		}
		/** If app send erase order, delete credentials */
		else if (jsonBuffer.containsKey("erase"))
		{ /** {"erase":"true"} */
			Serial.println("Received erase command");
			Preferences preferences;
			preferences.begin("WiFiCred", false);
			preferences.clear();
			preferences.end();
			connStatusChanged = true;
			hasCredentials = false;
			ssidPrim = "";
			pwPrim = "";
			ssidSec = "";
			pwSec = "";

			int err;
			err = nvs_flash_init();
			Serial.println("nvs_flash_init: " + err);
			err = nvs_flash_erase();
			Serial.println("nvs_flash_erase: " + err);
		}
		/** If app send read order, consult credentials and send to BT */
		else if (jsonBuffer.containsKey("read"))
		{ /** {"read":"true"} */
			Serial.println("BTSerial read request");
			String wifiCredentials;
			jsonBuffer.clear();

			/** JSON object for output data */
			jsonBuffer.clear();
			jsonBuffer["ssidPrim"] = ssidPrim;
			jsonBuffer["pwPrim"] = pwPrim;
			jsonBuffer["ssidSec"] = ssidSec;
			jsonBuffer["pwSec"] = pwSec;
			/** Convert JSON object to string */
			serializeJson(jsonBuffer, wifiCredentials);

			/** Encrypt data before BT send */
			int keyIndex = 0;
			Serial.println("Stored settings: " + wifiCredentials);
			for (int index = 0; index < wifiCredentials.length(); index++)
			{
				wifiCredentials[index] = (char)wifiCredentials[index] ^ (char)apName[keyIndex];
				keyIndex++;
				if (keyIndex >= strlen(apName))
					keyIndex = 0;
			}
			/** Print data to BT */
			delay(2000);
			Serial.println("Stored encrypted: " + wifiCredentials);
			SerialBT.print(wifiCredentials);
			SerialBT.print("This is your IP\n");
			SerialBT.print(WiFi.localIP());

			SerialBT.flush();
		}
		/** If app send reset order, ESP will be reset */
		else if (jsonBuffer.containsKey("reset"))
		{
			WiFi.disconnect();
			esp_restart();
		}
	}
	else
	{
		Serial.println("Received invalid JSON");
	}
	jsonBuffer.clear();
}

// =========== Callback's =========== //

/** Callback for receiving IP address from AP */
void gotIP(system_event_id_t event)
{
	isConnected = true;
	connStatusChanged = true;
}

/** Callback for connection loss */
void lostCon(system_event_id_t event)
{
	isConnected = false;
	connStatusChanged = true;
}

/** Callback for connection loss */
void gotCon(system_event_id_t event)
{
	Serial.println("Connection established, waiting for IP");
}

/** Callback for Station mode start */
void staStart(system_event_id_t event)
{
	Serial.println("Station mode start");
}

/** Callback for Station mode stop */
void staStop(system_event_id_t event)
{
	Serial.println("Station mode stop");
}

/**
	 scanWiFi
	 Scans for available networks 
	 and decides if a switch between
	 allowed networks makes sense

	 @return <code>bool</code>
	        True if at least one allowed network was found
*/
bool scanWiFi()
{
	/** RSSI for primary network */
	int8_t rssiPrim = -1;
	/** RSSI for secondary network */
	int8_t rssiSec = -1;
	/** Result of this function */
	bool result = false;

	Serial.println("Start scanning for networks");

	WiFi.disconnect(true);
	WiFi.enableSTA(true);
	WiFi.mode(WIFI_STA);

	/** Scan for AP */
	int apNum = WiFi.scanNetworks(false, true, false, 1000);
	if (apNum == 0)
	{
		Serial.println("No networks found");
		return false;
	}

	byte foundAP = 0;
	bool foundPrim = false;

	for (int index = 0; index < apNum; index++)
	{
		String ssid = WiFi.SSID(index);
		Serial.println("Found AP: " + ssid + " RSSI: " + WiFi.RSSI(index));
		if (!strcmp((const char *)&ssid[0], (const char *)&ssidPrim[0]))
		{
			Serial.println("Found primary AP");
			foundAP++;
			foundPrim = true;
			rssiPrim = WiFi.RSSI(index);
		}
		if (!strcmp((const char *)&ssid[0], (const char *)&ssidSec[0]))
		{
			Serial.println("Found secondary AP");
			foundAP++;
			rssiSec = WiFi.RSSI(index);
		}
	}

	switch (foundAP)
	{
	case 0:
		result = false;
		break;
	case 1:
		if (foundPrim)
		{
			usePrimAP = true;
		}
		else
		{
			usePrimAP = false;
		}
		result = true;
		break;
	default:
		Serial.printf("RSSI Prim: %d Sec: %d\n", rssiPrim, rssiSec);
		if (rssiPrim > rssiSec)
		{
			/** RSSI of primary network is better */
			usePrimAP = true;
		}
		else
		{
			/** RSSI of secondary network is better */
			usePrimAP = false;
		}
		result = true;
		break;
	}
	return result;
}

/** Start connection to AP */
void connectWiFi()
{
	/** Setup callback function for successful connection */
	WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
	/** Setup callback function for lost connection */
	WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);
	/** Setup callback function for lost connection */
	WiFi.onEvent(gotCon, SYSTEM_EVENT_STA_CONNECTED);
	/** Setup callback function for connection established */
	WiFi.onEvent(staStart, SYSTEM_EVENT_STA_START);
	/** Setup callback function for connection established */
	WiFi.onEvent(staStop, SYSTEM_EVENT_STA_STOP);

	WiFi.disconnect(true);
	WiFi.enableSTA(true);
	WiFi.mode(WIFI_STA);

	Serial.println();
	Serial.print("Start connection to ");
	if (usePrimAP)
	{
		Serial.println(ssidPrim);
		WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
	}
	else
	{
		Serial.println(ssidSec);
		WiFi.begin(ssidSec.c_str(), pwSec.c_str());
	}
}

/** Set the config to camera pins */
void configInitCamera()
{
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sscb_sda = SIOD_GPIO_NUM;
	config.pin_sscb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20000000;
	/** YUV422,GRAYSCALE,RGB565,JPEG */
	config.pixel_format = PIXFORMAT_JPEG;

	/** Select lower framesize if the camera doesn't support PSRAM 
	 *  FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
	 * 10-63 lower number means higher quality
	*/
	config.frame_size = FRAMESIZE_UXGA;
	config.jpeg_quality = 10;
	config.fb_count = 2;

	/** Initialize the Camera */
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK)
	{
		Serial.printf("Camera init failed with error 0x%x", err);
		return;
	}

	sensor_t *s = esp_camera_sensor_get();
	s->set_brightness(s, 0);				 // -2 to 2
	s->set_contrast(s, 2);					 // -2 to 2
	s->set_saturation(s, 2);				 // -2 to 2
	s->set_special_effect(s, 0);			 // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
	s->set_whitebal(s, 1);					 // 0 = disable , 1 = enable
	s->set_awb_gain(s, 1);					 // 0 = disable , 1 = enable
	s->set_wb_mode(s, 0);					 // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
	s->set_exposure_ctrl(s, 1);				 // 0 = disable , 1 = enable
	s->set_aec2(s, 0);						 // 0 = disable , 1 = enable
	s->set_ae_level(s, 0);					 // -2 to 2
	s->set_aec_value(s, 300);				 // 0 to 1200
	s->set_gain_ctrl(s, 1);					 // 0 = disable , 1 = enable
	s->set_agc_gain(s, 0);					 // 0 to 30
	s->set_gainceiling(s, (gainceiling_t)0); // 0 to 6
	s->set_bpc(s, 0);						 // 0 = disable , 1 = enable
	s->set_wpc(s, 1);						 // 0 = disable , 1 = enable
	s->set_raw_gma(s, 1);					 // 0 = disable , 1 = enable
	s->set_lenc(s, 1);						 // 0 = disable , 1 = enable
	s->set_hmirror(s, 0);					 // 0 = disable , 1 = enable
	s->set_vflip(s, 0);						 // 0 = disable , 1 = enable
	s->set_dcw(s, 1);						 // 0 = disable , 1 = enable
	s->set_colorbar(s, 0);					 // 0 = disable , 1 = enable
}

/** Http request to delete JSON trigger in server side */
void requestDelete()
{
	String url = "http://pinware.tech/host_files-main/json/delete.php";

	String json;
	StaticJsonDocument<200> doc;
	doc["esp"] = String(apName);

	serializeJson(doc, json);

	if (http.begin(client, url)) // Start request
	{
		Serial.print("[HTTP] POST... JSON DELETE\n");
		int httpCode = http.POST(json); // http request
		if (httpCode > 0)
		{
			Serial.printf("[HTTP] POST... code: %d\n", httpCode);
			if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
			{
				String payload = http.getString(); // Get response
				Serial.println(payload);		   // Show response in serial port
			}
		}
		else
		{
			Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
		}
		http.end();
	}
	else
	{
		Serial.printf("[HTTP} Unable to connect JSON DELETE\n");
	}
	delay(1000);
}

/** Send photo to the cloud server */
String sendPhoto()
{
	/** var to catch server response */
	String getAll;
	String getBody;

	/** take photo */
	camera_fb_t *fb = NULL;
	fb = esp_camera_fb_get();
	if (!fb)
	{
		Serial.println("Camera capture failed");
	}

	/** Start http request to host server */
	Serial.println("Connecting to server: " + serverName);

	if (client.connect(serverName.c_str(), serverPort))
	{
		Serial.println("Connection successful!");
		/** Header and tail for http POST request */
		String head = "--FaisanSMARTMirror\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"" + String(apName) + ".jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
		String tail = "\r\n--FaisanSMARTMirror--\r\n";

		/** Get length of data */
		uint32_t imageLen = fb->len;
		uint32_t extraLen = head.length() + tail.length();
		uint32_t totalLen = imageLen + extraLen;

		client.println("POST " + serverPath + " HTTP/1.1");
		client.println("Host: " + serverName);
		client.println("Content-Length: " + String(totalLen));
		client.println("Content-Type: multipart/form-data; boundary=FaisanSMARTMirror");
		client.println();
		client.print(head);

		/** send data (photo) to server */
		uint8_t *fbBuf = fb->buf;
		size_t fbLen = fb->len;
		for (size_t n = 0; n < fbLen; n = n + 1024)
		{
			if (n + 1024 < fbLen)
			{
				client.write(fbBuf, 1024);
				fbBuf += 1024;
			}
			else if (fbLen % 1024 > 0)
			{
				size_t remainder = fbLen % 1024;
				client.write(fbBuf, remainder);
			}
		}
		client.print(tail);

		esp_camera_fb_return(fb);

		/** Timer to get server response body */
		int timoutTimer = 10000;
		long startTimer = millis();
		boolean state = false;

		while ((startTimer + timoutTimer) > millis())
		{
			Serial.print(".");
			delay(100);
			while (client.available())
			{
				char c = client.read();
				if (c == '\n')
				{
					if (getAll.length() == 0)
					{
						state = true;
					}
					getAll = "";
				}
				else if (c != '\r')
				{
					getAll += String(c);
				}
				if (state == true)
				{
					getBody += String(c);
				}
				startTimer = millis();
			}
			if (getBody.length() > 0)
			{
				break;
			}
		}
		Serial.println();
		client.stop();
		Serial.println(getBody);
	}
	else
	{
		getBody = "Connection to " + serverName + " failed.";
		Serial.println(getBody);
	}
	return getBody;
	
}

/** Asking server for photo trigger */
void requestJSON()
{
	String url = "http://pinware.tech/host_files-main/json/" + String(apName) + ".json";

	/** Start request */
	if (http.begin(client, url))
	{
		Serial.print("[HTTP] GET...\n");
		/** http request */
		int httpCode = http.GET();
		if (httpCode > 0)
		{
			Serial.printf("[HTTP] GET... code: %d\n", httpCode);
			if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
			{
				/** Get response */
				String payload = http.getString();
				/** Show response in serial port */
				Serial.println(payload);

				if (httpCode == 200)
				{
					/** Call for send photo if servers response is OK */
					sendPhoto();
					requestDelete();
				}
			}
		}
		else
		{
			Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
		}
		http.end();
	}
	else
	{
		Serial.printf("[HTTP} Unable to connect \n");
	}
	delay(100);
}

// ============================================ //
// ============= Arduino Functions ============ //
// ============================================ //

void setup()
{
	/** Disable brownout detector */
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
	/** Open camera config */
	configInitCamera();
	/** Create unique device name */
	createName();

	/** Initialize Serial port */
	Serial.begin(115200);
	/** Send device info */
	Serial.print("Build: ");
	Serial.println(compileDate);

	/** Check for WiFi credencials*/
	Preferences preferences;
	preferences.begin("WiFiCred", false);
	bool hasPref = preferences.getBool("valid", false);
	if (hasPref)
	{
		ssidPrim = preferences.getString("ssidPrim", "");
		ssidSec = preferences.getString("ssidSec", "");
		pwPrim = preferences.getString("pwPrim", "");
		pwSec = preferences.getString("pwSec", "");

		if (ssidPrim.equals("") || pwPrim.equals("") || ssidSec.equals("") || pwPrim.equals(""))
		{
			Serial.println("Found preferences but credentials are invalid");
		}
		else
		{
			Serial.println("Read from preferences:");
			Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
			Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
			hasCredentials = true;
		}
	}
	else
	{
		Serial.println("Could not find preferences, need send data over BLE");
	}
	preferences.end();

	/** Start BTSerial */
	initBTSerial();

	if (hasCredentials)
	{
		/** Check for available AP's */
		if (!scanWiFi())
		{
			Serial.println("Could not find any AP");
		}
		else
		{
			/** If AP was found, start connection */
			connectWiFi();
		}
	}
}

void loop()
{
	/** Check for connection status */
	if (connStatusChanged)
	{
		if (isConnected)
		{
			Serial.print("Connected to AP: ");
			Serial.print(WiFi.SSID());
			Serial.print(" with IP: ");
			Serial.print(WiFi.localIP());
			Serial.print(" RSSI: ");
			Serial.println(WiFi.RSSI());
		}
		else
		{
			if (hasCredentials)
			{
				Serial.println("Lost WiFi connection");
				/** Received WiFi credentials */
				if (!scanWiFi())
				{
					/** Check for available AP's */
					Serial.println("Could not find any AP");
				}
				else
				{
					/** If AP was found, start connection */
					connectWiFi();
				}
			}
		}
		connStatusChanged = false;
	}

	/** Check if Data over SerialBT has arrived */
	if (SerialBT.available() != 0)
	{
		/** Get and parse received data */
		readBTSerial();
	}

	/** Request to server JSON trigger */
	requestJSON();
	delay(1000);
}