/*

# Program Platform

Lilygo SIM7000G

# Program Description

A water pump monitoring system that sends daily water usage statistics via SMS.

The system is designed to be powered by a solar panel and a battery, and to be able to operate in a low-power mode for extended periods of time.

The system is designed to be able to send an SMS message with the previous day's water usage statistics to a configured phone number.

The system is designed to be able to detect the rising and falling edges of a water sensor input pin, and to be able to calculate the time span between the rising and falling edges.

The system is designed to be able to accumulate the time spans between the rising and falling edges of the water sensor input pin, and to be able to send an SMS message with the accumulated time spans to a configured phone number.

The system is designed to be able to operate in a low-power mode for extended periods of time, and to be able to wake up from low-power mode to send an SMS message with the accumulated time spans to a configured phone number.

# Program Outline

On startup, check the wakeup reason.

* If it's powering on for the first time, then do all of our initialization
  * Ensure that we can communicate with the cell modem, and that it can connect with the tower
  * If it can connect with the tower, then set the internal system clock according to the cell tower's information.
  * Send an SMS with a "powered on" message including our MAC address, GPS, battery voltage, etc.
  * DoDeepSleep()

* If it's waking up from deep sleep due to our SMS send timer going off, then send a text message (to a configured number) with the the previous day's cumulative water usage time. If the message is sent successfully, then clear the cumulative amount and go back to sleep for another 24 hrs.

* If it's waking up from deep sleep due to a rising edge on the water sensor input pin, then log the current time of the rising edge, and go back into deep sleep.std

* If it's waking up from deep sleep due to a falling edge on the water sensor input pin, then log the current time of the falling edge, calculate the time span between the rising and falling edges, and add that time span to our day's total accumulation. Then go back into deep sleep.

* DoDeepSleep()
  * Calculate the amount of time remaining until our target SMS send time (10pm)
  * Set wake conditions of the device to be either the target SMS send time or a rising edge on the water sensor input pin -- whichever comes first.

*/


#define TINY_GSM_MODEM_SIM7000  //  Purpose:  inform the TinyGSM library which GSM module you are using. This allows the library to tailor its operations, such as AT commands and responses
#define TINY_GSM_RX_BUFFER 1024 //  Purpose:  determines how much data can be stored temporarily while it is being received from the GSM module.

#define SerialAT Serial1  //  Purpose:  defined and initialized with specific RX and TX pins to communicate with the GSM module.
#define DUMP_AT_COMMANDS  //  Purpose:  you can see every AT command that the TinyGSM library sends to the GSM module, as well as the responses received.

#define SMS_TARGET  "+12345678900"   //  Purpose:  this is the cell number SENT too  

#include <TinyGsmClient.h>  //  Purpose:  header file in the TinyGSM library, for communicating with various GSM modules. The library provides an abstraction layer that simplifies the process of sending AT commands.
#include <SPI.h>  //  Purpose:  a header file for the SPI (Serial Peripheral Interface) library in Arduino for communicating to SD cards. SPI devices etc
#include <Ticker.h>  //  Purpose:   header file for the Ticker library, which is used in Arduino and ESP8266/ESP32 platforms to perform periodic tasks at specified intervals without using delay functions

#ifdef DUMP_AT_COMMANDS  //  Need:  if enabled it requires the streamDebugger library
#include <StreamDebugger.h>  //  Purpose:  to check data streams back and forth
StreamDebugger debugger(SerialAT, Serial);  //  Serial ports monitored?
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define INPUT_PIN 34
#define INPUT_PIN_GPIO GPIO_NUM_34

#define DEST_PHONE_NUMBER "+19876543210"
#define SMS_SEND_TIME_HOUR 22

// All RTC_DATA_ATTR variables are persisted while the device is in deep sleep
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int64_t total_water_usage_time_s = 0; // Total time in seconds that the water pump has been in use
RTC_DATA_ATTR int64_t last_rising_edge_time_s = 0; // Seconds since epoch of the last rising edge
RTC_DATA_ATTR int64_t last_send_time_s = 0; // Seconds since epoch of the last SMS send time

// The current value of the input pin
int input_pin_value = 0;

// Function prototypes
void doDeepSleep();
void doFirstTimeInitialization();
void doLogRisingEdge();
void doLogFallingEdge();
int calculateTimeLeft();
void doSendSMS();

void setup() {
  // Configure the input pin with a pulldown resistor
  pinMode(INPUT_PIN, INPUT_PULLDOWN);

  bootCount++;

  Serial.begin(115200);

  Serial.println("setup()");
  Serial.println("  Boot number: " + String(bootCount));

  // Configure the ADC
  // adc1_config_width(ADC_WIDTH_BIT_12);
  // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  // adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);

  input_pin_value = digitalRead(INPUT_PIN);

  Serial.println("  Current input pin value: " + String(input_pin_value));
  
  // Check the wakeup reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  Serial.println("  Wakeup reason: " + String(wakeup_reason));

  // If it's powering on for the first time, then do all of our initialization
  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
      doFirstTimeInitialization();
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
      // If it's waking up from deep sleep due to an edge on the water sensor input, determine which edge it is.
      if (input_pin_value == HIGH) {
          // If it's waking up from deep sleep due to a rising edge on the water sensor input pin, then log the current time of the rising edge, and go back into deep sleep.
          doLogRisingEdge();
      } else {
          // If it's waking up from deep sleep due to a falling edge on the water sensor input pin, then log the current time of the falling edge, calculate the time span between the rising and falling edges, and add that time span to our day's total accumulation. Then go back into deep sleep.
          doLogFallingEdge();
      }
  }
  
  int timeLeftBeforeSMSDue = calculateTimeLeft();

  if (timeLeftBeforeSMSDue < 0) {
      doSendSMS();
  }

  // No matter how we woke up, always go back to sleep at the end.
  doDeepSleep();

  Serial.println("This line will NEVER execute.");
}

void doFirstTimeInitialization() {
    Serial.println("doFirstTimeInitialization()");
    // TODO: Ensure that we can communicate with the cell modem, and that it can connect with the tower
    // TODO: If it can connect with the tower, then set the internal RTC according to the cell tower's information so that we have correct local time.
    // TODO: Send an SMS with a "powered on" message including our MAC address, GPS, battery voltage, etc.
}

void doLogRisingEdge() {
    Serial.println("doLogRisingEdge()");

    // Log the current time of the rising edge, and go back into deep sleep.
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    last_rising_edge_time_s += tv.tv_sec;

    Serial.println("  Rising edge detected at " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));

}

void doLogFallingEdge() {
    Serial.println("doLogFallingEdge()");
    // Log the current time of the falling edge, calculate the time span between the rising and falling edges, and add that time span to our day's total accumulation. Then go back into deep sleep.
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    if (last_rising_edge_time_s == 0) {
      Serial.println("  No rising edge time logged -- invalid reading (noise in the line, or switch triggered too quickly?)");
    } else {
      int64_t time_diff_s = (tv.tv_sec - last_rising_edge_time_s);

      total_water_usage_time_s += time_diff_s;

      // Reset our last rising edge time to zero
      last_rising_edge_time_s = 0;

      Serial.println("  Falling edge detected at " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));
      Serial.println("  Time difference: " + String(time_diff_s) + " seconds");
      Serial.println("  Total water usage time: " + String(total_water_usage_time_s) + " seconds");
    }
}

int calculateTimeLeft() {

}

void doSendSMS() {
    Serial.println("doSendSMS()");
    // Send a text message (to a configured number) with the the previous day's cumulative water usage time. If the message is sent successfully, then clear the cumulative amount and go back to sleep for another 24 hrs.
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // Send the SMS
    Serial.println("  Sending SMS with water usage time: " + String(total_water_usage_time_s) + " seconds");
    Serial.println("  SMS sent at " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));

    // TODO: Confirm that it sent correctly, and if so, clear the total water usage time.
    bool success = true;

    if (success) {
        Serial.println("SMS sent successfully");
        // Clear the total water usage time
        total_water_usage_time_s = 0;
        last_send_time_s = tv.tv_sec;
    } else {
        Serial.println("SMS failed to send");
    }
}

// This function takes care of all housekeeping needed to go to deep sleep and save our battery.
void doDeepSleep() {
  Serial.println("doDeepSleep()");
  // Calculate the amount of time remaining until our target SMS send time (10pm)
  // Set wake conditions of the device to be either the target SMS send time or a rising edge on the water sensor input pin -- whichever comes first.

  // Calculate the time until the target SMS send time. Get current RTC time via gettimeofday()
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t now = tv.tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  int hours_until_sms_send = SMS_SEND_TIME_HOUR - timeinfo.tm_hour;
  if (hours_until_sms_send < 0) {
    hours_until_sms_send += 24;
  }
  int minutes_until_sms_send = 60 - timeinfo.tm_min;
  int seconds_until_sms_send = 60 - timeinfo.tm_sec;
  int total_seconds_until_sms_send = hours_until_sms_send * 3600 + minutes_until_sms_send * 60 + seconds_until_sms_send;

  // NOTE: This will not be correct until we set the time correctly in doFirstTimeInitialization()
  //  For more info, see: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html
  Serial.println("  Current time of day: " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));

  int triggerOnEdge = 1; // Default to triggering on a rising edge.

  // If we're currently tracking a rising edge, then configure to trigger on a falling edge instead.
  if (last_rising_edge_time_s > 0) {
    triggerOnEdge = 0;
    Serial.println("  Configuring trigger for falling edge");
  } else {
    Serial.println("  Configuring trigger for rising edge");
  }
  
  // Configure the deep sleep wakeup
  esp_sleep_enable_ext0_wakeup(INPUT_PIN_GPIO, triggerOnEdge);

  // Configure the deep sleep timer
  esp_sleep_enable_timer_wakeup(total_seconds_until_sms_send * 1000000);

  // Log some information for debugging purposes:
  Serial.println("  Total water usage time: " + String(total_water_usage_time_s) + " seconds");
  
  // Go to sleep
  Serial.println("  Going to sleep now for " + String(hours_until_sms_send) + ":" + String(minutes_until_sms_send) + ":" + String(seconds_until_sms_send));
  esp_deep_sleep_start();
}

void loop() {
  // Our code shouldn't ever get here, but if we do, then go immediately into deep sleep.
  Serial.println("loop() -- SHOULD NOT BE HERE");
  doDeepSleep();
}
