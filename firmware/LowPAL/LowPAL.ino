#define DEST_PHONE_NUMBER "+19876543210" // Update phone numbers here.  Clint's cell # 19876543210.  Steve's is +12345678900
#define SMS_TARGET "+19876543210"        // Secondary phone
/*

Sep 20 - Fixed Clint's bug w/ "+=" vs "+"
Sep 19 - added GPS test and send from lines 318-368
Aug 31 - Cell tower info added
Aug 28 - Move debounce code to replace previous read for input_pin_value and add more debug code. Add ratcheting code to handle potential negative time deltas.
Aug 19 - removed initial program - just a pin tester, check IO
Aug 7 - add pull up resistor on pin 27 - Steve

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

#define TINY_GSM_TEST_GPS false //  Use true or false to include or exclude GPS search
#define GSM_PIN ""              //  NOTE: not sure if needed or not

#define SerialAT Serial1 //  Purpose:  defined and initialized with specific RX and TX pins to communicate with the GSM module.
#define DUMP_AT_COMMANDS //  Purpose:  you can see every AT command that the TinyGSM library sends to the GSM module, as well as the responses received.

#include <TinyGsmClient.h> //  Purpose:  header file in the TinyGSM library, for communicating with various GSM modules. The library provides an abstraction layer that simplifies the process of sending AT commands.
#include <SPI.h>           //  Purpose:  a header file for the SPI (Serial Peripheral Interface) library in Arduino for communicating to SD cards. SPI devices etc
#include <Ticker.h>        //  Purpose:   header file for the Ticker library, which is used in Arduino and ESP8266/ESP32 platforms to perform periodic tasks at specified intervals without using delay functions

#include <esp_sleep.h>

#ifdef DUMP_AT_COMMANDS                    //  NOTE: If enabled it requires the streamDebugger library
#include <StreamDebugger.h>                //  Purpose:  to check data streams back and forth
StreamDebugger debugger(SerialAT, Serial); //  Serial ports monitored?
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define INPUT_PIN 14               // Check Lilygo
#define INPUT_PIN_GPIO GPIO_NUM_14 // see line above

//  Purpose: use for SMS send timing?
#define SMS_SEND_TIME_HOUR 22

// Steve - August 8 - added modem definitions
#define UART_BAUD 9600 //  Baud rate dencreased from 115200 to 9600. This is for the SIM part only.
#define PIN_DTR 25     //
#define PIN_TX 27      //  Communication out
#define PIN_RX 26      //  Commnuncation in
#define PWR_PIN 4      //  Power on pin (needs to go high)

// All RTC_DATA_ATTR variables are persisted while the device is in deep sleep
volatile RTC_DATA_ATTR int bootCount = 0;
volatile RTC_DATA_ATTR int64_t total_water_usage_time_s = 0; // Total time in seconds that the water pump has been in use
volatile RTC_DATA_ATTR int64_t last_rising_edge_time_s = 0;  // Seconds since epoch of the last rising edge
volatile RTC_DATA_ATTR int64_t last_send_time_s = 0;         // Seconds since epoch of the last SMS send time

// Aug 29
uint32_t tStamp = 0;

// The current value of the input pin
int input_pin_value = 0;

//  September 19 - add GPS float for lat/lon
float lat;
float lon;

// Function prototypes
void doDeepSleep();
void doFirstTimeInitialization();
void doLogRisingEdge();
void doLogFallingEdge();
void doSendSMS();

// Part 1 is choosing what triggered the wakeup cycle, then drop to specific case
void setup()
{
  // Configure the input pin with a pulldown resistor
  pinMode(INPUT_PIN, INPUT); //  Steve - Aug 7 - pullup two x 10k resistor added, which also drains while switch is closed.  removed aug 8 , INPUT_PULLUP, left input, input
  delay(100);                // Steve - Aug 7 - added 1/10 second delay

  bootCount++;

  Serial.begin(115200); // Serial port baud rate

  delay(1000); // TODO: Shouldn't have too many unnecessary delays. Figure out how many of these are actually needed.

  Serial.println("setup()");
  Serial.println("  Boot number: " + String(bootCount));

  // Configure the ADC
  // adc1_config_width(ADC_WIDTH_BIT_12);
  // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  // adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);

  // Arduino code to read from inputPin X times and debounce by taking the majority reading from X readings with a 50ms delay in between each read.
  int totalReading = 0; // start with no count
  int NUM_READS = 5;    // count X times, should always be an odd number.

  for (int cnt = 0; cnt < NUM_READS; cnt++)
  {
    totalReading += digitalRead(INPUT_PIN); // Accumulate readings from the input pin
    delay(50);                              // Delay for a bit between each read.
  }

  // Democratically vote for the value of the input pin
  if (totalReading * 2 > NUM_READS)
  { 
    // Majority of readings are HIGH
    input_pin_value = HIGH;
  }
  else
  {
    // Majority of readings are LOW
    input_pin_value = LOW;
  }

  Serial.println("  Current input pin value: " + String(input_pin_value));

  if (totalReading > 0 && totalReading < NUM_READS)
  {
    // If we have a mix of readings, then note it in the log so that we know the debounce code did something
    Serial.println("   Mixed readings detected -- debouncing code worked! Total readings: " + String(totalReading) + " out of " + String(NUM_READS));
  }

  // Check the wakeup reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  Serial.println("  Wakeup reason: " + String(wakeup_reason));

  // If it's powering on for the first time, then do all of our initialization
  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED)
  {
    doFirstTimeInitialization();
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
  {
    // If it's waking up from deep sleep due to an edge on the water sensor input, determine which edge it is.
    if (input_pin_value == HIGH)
    {
      // If it's waking up from deep sleep due to a rising edge on the water sensor input pin, then log the current time of the rising edge, and go back into deep sleep.
      doLogRisingEdge();
    }
    else
    {
      // If it's waking up from deep sleep due to a falling edge on the water sensor input pin, then log the current time of the falling edge, calculate the time span between the rising and falling edges, and add that time span to our day's total accumulation. Then go back into deep sleep.
      doLogFallingEdge();
    }
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
  {
    // If it's waking up from deep sleep due to our SMS send timer going off, then send a text message (to a configured number) with the the previous day's cumulative water usage time. If the message is sent successfully, then clear the cumulative amount and go back to sleep for another 24 hrs.
    doSendSMS();
  }

  // No matter how we woke up, always go back to sleep at the end.
  doDeepSleep();
}


void printLocalTime(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t now = tv.tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();
}

// AT+CLTS Get Local Timestamp
/*
int8_t getLocalTimestampUTC() {
  modem.sendAT("+CLTS?");
  if (modem.waitResponse("+CSQ:") != 1) { return 99; }
  int8_t res = modem.streamGetIntBefore(',');
  modem.waitResponse();
  return res;
}
*/

bool parseTimestamp(const String& timestamp, struct tm& timeinfo, int16_t& quarterHourOffset) {
    int year, month, day, hour, minute, second;
    char tzSign;
    int tzOffset;

    Serial.println(">> Parsing timestamp: " + timestamp);

    // Try parsing with different potential formats
    if (sscanf(timestamp.c_str(), "\"%d/%d/%d,%d:%d:%d%c%d\"",
               &year, &month, &day, &hour, &minute, &second, &tzSign, &tzOffset) == 8) {
        // Successfully parsed
        Serial.println(">> Parsed timestamp with timezone information: " + String(year) + "/" + String(month) + "/" + String(day) + " " + String(hour) + ":" + String(minute) + ":" + String(second) + " " + String(tzSign) + String(tzOffset));
    } else if (sscanf(timestamp.c_str(), "\"%d/%d/%d,%d:%d:%d\"",
                      &year, &month, &day, &hour, &minute, &second) == 6) {
        // Timestamp without timezone information
        tzSign = '+';
        tzOffset = 0;

        Serial.println(">> Parsed timestamp WITHOUT timezone information: " + String(year) + "/" + String(month) + "/" + String(day) + " " + String(hour) + ":" + String(minute) + ":" + String(second) + " " + String(tzSign) + String(tzOffset));
    } else {
        return false; // Parsing failed
    }

    // Fill the tm struct
    timeinfo.tm_year = year + 2000 - 1900;
    timeinfo.tm_mon = month - 1;
    timeinfo.tm_mday = day;
    timeinfo.tm_hour = hour;
    timeinfo.tm_min = minute;
    timeinfo.tm_sec = second;

    // Convert timezone offset
    quarterHourOffset = (tzSign == '-' ? 1 : -1) * tzOffset;

    return true;
}

int8_t setLocalTimeFromCCLK() {
  // Send:
    // AT+CCLK?
  // Receive
    // -> +CCLK: "24/10/08,23:39:49-16"
    // -> 
    // -> OK
  modem.sendAT("+CCLK?");
  if (modem.waitResponse("+CCLK:") != 1) { return 0; }

  String timestamp = SerialAT.readString();
  timestamp.trim();

  struct tm timeinfo;
  // Zero-out the struct
  memset(&timeinfo, 0, sizeof(timeinfo));
  
  int16_t timezone_quarterHourOffset; // tm struct does not have a space for the timezone offset, so store it in a separate number for now.
  
  // Note that the timezone offset is in quarter-hour increments, which can handle things like India's 5.5 hour offset.
  if (!parseTimestamp(timestamp, timeinfo, timezone_quarterHourOffset)) {
    Serial.println(">>> Failed to parse timestamp: " + timestamp);
    return 0;
  }

  // TODO: How to use timezone information?
  // TODO: How to populate timeinfo.tm_isdst properly?
  //  NOTE: Most developing countries do not use DST, so we can default to 0 for now.

  // Set the system time
  time_t t_of_day = mktime(&timeinfo);
  struct timeval tv;
  tv.tv_sec = t_of_day;
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);

  int8_t res = modem.waitResponse(); // Clear the OK

  return res;
}

void doFirstTimeInitialization()
{
  Serial.println("doFirstTimeInitialization()");
  printLocalTime();
  
  //delay(1000);
  // TODO: Ensure that we can communicate with the cell modem, and that it can connect with the tower
  // TODO: If it can connect with the tower, then set the internal RTC according to the cell tower's information so that we have correct local time.
  // TODO: Send an SMS with a "powered on" message including our MAC address, GPS, battery voltage, etc.

  Serial.println("Powering on cell modem...");

  // This section starts the cell antenna
  pinMode(PWR_PIN, OUTPUT);    //  Set power pin to output needed to START modem on power pin 4
  digitalWrite(PWR_PIN, HIGH); // Set power pin high (on)
  delay(300);
  digitalWrite(PWR_PIN, LOW); //  Not sure why it gets toggled off?

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX); // Set conditions for serial port to read and write (it can accept AT commands from PC)

  Serial.println("Enabling SMS message indications.");

  modem.sendAT("+CNMI=1,2,0,0,0"); //    Enable new SMS message indications.  Buffer / storage or not?
  if (SerialAT.available())
  { // Listen for incoming SMS messages
    String sms = SerialAT.readString();
    Serial.println("Response: " + sms);
  }

  if (true) {
    Serial.println("Initializing modem via restart..."); // Start modem on next line
    if (!modem.restart())
    { //  Command to start modem, see extended notes tab
      Serial.println("Failed to restart modem, attempting to continue without restarting");
    } else {
      Serial.println("Modem restarted");
    }
  } else {
    Serial.println("Initializing modem via initialize..."); // Start modem on next line
    if (!modem.init())
    { //  Command to start modem, see extended notes tab
      Serial.println("Failed to init modem, attempting to continue...");
    } else {
      Serial.println("Modem initialized");
    }
  }

  // RF antenna should be started by now
  Serial.println("Println: Your boot count start number is: " + String(bootCount));

  // Texts to send on initialization loop:
  // Test: modem.sendSMS(DEST_PHONE_NUMBER, String("SMS from doFirstTimeInitialization block"));  //send SMS
  modem.sendSMS(DEST_PHONE_NUMBER, "SMS: Your boot count start number is: " + String(+bootCount)); // send SMS
  if (modem.waitResponse(10000L) != 1)
  { // wait 10 seconds for tower ping
    Serial.println("Counter send failed");
  } else {
    Serial.println("Counter send successful");
  }

  // This section checks battery level.  See page 58 of SIM manual.  Output from CBC is (battery charging on or off 0,1,2),(perceantgge capacity),(voltage in mV)
  modem.sendAT("+CBC");                              //    Check battery level.  This line sends the AT request to modem
  String battLoop = modem.stream.readStringUntil('\n');
  battLoop.replace("\r", "");
  battLoop.replace("\n", ""); //  This helps strip out confusing character formatting
  modem.waitResponse();
  Serial.println("Batt Volt = " + String(battLoop)); // This prints string to serial port

  modem.sendSMS(DEST_PHONE_NUMBER, String("Battery level: charge status,capacity,voltage: " + battLoop)); // send cell tower strength to text
  if (modem.waitResponse(10000L) != 1)
  { // ping tower for ten seconds
    DBG("Battery level send failed");
  }
  // End battery check section

  // This section checks cell tower info
  String res = modem.getIMEI();
  Serial.print("IMEI:");
  Serial.println(res); // Called "res" for "result"
  Serial.println();

  modem.sendAT("+CPSI?"); //  test cell provider info
  if (modem.waitResponse("+CPSI: ") == 1)
  {
    res = modem.stream.readStringUntil('\n');
    res.replace("\r", "");
    res.replace("\n", ""); //  This helps strip out confusing character formatting
    modem.waitResponse();
    Serial.print("The current network parameters are:");
    Serial.println(res);
  } else {
    Serial.println("No network parameters found");
  }

  Serial.println("Sending network parameters via SMS...");
  // Send tower info
  if (!modem.sendSMS(SMS_TARGET, String(res)))
  {
    DBG("Network parameter send failed");
  }
  // End tower info section

  // Second SMS_TARGET, just as an example of second SMS number to call.  Useful for Africa contact, plus US contact?  Maybe third phone added too?
  //modem.sendSMS(SMS_TARGET, String("Second cell number listed +1(574)329-0975")); // send SMS
  //if (modem.waitResponse(10000L) != 1)
  //{ // wait 15 seconds for tower ping
  //  DBG("Counter send failed");
  //}
  //  End second phone number section
  Serial.println("\n---Getting Signal Quality---\n");

  // This section is a second way to get minimal tower strength
  int csq = modem.getSignalQuality();
  Serial.println("Signal quality: " + String(csq));

  modem.sendSMS(SMS_TARGET, String(" Your signal strength: " + String(csq)));
  if (modem.waitResponse(10000L) != 1)
  { // wait 10 seconds for tower ping
    DBG("Counter send failed");
  }
  //   End tower signal strength section

  // This section gathers timestamp
  Serial.println("\n---Getting Tower Timestamp---\n");
  setLocalTimeFromCCLK(); //  Set local time from CCLK

  // Now get the local time again and print it
  printLocalTime();

//  End timestamp and cell tower strength section

#if TINY_GSM_TEST_GPS
  Serial.println("\n---Starting GPS TEST---\n");
  // Set Modem GPS Power Control Pin to HIGH ,turn on GPS power
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+CGPIO=0,48,1,1");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG("Set GPS Power HIGH Failed");
  }

  Serial.println("\nEnabling GPS...\n");
  modem.enableGPS();

  float lat, lon; //  Check float has more than two decimal places in SMS?
  while (1)
  {
    if (modem.getGPS(&lat, &lon))
    {
      Serial.printf("lat:%f lon:%f\n", lat, lon);
      break;
    }
    else
    {
      Serial.print("getGPS failed. Is your antenna plugged in? Time: ");
      Serial.println(millis());
    }
    delay(2000);
  }
  modem.disableGPS();

  // Set Modem GPS Power Control Pin to LOW ,turn off GPS power
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+CGPIO=0,48,1,0");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG("Set GPS Power LOW Failed");
  }

  //  Send GPS info to SMS target
  modem.sendSMS(SMS_TARGET, String(" Lat " + String(+lat))); //  Send latitude to text, but more than two decimal places?
  modem.sendSMS(SMS_TARGET, String(" Lon " + String(+lon))); //  Send longitude to text, but more than two decimal places?
  if (modem.waitResponse(10000L) != 1)
  {
    DBG("Lat fail");
  }

  Serial.println("\n---End of GPRS TEST---\n");
#endif // TINY_GSM_TEST_GPS


  delay(1000); // delay 1 second before powering antenna down

  //  This section powers down RF (cell) antenna
  SerialAT.println("AT+CPOWD=1"); // see page 83
  delay(1000);
  while (SerialAT.available())
  {
    Serial.write(SerialAT.read());
  }
  // End RF antenna power down section

  Serial.println("\n---End of doFirstTimeInitialization()---");
}

void doLogRisingEdge()
{
  Serial.println("doLogRisingEdge()");

  // Log the current time of the rising edge, and go back into deep sleep.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t now = tv.tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  last_rising_edge_time_s = tv.tv_sec; // line changed here to remove "+"rfkl
  //  Last_rising_edge_time_s = last_rising_edge_time_s + tv.tv_sec;
  //  Was: last_rising_edge_time_s += tv.tv_sec; but was changed by Tom

  Serial.println("  Rising edge detected at " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));
}

void doLogFallingEdge()
{
  Serial.println("doLogFallingEdge()");
  // Log the current time of the falling edge, calculate the time span between the rising and falling edges, and add that time span to our day's total accumulation. Then go back into deep sleep.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t now = tv.tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo); // check time now

  if (last_rising_edge_time_s == 0)
  { //  only zero if falling edge
    Serial.println("  No rising edge time logged -- invalid reading (noise in the line, or switch triggered too quickly?)");
  }
  else
  {
    time_t then = last_rising_edge_time_s; // get start time
    struct tm then_timeinfo;
    localtime_r(&then, &then_timeinfo); // turn into hr, min
    Serial.println("  Rising edge was detected at " + String(then_timeinfo.tm_hour) + ":" + String(then_timeinfo.tm_min) + ":" + String(then_timeinfo.tm_sec));

    int64_t time_diff_s = (tv.tv_sec - last_rising_edge_time_s); //  subtract start from now elapsed

    //  Check for negative time differences (noise in the line, or switch triggered too quickly?)
    if (time_diff_s < 0)
    {
      Serial.println("  Negative time difference detected -- invalid reading (noise in the line, or switch triggered too quickly?)");
      Serial.println("  Time difference: " + String(time_diff_s) + " seconds");
      // Set the time difference to zero to "ratchet" so that we can't actually count downwards.
      time_diff_s = 0;
    }

    total_water_usage_time_s += time_diff_s;

    // Reset our last rising edge time to zero
    last_rising_edge_time_s = 0; //  reset clock to zero

    Serial.println("  Falling edge detected at " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec));
    Serial.println("  Time difference: " + String(time_diff_s) + " seconds");
    Serial.println("  Total water usage time: " + String(total_water_usage_time_s) + " seconds");
  }
}

void doSendSMS()
{
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

  if (success)
  {
    Serial.println("SMS sent successfully");
    // Clear the total water usage time
    total_water_usage_time_s = 0;
    last_send_time_s = tv.tv_sec;
  }
  else
  {
    Serial.println("SMS failed to send");
  }
}

// This function takes care of all housekeeping needed to go to deep sleep and save our battery.
void doDeepSleep()
{
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
  if (hours_until_sms_send < 0)
  {
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
  if (last_rising_edge_time_s > 0)
  {
    triggerOnEdge = 0;
    Serial.println("  Configuring trigger for falling edge");
  }
  else
  {
    Serial.println("  Configuring trigger for rising edge");
  }

  // Configure the deep sleep wakeup
  esp_sleep_enable_ext0_wakeup(INPUT_PIN_GPIO, triggerOnEdge);

  // Aug 21 edit out
  //  Configure the deep sleep timer
  // esp_sleep_enable_timer_wakeup(total_seconds_until_sms_send * 1000000);

  // Log some information for debugging purposes:
  Serial.println("  Total water usage time: " + String(total_water_usage_time_s) + " seconds");

  // Go to sleep
  Serial.println("  Aug 24 Going to sleep now for but not enable_timer_wakeup now " + String(hours_until_sms_send) + ":" + String(minutes_until_sms_send) + ":" + String(seconds_until_sms_send));
  esp_deep_sleep_start();
}

void loop()
{
  // Our code shouldn't ever get here, but if we do, then go immediately into deep sleep.
  Serial.println("loop() -- SHOULD NOT BE HERE");
  doDeepSleep();
}
