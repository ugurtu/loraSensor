#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <LoRa.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "I2C_AHT10.h"


String node_id = String("ID") + "010514";

//Set sleep time, when value is 1 almost sleep 20s,when value is 450, almost 1 hour.
#define SLEEP_CYCLE 2

//Lora set
//Set Lora frequency
#define FREQUENCY 868.0

#define BANDWIDTH 250.0
#define SPREADING_FACTOR 7
#define CODING_RATE 8
#define OUTPUT_POWER 10
#define PREAMBLE_LEN 8
#define GAIN 0
int counter = 0;

//328p
#define DIO0 2
#define DIO1 6

#define LORA_RST 4
#define LORA_CS 10

#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 13

//pin set
#define VOLTAGE_PIN A3
#define PWM_OUT_PIN 9
#define SENSOR_POWER_PIN 5
#define ADC_PIN A2

#define DEBUG_OUT_ENABLE 1


String saveData[10];
byte localAddress = 187;     // address of this device
byte destination = 0xFF;

SX1276 radio = new Module(LORA_CS, DIO0, LORA_RST, DIO1);
AHT10 humiditySensor;
int dataSize = sizeof(saveData) / sizeof(char);

bool readSensorStatus = false;
int sensorValue = 0; // variable to store the value coming from the sensor
int batValue = 0;    // the voltage of battery
int count = 0;
int ADC_O_1;           // ADC Output First 8 bits
int ADC_O_2;           // ADC Output Next 2 bits
int16_t packetnum = 0; // packet counter, we increment per xmission
float temperature = 0.0;
float humidity = 0.0;

/**
 * Init AHT
 * Check if AHT is working on Sensor.
 * 
 */
bool AHT_init()
{
    bool ret = false;
    Wire.begin();
    if (humiditySensor.begin() == false)
    {
#if DEBUG_OUT_ENABLE
        Serial.println("AHT10 not detected. Please check wiring. Freezing.");
#endif
    }

    if (humiditySensor.available() == true)
    {
        temperature = humiditySensor.getTemperature();
        humidity = humiditySensor.getHumidity();
        ret = true;
    }
    if (isnan(humidity) || isnan(temperature))
    {
#if DEBUG_OUT_ENABLE
        Serial.println(F("Failed to read from AHT sensor!"));
#endif
    }
    return ret;
}

/**
 * Inittialise Lora Sender/Receiver
 * 
 */
void Lora_init()
{
    int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, OUTPUT_POWER, PREAMBLE_LEN, GAIN);
    if (state == SKIP_NONE)
    {
#if DEBUG_OUT_ENABLE
        Serial.println(F("success!"));
#endif
    }
    else
    {
#if DEBUG_OUT_ENABLE
        Serial.print(F("failed, code "));
        Serial.println(state);
#endif
        // while (true)
        //     ;
    }
}
void setup()
{
#if DEBUG_OUT_ENABLE
    Serial.begin(115200);
    Serial.println("Soil start.");
#endif
    delay(100);

    // set up Timer 1
    pinMode(PWM_OUT_PIN, OUTPUT);

    TCCR1A = bit(COM1A0);            // toggle OC1A on Compare Match
    TCCR1B = bit(WGM12) | bit(CS10); // CTC, scale to clock
    OCR1A = 1;

    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, HIGH); //Sensor power on
    delay(100);

    Lora_init();

    Wire.begin();
    if (humiditySensor.begin() == false)
    {

#if DEBUG_OUT_ENABLE
        Serial.println("AHT10 not detected. Please check wiring. Freezing.");
#endif
    }
#if DEBUG_OUT_ENABLE
    else
        Serial.println("AHT10 acknowledged.");
#endif

    LoRa.onReceive(onReceive);
    LoRa.receive();
    do_some_work();
    
//setup over
#if DEBUG_OUT_ENABLE
    Serial.println("[Set]Sleep Mode Set");
#endif
    low_power_set();
}

void loop()
{
    wdt_disable();

    if (count > SLEEP_CYCLE)
    {
#if DEBUG_OUT_ENABLE
        //code start
        Serial.println("Code start>>");
#endif
       
       if(onReceive(255) && counter == 0){ 
        do_some_work(); 
        all_pins_low();
       }
       else{
        String message = "INEDX:" + (String)packetnum + " H:" + (String)humidity + " T:" + (String)temperature + " ADC:" + (String)sensorValue + " BAT:" + (String)batValue;
        String back_str = node_id + " REPLY : SOIL " + message;
        saveData[counter] = back_str;
        counter++;
        Serial.println("Data Added");
       }
       
       if(onReceive(255) && counter > 0){
        Serial.println(dataSize);
        for(int i = 0; i < counter; i++){ 
          radio.transmit(saveData[i]);
          all_pins_low();
        }
       }
        counter = 0;
        memset(saveData, 0, sizeof(saveData));
        

#if DEBUG_OUT_ENABLE
        //code end
        Serial.println("Code end<<");

#endif
        //count init
        count = 0;
    }

    low_power_set();
}

ISR(WDT_vect)
{
#if DEBUG_OUT_ENABLE
    Serial.print("[Watch dog]");
    Serial.println(count);
#endif
    delay(100);
    count++;
    //wdt_reset();
    wdt_disable(); // disable watchdog
}

//Set low power mode and into sleep
void low_power_set()
{
    all_pins_low();
    delay(10);
    // disable ADC
    ADCSRA = 0;

    sleep_enable();
    watchdog_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    delay(10);
    noInterrupts();
    sleep_enable();

    // turn off brown-out enable in software
    MCUCR = bit(BODS) | bit(BODSE);
    MCUCR = bit(BODS);
    interrupts();

    sleep_cpu();
    sleep_disable();
}

//Enable watch dog
void watchdog_init()
{
    // clear various "reset" flags
    MCUSR = 0;
    // allow changes, disable reset
    WDTCSR = bit(WDCE) | bit(WDE);
    WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0); // set WDIE, and 8 seconds delay
    wdt_reset();                                // pat the dog
}

void do_some_work()
{

    digitalWrite(SENSOR_POWER_PIN, HIGH); // Sensor/RF95 power on
    digitalWrite(LORA_RST, HIGH);
    delay(5);
    pinMode(PWM_OUT_PIN, OUTPUT);    //digitalWrite(PWM_OUT_PIN, LOW);
    TCCR1A = bit(COM1A0);            // toggle OC1A on Compare Match
    TCCR1B = bit(WGM12) | bit(CS10); // CTC, scale to clock
    OCR1A = 1;                       // compare A register value (5000 * clock speed / 1024).When OCR1A == 1, PWM is 2MHz

    Lora_init();
    delay(50);

    //ADC2  AVCC as reference voltage
    ADMUX = _BV(REFS0) | _BV(MUX1);

    //ADC2  internal 1.1V as ADC reference voltage
    //ADMUX = _BV(REFS1) |_BV(REFS0) | _BV(MUX1);

    // 8 
    ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
    delay(50);
    for (int i = 0; i < 3; i++)
    {
        //start ADC conversion
        ADCSRA |= (1 << ADSC);

        delay(10);

        if ((ADCSRA & 0x40) == 0)
        {
            ADC_O_1 = ADCL;
            ADC_O_2 = ADCH;

            sensorValue = (ADC_O_2 << 8) + ADC_O_1;
            ADCSRA |= 0x40;
#if DEBUG_OUT_ENABLE
            Serial.print("ADC:");
            Serial.println(sensorValue);
#endif

            if (readSensorStatus == false)
                readSensorStatus = AHT_init();
        }
        ADCSRA |= (1 << ADIF); //reset as required
        delay(50);
    }

    //ADC3  internal 1.1V as ADC reference voltage
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX1) | _BV(MUX0);

    delay(50);
    for (int i = 0; i < 3; i++)
    {
        //start ADC conversion
        ADCSRA |= (1 << ADSC);

        delay(10);
        
        if ((ADCSRA & 0x40) == 0)
        {
            ADC_O_1 = ADCL;
            ADC_O_2 = ADCH;

            batValue = (ADC_O_2 << 8) + ADC_O_1;
            ADCSRA |= 0x40;
#if DEBUG_OUT_ENABLE
            Serial.print("BAT:");
            Serial.println(batValue);
            float bat = (float)batValue * 3.3;
            bat = bat / 1024.0;
            Serial.print(bat);
            Serial.print("V");
#endif
        }
        ADCSRA |= (1 << ADIF); //reset as required
        delay(50);
    }
    send_lora();
    delay(1000);
    radio.sleep();

    packetnum++;
    readSensorStatus = false;
    digitalWrite(SENSOR_POWER_PIN, LOW); // Sensor/RF95 power off
    delay(100);
}

void all_pins_low()
{
    pinMode(PWM_OUT_PIN, INPUT);
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);

    delay(50);
}

boolean onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  Serial.println(incomingLength);
  String incoming = "";                 // payload of packet

  incoming += (char)LoRa.read();
  
  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();
    // add bytes one by one
  }
  

  if (incomingLength != 255) {   // check length for error
    Serial.println("error: message length does not match length");
    return false;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  
  if (recipient != 187 && recipient != -1) {
    Serial.print("Receipent: ");
    Serial.println(recipient);
    Serial.print("Localadress: ");
    Serial.println(localAddress); 
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

return true;
}



void send_lora()
{
    String message = "INEDX:" + (String)packetnum + " H:" + (String)humidity + " T:" + (String)temperature + " ADC:" + (String)sensorValue + " BAT:" + (String)batValue;
    String back_str = node_id + " REPLY : SOIL " + message;

#if DEBUG_OUT_ENABLE
    Serial.println(back_str);
#endif

    radio.transmit(back_str);
}
