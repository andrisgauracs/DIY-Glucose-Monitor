#include <Wire.h>
#include <Adafruit_GFX.h>

#define SSD1306_NO_SPLASH
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansOblique12pt7b.h>

#include <ADS1X15.h>
#include <avr/pgmspace.h>
#include "glucose_model.h"

#define SCREEN_I2C_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 48
#define OLED_RST_PIN -1

#define FRAME_DELAY (15)
#define FRAME_WIDTH (32)
#define FRAME_HEIGHT (32)

byte buffer[128];

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST_PIN);
ADS1115 ADS(0x48);

#define BASELINE_SAMPLE_COUNT 10
#define SAMPLE_COUNT 50
#define SAMPLE_INTERVAL 0.1f
#define VOLTS_SCALER 10

static float recovery_readings[SAMPLE_COUNT];

float baseline_buffer[BASELINE_SAMPLE_COUNT];
int baseline_index = 0;
bool baseline_ready = false;
int baseline_filled = 0;

const int buttonPin = 2;
bool lastState = HIGH;
bool logging = false;
bool lastButtonState = LOW;
bool stableButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
const unsigned long toggleDelay = 500;

float baseline = 0;
int baseline_mV = 0;
int voltageDiff = 0;

int frameY = 16;
int frame = 0;

bool drop_detected = false;
bool collecting_recovery = false;
bool done = false;
bool samplesCollected = false;
int recovery_index = 0;

float min_voltage = 10000.0;
float time_counter = 0.0f;

float glucose_result = 0;

int countdown = 5;
unsigned long lastLogTime = 0;
const unsigned long logInterval = 100;
unsigned long startTime = 0;

#define MAX_INPUT_LENGTH 32
char inputBuffer[MAX_INPUT_LENGTH];
byte inputIndex = 0;

String message = "Waiting for blood sample... ";
int scrollX = SCREEN_WIDTH;
int messageLength = -message.length() * 11;

void drawIntroDisplay()
{
    display.clearDisplay();
    display.setCursor(scrollX, 42);
    display.print("Waiting for blood sample...");
    scrollX = scrollX - 3;
    if (scrollX < messageLength)
    {
        scrollX = SCREEN_WIDTH;
    }
    display.display();
}

void drawCountdown()
{
    display.clearDisplay();
    display.setCursor((SCREEN_WIDTH / 2), 42);
    display.print(countdown);
    display.display();
}

void drawResult(float mmolL)
{
    display.clearDisplay();
    display.setCursor(2, (SCREEN_HEIGHT / 2) + 18);
    display.print(mmolL, 1);
    display.print(" mmol/l");
    display.display();
}

double simpson_area_under(float *readings, int count, float baseline, float dx)
{
    if (count < 3 || (count - 1) % 2 != 0)
    {
        return -1.0;
    }

    double area = (baseline * VOLTS_SCALER) - (readings[0] * VOLTS_SCALER);
    for (int i = 1; i < count - 1; i++)
    {
        double y = (baseline * VOLTS_SCALER) - (readings[i] * VOLTS_SCALER);
        if (i % 2 == 0)
        {
            area += 2 * y;
        }
        else
        {
            area += 4 * y;
        }
    }
    area += (baseline * VOLTS_SCALER) - (readings[count - 1] * VOLTS_SCALER);

    area *= dx / 3.0;
    return area;
}

void measureGlucose(float voltage)
{
    time_counter += SAMPLE_INTERVAL;
    int voltage_mV = (int)(voltage * 10000.0);

    if (!drop_detected)
    {
        baseline_buffer[baseline_index] = voltage;
        baseline_index = (baseline_index + 1) % BASELINE_SAMPLE_COUNT;
        if (baseline_filled < BASELINE_SAMPLE_COUNT)
            baseline_filled++;
        if (baseline_filled >= BASELINE_SAMPLE_COUNT)
            baseline_ready = true;
    }

    if (baseline_ready && !drop_detected)
    {
        float sum = 0.0;
        for (int i = 0; i < BASELINE_SAMPLE_COUNT; i++)
        {
            sum += baseline_buffer[i];
        }
        baseline = sum / BASELINE_SAMPLE_COUNT;
        baseline_mV = (int)(baseline * 10000.0);
    }

    if (!drop_detected && baseline_ready)
    {
        voltageDiff = baseline_mV - voltage_mV;
        if (voltageDiff >= 500)
        {
            drop_detected = true;
            min_voltage = voltage;
            collecting_recovery = true;
        }
    }
    else if (drop_detected)
    {
        if (voltage < min_voltage)
        {
            min_voltage = voltage;
        }
    }
    if (collecting_recovery)
    {
        if (recovery_index < SAMPLE_COUNT)
        {
            recovery_readings[recovery_index++] = voltage;
        }

        if (recovery_index >= 9)
        {
            countdown = 4;
        }
        if (recovery_index >= 19)
        {
            countdown = 3;
        }
        if (recovery_index >= 29)
        {
            countdown = 2;
        }
        if (recovery_index >= 39)
        {
            countdown = 1;
        }

        if (recovery_index >= SAMPLE_COUNT)
        {
            samplesCollected = 1;

            float min_voltage = recovery_readings[0];
            for (int i = 1; i < SAMPLE_COUNT; i++)
            {
                if (recovery_readings[i] < min_voltage)
                {
                    min_voltage = recovery_readings[i];
                }
            }

            float drop_height = baseline - min_voltage;

            float slope = (recovery_readings[SAMPLE_COUNT - 1] - recovery_readings[0]) / (SAMPLE_COUNT * SAMPLE_INTERVAL);

            int adjusted_count = (SAMPLE_COUNT % 2 == 0) ? SAMPLE_COUNT - 1 : SAMPLE_COUNT;
            double area_under = simpson_area_under(recovery_readings, adjusted_count, baseline, SAMPLE_INTERVAL);

            // Time to recovery (90% of drop height)
            float recovery_threshold_voltage = min_voltage + (baseline - min_voltage) * 0.9f;
            float time_to_recovery = (SAMPLE_COUNT - 1) * SAMPLE_INTERVAL;
            for (int i = 0; i < SAMPLE_COUNT; i++)
            {
                if (recovery_readings[i] >= recovery_threshold_voltage)
                {
                    time_to_recovery = i * SAMPLE_INTERVAL;
                    break;
                }
            }

            // Peak-to-peak: max - min in recovery window
            float max_voltage = recovery_readings[0];
            for (int i = 1; i < SAMPLE_COUNT; i++)
            {
                if (recovery_readings[i] > max_voltage)
                {
                    max_voltage = recovery_readings[i];
                }
            }
            float peak_to_peak = max_voltage - min_voltage;

            float glucose = predict_glucose_rf(
                baseline,
                drop_height,
                min_voltage,
                area_under,
                slope,
                time_to_recovery,
                peak_to_peak);

            Serial.print("Predicted Glucose: ");
            Serial.print(glucose);
            Serial.println(" mmol/L\n\n");

            drop_detected = false;
            collecting_recovery = false;
            recovery_index = false;
            glucose_result = glucose;
            done = true;
        }
    }
}

float predict_glucose_rf(
    float baseline,
    float drop_height,
    float min_voltage,
    float area_under,
    float slope,
    float time_to_recovery,
    float peak_to_peak)
{
    float x[7] = {
        baseline,
        drop_height,
        min_voltage,
        area_under,
        slope,
        time_to_recovery,
        peak_to_peak};

    Eloquent::ML::Port::RandomForestRegressor model;
    return model.predict(x);
}

void setup()
{
    delay(2000);
    pinMode(buttonPin, INPUT_PULLUP);
    lastState = digitalRead(buttonPin);
    Wire.begin();
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR);
    display.setFont(&FreeSansOblique12pt7b);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);

    ADS.begin();
    ADS.setGain(1);
    ADS.setDataRate(7);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    display.clearDisplay();
    display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void loop()
{

    // --- BUTTON LOGIC ---
    bool currentState = digitalRead(buttonPin);

    if (currentState != lastState)
    {
        delay(50);
        if (digitalRead(buttonPin) == LOW)
        {
            logging = !logging;
        }
        if (logging)
        {
            scrollX = SCREEN_WIDTH;
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        else
        {
            display.clearDisplay();
            display.display();
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            drop_detected = false;
            collecting_recovery = false;
            recovery_index = false;
            done = false;
        }
        lastState = currentState;
    }

    // --- LOGGING LOOP ---
    if (logging)
    {
        display.ssd1306_command(SSD1306_DISPLAYON);
        if (millis() - lastLogTime >= logInterval && !done)
        {
            lastLogTime = millis();
            int16_t raw = ADS.readADC(0);
            float voltage = ADS.toVoltage(raw);
            float elapsed = (millis() - startTime) / 1000.0;
            measureGlucose(voltage);
        }
        if (done)
        {
            drawResult(glucose_result);
        }
        else if (drop_detected)
        {
            drawCountdown();
        }
        else
        {
            drawIntroDisplay();
        }
    }
}
