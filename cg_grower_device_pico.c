#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define ADC_PIN_GP26_ADC0 26
#define ADC_INPUT_0 0
// #define ADC_INPUT_4 4
#define WATER_PUMP_PIN 22
#define WATER_PUMP_RUN_PIN 3
#define TEMP_SENSOR_DS18B20_PIN 2

#define MOVING_AVERAGE_PERIOD 32
#define SENSOR_READINGS_NUMBER 8

#define MIN_MOISTURE_READING 3385 // The sensor is dry
#define MAX_MOISTURE_READING 130  // The sensor is completely in water

float get_mean(float sensor_readings[], float *sum, float readings_seq[], uint8_t *curr_indx, bool *is_seq_filled)
{
    for (int i = 0; i < SENSOR_READINGS_NUMBER; i++)
    {
        *is_seq_filled = *is_seq_filled || MOVING_AVERAGE_PERIOD - *curr_indx == 1;
        *sum -= readings_seq[*curr_indx];
        readings_seq[*curr_indx] = sensor_readings[i];
        *sum += readings_seq[*curr_indx];
        *curr_indx = ++(*curr_indx) % MOVING_AVERAGE_PERIOD;
    }
    return *is_seq_filled ? *sum / MOVING_AVERAGE_PERIOD : *sum / *curr_indx;
}

int test_device_presence_1_wire(uint8_t pin)
{
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 1);
    sleep_ms(1);
    gpio_put(pin, 0);
    sleep_us(480);
    gpio_set_dir(pin, GPIO_IN);
    sleep_us(70);
    int pres_flag = gpio_get(pin); // pres_flag == 0 if the device is present
    sleep_us(410);
    return pres_flag;
}

void write_bit_1_wire(uint8_t pin, int b)
{
    int delay1, delay2;
    if (b == 1)
    {
        delay1 = 6;
        delay2 = 64;
    }
    else
    {
        delay1 = 60;
        delay2 = 10;
    }
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    sleep_us(delay1);
    gpio_set_dir(pin, GPIO_IN);
    sleep_us(delay2);
}

void write_byte_1_wire(uint8_t pin, int byte)
{
    for (int i = 0; i < 8; i++)
    {
        if (byte & 1)
        {
            write_bit_1_wire(pin, 1);
        }
        else
        {
            write_bit_1_wire(pin, 0);
        }
        byte = byte >> 1;
    }
}

uint8_t read_bit_1_wire(uint8_t pin)
{
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    sleep_us(8);
    gpio_set_dir(pin, GPIO_IN);
    sleep_us(2);
    uint8_t b = gpio_get(pin);
    sleep_us(60);
    return b;
}

int read_byte_1_wire(uint8_t pin)
{
    int byte = 0;
    for (int i = 0; i < 8; i++)
    {
        byte = byte | read_bit_1_wire(pin) << i;
    }
    return byte;
}

uint8_t crc8_1_wire(uint8_t *data, uint8_t len)
{
    uint8_t i, j, temp, databype;
    uint8_t crc = 0;
    for (i = 0; i < len; i++)
    {
        databype = data[i];
        for (j = 0; j < 8; j++)
        {
            temp = (crc ^ databype) & 0x01;
            crc >>= 1;
            if (temp)
            {
                crc ^= 0x8C;
            }
            databype >>= 1;
        }
    }
    return crc;
}

int convert_1_wire(uint8_t pin)
{
    write_byte_1_wire(pin, 0x44);
    int i;
    for (i = 0; i < 500; i++)
    {
        sleep_ms(10);
        if (read_bit_1_wire(pin) == 1)
            break;
    }
    return i;
}

float get_temperature(uint8_t pin)
{
    if (test_device_presence_1_wire(pin) == 1)
    {
        return -1000;
    }
    write_byte_1_wire(pin, 0xCC);
    if (convert_1_wire(pin) == 500)
    {
        return -3000;
    }
    test_device_presence_1_wire(pin);
    write_byte_1_wire(pin, 0xCC);
    write_byte_1_wire(pin, 0xBE);
    int i;
    uint8_t data[9];
    for (i = 0; i < 9; i++)
    {
        data[i] = read_byte_1_wire(pin);
    }
    uint8_t crc = crc8_1_wire(data, 9);
    if (crc != 0)
    {
        return -2000;
    }
    int t1 = data[0];
    int t2 = data[1];
    int16_t temp1 = (t2 << 8 | t1);
    float temp = (float)temp1 / 16;
    return temp;
}

void read_moisture_raw(float sensor_seq_readings[])
{
    for (int i = 0; i < SENSOR_READINGS_NUMBER; i++)
    {
        sensor_seq_readings[i] = adc_read();
        printf("---------- ADC value: %f\n", sensor_seq_readings[i]);
        sleep_ms(2000);
    }
}

float get_moisture_percent(int moisture_raw)
{
    //  SEE: https://www.arduino.cc/reference/en/language/functions/math/map/
    return 100.0 * (moisture_raw - MIN_MOISTURE_READING) / (MAX_MOISTURE_READING - MIN_MOISTURE_READING);
}

void read_temperature(float sensor_seq_readings[])
{
    for (int i = 0; i < SENSOR_READINGS_NUMBER; i++)
    {
        do
        {
            sensor_seq_readings[i] = get_temperature(TEMP_SENSOR_DS18B20_PIN);
        } while (sensor_seq_readings[i] < -999);
    }
}

static char event_str[128];

void gpio_event_string(char *buf, uint32_t events);

void gpio_callback(uint gpio, uint32_t events)
{
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    printf("GPIO %d %s\n", gpio, event_str);
}

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events)
{
    for (uint i = 0; i < 4; i++)
    {
        uint mask = (1 << i);
        if (events & mask)
        {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0')
            {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events)
            {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

int main()
{
    stdio_init_all();

    float sensor_readings[SENSOR_READINGS_NUMBER] = {0};
    float soil_moisture_seq[MOVING_AVERAGE_PERIOD] = {0};
    float temperature_seq[MOVING_AVERAGE_PERIOD] = {0};
    float soil_moisture_seq_sum = 0;
    float temperature_seq_sum = 0;
    uint8_t soil_moisture_curr_indx = 0;
    uint8_t temperature_curr_indx = 0;
    bool is_soil_moisture_seq_filled = false;
    bool is_temperature_seq_filled = false;

    char pump_state[] = "off";

    adc_init();
    adc_gpio_init(ADC_PIN_GP26_ADC0);
    adc_select_input(ADC_INPUT_0);
    // adc_set_temp_sensor_enabled(true);
    // adc_select_input(ADC_INPUT_4);

    gpio_init(TEMP_SENSOR_DS18B20_PIN); // Init PIN for 1-Wire device (DS18B20 temp sensor)

    float soil_moisture_raw, soil_moisture_percent, temp;

    gpio_set_function(WATER_PUMP_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(WATER_PUMP_PIN, true);

    gpio_set_function(WATER_PUMP_RUN_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(WATER_PUMP_RUN_PIN, false);
    gpio_pull_down(WATER_PUMP_RUN_PIN);
    gpio_set_irq_enabled_with_callback(WATER_PUMP_RUN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    for (;;)
    {
        read_moisture_raw(sensor_readings);
        soil_moisture_raw = get_mean(sensor_readings, &soil_moisture_seq_sum, soil_moisture_seq, &soil_moisture_curr_indx, &is_soil_moisture_seq_filled);
        soil_moisture_percent = get_moisture_percent(soil_moisture_raw);
        printf("********** Soil Moisture, ADC mean: %f\t\tSoil Moisture, %%: %0.2f\n", soil_moisture_raw, soil_moisture_percent);

        read_temperature(sensor_readings);
        temp = get_mean(sensor_readings, &temperature_seq_sum, temperature_seq, &temperature_curr_indx, &is_temperature_seq_filled);
        float soil_moisture_percent = get_moisture_percent(soil_moisture_raw);
        printf("++++++++++ Temperature: %0.2f\n", temp);
    }
}
