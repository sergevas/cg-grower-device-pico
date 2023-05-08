#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "lwip/apps/http_client.h"

#define ADC_PIN_GP26_ADC0 26
#define ADC_INPUT_0 0
// #define ADC_INPUT_4 4
#define WATER_PUMP_PIN 22
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

int init_wifi_connection(uint32_t country, const char *ssid, const char *password, uint32_t auth)
{
    if (cyw43_arch_init_with_country(country))
    {
        printf("Failed to init WiFi with country\n");
        return 1;
    }
    printf("Success init WiFi with country\n");
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_async(ssid, password, auth))
    {
        printf("Failed to establish WiFi connection\n");
        return 2;
    }
}

int setup_wifi_connection()
{
    int status = CYW43_LINK_UP + 1;
    while (status != CYW43_LINK_UP && status != CYW43_LINK_BADAUTH)
    {
        int new_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (new_status != status)
        {
            status = new_status;
            printf("connect status: %d %d\n", status);
        }
    }
    if (status < 0)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    else
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }
    return status;
}

// HTTP client implementation start
char buff[1000];

void result(void *arg, httpc_result_t httpc_result, u32_t rx_content_len, u32_t srv_res, err_t err)
{
    printf("transfer complete\n");
    printf("local result=%d\n", httpc_result);
    printf("http result=%d\n", srv_res);
    if (httpc_result != HTTPC_RESULT_OK)
    {
        printf("Trying to reconnect...\n");
        setup_wifi_connection();
    }
}

err_t headers(httpc_state_t *connection, void *arg, struct pbuf *hdr, u16_t hdr_len, u32_t content_len)
{
    printf("headers recieved\n");
    printf("content length=%d\n", content_len);
    printf("header length %d\n", hdr_len);

    pbuf_copy_partial(hdr, buff, hdr->tot_len, 0);
    printf("headers \n");
    printf("%s", buff);
    return ERR_OK;
}

err_t body(void *arg, struct altcp_pcb *conn, struct pbuf *p, err_t err)
{
    printf("body\n");
    pbuf_copy_partial(p, buff, p->tot_len, 0);
    pbuf_free(p);
    printf("%s", buff);
    return ERR_OK;
}
// HTTP client implementation complete

void set_get_command_uri(char *uri, const char *device_id, float temp, float moist, char *pump)
{
    // URI example: http://192.168.1.76:8000/cg/gateway/device/grower/0001?tmp=25.01&mst=56.4&pump=off
    sprintf(uri, "/cg/gateway/device/grower/%s?tmp=%0.2f&mst=%0.2f&pump=%s", device_id, temp, moist, pump);
}

uint32_t country = CYW43_COUNTRY_UK;
char ssid[] = "IoT";
char password[] = "VeryL0ngPas$wd!2015";
uint32_t auth = CYW43_AUTH_WPA2_MIXED_PSK;
const int port = 8085;
char device_id[] = "0002";
char uri[53];

int main()
{
    stdio_init_all();
    init_wifi_connection(country, ssid, password, auth);
    ip_addr_t ip;
    IP4_ADDR(&ip, 192, 168, 1, 87);
    httpc_connection_t settings;
    settings.result_fn = result;
    settings.headers_done_fn = headers;

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

    sleep_ms(10000);

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

        set_get_command_uri(uri, device_id, temp, soil_moisture_percent, pump_state);
        err_t err = httpc_get_file(
            &ip,
            port,
            uri,
            &settings,
            body,
            NULL,
            NULL);
        printf("status %d \n", err);
        if (err != ERR_OK)
        {
            printf("Trying to reconnect...\n");
            setup_wifi_connection();
            err_t err = httpc_get_file(
                &ip,
                port,
                uri,
                &settings,
                body,
                NULL,
                NULL);
            printf("status %d \n", err);
        }
        sleep_ms(30000);
    }
    return 0;
}
