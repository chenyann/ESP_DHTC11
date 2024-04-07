#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define DHTC11_PIN         GPIO_NUM_47

#define DELAY_MS(ms)       vTaskDelay(pdMS_TO_TICKS(ms))
#define DELAY_US(us)       vTaskDelay(pdMS_TO_TICKS(us / 1000))

#define HDCSDA_Input()     gpio_set_direction(DHTC11_PIN, GPIO_MODE_INPUT)// 将SDA引脚设置为输入模式
#define HDCSDA_Output()    gpio_set_direction(DHTC11_PIN, GPIO_MODE_OUTPUT)// 将SDA引脚设置为输出模式

#define HDCSDA_SET()       gpio_set_level(DHTC11_PIN, 1)// 将SDA引脚输出高电平
#define HDCSDA_CLR()       gpio_set_level(DHTC11_PIN, 0)// 将SDA引脚输出低电平
#define HDCGet_SDA()       gpio_get_level(DHTC11_PIN)   // 获取SDA引脚的当前状态

static const char *TAG = "DHTC11 Sensor";

// 发送复位信号给传感器
void DQ_Rst(void) {
    HDCSDA_Output();
    DELAY_US(2);
    HDCSDA_CLR();
    DELAY_US(500);//>480us 典型值960us  tRSTL
    HDCSDA_SET();
    DELAY_US(7);
}

// 检测传感器响应
uint8_t DQ_Presence(void) {
    uint8_t pulse_time = 0;
    HDCSDA_Input();
    DELAY_US(2);
    //存在检测高电平15~60us  tPDHIGH
    while( (HDCGet_SDA()) && pulse_time<100 ) {
        pulse_time++;
        DELAY_US(5);
    }
    if (pulse_time >= 10)
        return 0x01;
    else
        pulse_time = 0;
    //存在检测低电平时间60~240us  tPDLOW
    while((HDCGet_SDA()==0) && pulse_time<240 ) {
        pulse_time++;
        DELAY_US(2);
    }
    if (pulse_time >= 10)
        return 0x01;
    else
        return 0x0;
}

// 从总线读取一个位
uint8_t DQ_Read_Bit(void) {
    uint8_t dat;
    HDCSDA_Output();
    HDCSDA_CLR();
    DELAY_US(2);
    HDCSDA_Input();
    DELAY_US(5);
    if (HDCGet_SDA())
        dat = 1;
    else
        dat = 0;
    DELAY_US(33);
    return dat;
}

// 从总线读取一个字节
uint8_t DQ_Read_Byte(void) {
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++) {
        dat |= (DQ_Read_Bit() << i);
    }
    return dat;
}

// 向总线写入一个位
void DQ_Write_Bit(uint8_t bit) {
    uint8_t testb;
    testb = bit&0x01;
    HDCSDA_Output();
    if(testb)//写1
    {
        HDCSDA_CLR();
        DELAY_US(5);//>1us  <15us
        HDCSDA_SET();
        DELAY_US(60);//>=60
    }
    else//写0
    {
        HDCSDA_CLR();
        DELAY_US(60);//>=60us
        HDCSDA_SET();
        DELAY_US(5);//典型5us
    }
}

// 向总线写入一个字节
static void DQ_Write_Byte(uint8_t dat) {
    HDCSDA_Output();
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t testb = dat & 0x01;
        dat >>= 1;
        if (testb) { // 写1
            HDCSDA_CLR();
            DELAY_US(5); // 大于1us，小于15us
            HDCSDA_SET();
            DELAY_US(10); // 典型为10us
        } else { // 写0
            HDCSDA_CLR();
            DELAY_US(15); // 大于15us
            HDCSDA_SET();
            DELAY_US(1); // 典型为1us
        }
    }
}

// CRC8 校验计算
uint8_t CRC8MHT_Cal(uint8_t *serial, uint8_t length) {
    uint8_t result = 0x00;
    uint8_t pDataBuf;
    uint8_t i;

    while (length--) {
        pDataBuf = *serial++;
        for (i = 0; i < 8; i++) {
            if ((result ^ (pDataBuf)) & 0x01) {
                result ^= 0x18;
                result >>= 1;
                result |= 0x80;
            } else {
                result >>= 1;
            }
            pDataBuf >>= 1;
        }
    }
    return result;
}

// 传感器初始化函数
static int16_t OwHumA,OwHumB;
void HTMC01_MInit_OW(void) {
    DQ_Rst();
    DQ_Presence();
    DQ_Write_Byte(0xcc);
    DQ_Write_Byte(0xdd);
    // 读取 OwHumA 和 OwHumB
    OwHumA = DQ_Read_Byte();
    OwHumA = (OwHumA << 8) | DQ_Read_Byte();
    OwHumB = DQ_Read_Byte();
    OwHumB = (OwHumB << 8) | DQ_Read_Byte();
}

uint16_t calculateHumidity(int16_t temperature, int16_t temperatureOffset, int16_t rawHumidity) {
    uint16_t humidity;
    // 根据提供的公式计算湿度
    humidity = (rawHumidity - temperatureOffset) * 600 / (temperature + temperatureOffset) + 300;
    // 20℃为5个湿度点  即1℃为0.25个湿度点 0.1℃ 为0.025
    humidity += 25 * (temperature - 200) / 10;
    if (humidity > 999) {
        humidity = 999;
    } else if (humidity < 0) {
        humidity = 0;
    }
    return humidity;
}


// 读取传感器数据函数
uint8_t ReadMDC04CapTem_onewire(int16_t *tem, uint16_t *Cap) {
    uint8_t ResDat[5], crc = 0;
    int16_t TemBuf;
    int16_t CapBuf;

    DQ_Rst();
    DQ_Presence();
    DQ_Write_Byte(0xcc);
    DQ_Write_Byte(0x10);
    DELAY_MS(35);

    DQ_Rst();
    DQ_Presence();
    DQ_Write_Byte(0xcc);
    DQ_Write_Byte(0xbd);
    ResDat[0] = DQ_Read_Byte();
    ResDat[1] = DQ_Read_Byte();
    ResDat[2] = DQ_Read_Byte();
    ResDat[3] = DQ_Read_Byte();
    ResDat[4] = DQ_Read_Byte();

    crc = CRC8MHT_Cal(ResDat, 5);
    if (crc == 0) {
        TemBuf = (int16_t)(ResDat[1] << 8 | ResDat[0]);
        TemBuf = 400 + TemBuf / 25.6;
        *tem = TemBuf;

        CapBuf = (int16_t)(ResDat[3] << 8 | ResDat[2]);

        // CapBuf = ((float)CapBuf-OwHumB)*600/(OwHumA-OwHumB)+300;//同样结果*10
        // //20℃为5个湿度点  即1℃为0.25个湿度点 0.1℃ 为0.025
        // CapBuf = CapBuf+ 25*(TemBuf-250)/100;	

        // // 根据提供的公式计算湿度
        // if (CapBuf > 999)
        //     CapBuf = 999;
        // else if (CapBuf < 0)
        //     CapBuf = 0;

        CapBuf = calculateHumidity(TemBuf, 250, CapBuf);

        *Cap = (uint16_t)CapBuf;
    }
    return crc;
}


// 任务函数，用于周期性读取传感器数据
void dht_task(void *pvParameter) {
    int16_t temperature;
    uint16_t humidity;

    while (1) {
        // 从传感器读取温度和湿度数据
        if (ReadMDC04CapTem_onewire(&temperature, &humidity) == 0) {
            ESP_LOGI(TAG, "Temperature: %d.%d °C, Humidity: %d.%d%%", temperature / 10, temperature % 10, humidity / 10, humidity % 10);
        } else {
            ESP_LOGE(TAG, "CRC error occurred while reading sensor data");
        }
        DELAY_MS(1000); // 延迟 1 秒
    }
}

// 主函数入口
void app_main() {
    // 初始化 GPIO
    esp_rom_gpio_pad_select_gpio(DHTC11_PIN);
    HDCSDA_Output();

    // 创建任务用于读取传感器数据
    xTaskCreate(dht_task, "dht_task", 2048, NULL, 5, NULL);
}
