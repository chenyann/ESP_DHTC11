#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define HDCSDA_PIN GPIO_NUM_47

#define DELAY_US(us)       esp_rom_delay_us(us)
#define DELAY_MS(ms)       vTaskDelay(pdMS_TO_TICKS(ms))

static const char* TAG = "DHTC11 Sensor";

// 定义传感器引脚以及操作函数
#define HDCSDA_Input()     gpio_set_direction(HDCSDA_PIN, GPIO_MODE_INPUT)// 将SDA引脚设置为输入模式
#define HDCSDA_Output()    gpio_set_direction(HDCSDA_PIN, GPIO_MODE_OUTPUT)// 将SDA引脚设置为输出模式

#define HDCSDA_SET()       gpio_set_level(HDCSDA_PIN, 1)// 将SDA引脚输出高电平
#define HDCSDA_CLR()       gpio_set_level(HDCSDA_PIN, 0)// 将SDA引脚输出低电平
#define HDCGet_SDA()       gpio_get_level(HDCSDA_PIN)   // 获取SDA引脚的当前状态

// 发送复位信号给传感器
void DQ_Rst(void)
{
    HDCSDA_Output();
    DELAY_US(2);
    HDCSDA_CLR();
    DELAY_US(960);//>480us 典型值960us 复位低电平时间tRSTL
    HDCSDA_SET();
    DELAY_US(10);
}

// 检测传感器响应
uint8_t DQ_Presence(void)
{
    uint8_t pulse_time = 0;
    HDCSDA_Input();
    DELAY_US(2);
    while ((HDCGet_SDA()) && pulse_time < 100)
    {//存在检测高电平15~60us tPDHIGH 典型值为30us
        pulse_time++;
        DELAY_US(3);
    }
    if (pulse_time >= 10)
        return 0x01;
    else
        pulse_time = 0;

    while ((!HDCGet_SDA()) && pulse_time < 240)
    {//存在检测低电平时间60~240us  tPDLOW
        pulse_time++;
        DELAY_US(1);
    }
    if (pulse_time >= 10)
        return 0x01;
    else
        return 0x0;
}

// 向总线写入一个位
void DQ_Write_Bit(uint8_t bit)
{
    uint8_t testb;
    testb = bit & 0x01;
    HDCSDA_Output();
    if (testb)//写1
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
void DQ_Write_Byte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        DQ_Write_Bit(dat & 0x01);
        dat >>= 1;
    }
}

// 从总线读取一个位
uint8_t DQ_Read_Bit(void)
{
    uint8_t dat;
    HDCSDA_Output();
    HDCSDA_CLR();
    DELAY_US(5);//tINIT>1us 典型5us <15us  
    HDCSDA_Input();
    DELAY_US(5);
    if (HDCGet_SDA())
        dat = 1;
    else
        dat = 0;
    DELAY_US(60);//tDelay 60us 确保一帧数据传输完毕
    return dat;
}

// 从总线读取一个字节
uint8_t DQ_Read_Byte(void)
{
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++)
    {
        dat |= (DQ_Read_Bit() << i);
    }
    return dat;
}

// CRC8 校验计算
uint8_t CRC8MHT_Cal(uint8_t* serial, uint8_t length)
{
    uint8_t result = 0x00;
    uint8_t pDataBuf;
    uint8_t i;

    while (length--)
    {
        pDataBuf = *serial++;
        for (i = 0; i < 8; i++)
        {
            if ((result ^ (pDataBuf)) & 0x01)
            {
                result ^= 0x18;
                result >>= 1;
                result |= 0x80;
            }
            else
            {
                result >>= 1;
            }
            pDataBuf >>= 1;
        }
    }
    return result;
}

// 传感器初始化函数
static int16_t OwHumA, OwHumB;
void HTMC01_MInit_OW(void)
{
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

// 读取传感器数据函数
uint8_t ReadMDC04CapTem_onewire(int16_t* tem, uint16_t* Cap)
{
    uint8_t ResDat[5], crc = 0;
    int16_t TemBuf;
    int16_t CapBuf;
    HTMC01_MInit_OW();

    DQ_Rst();
    DQ_Presence();
    DQ_Write_Byte(0xcc);
    DQ_Write_Byte(0x10);//触发温湿度转化
    // DELAY_MS(7075);
    DELAY_MS(35);

    DQ_Rst();
    DQ_Presence();
    DQ_Write_Byte(0xcc);
    DQ_Write_Byte(0xbd);//获取温湿度转化结果
    ResDat[0] = DQ_Read_Byte();
    ResDat[1] = DQ_Read_Byte();
    ResDat[2] = DQ_Read_Byte();
    ResDat[3] = DQ_Read_Byte();
    ResDat[4] = DQ_Read_Byte();

    crc = CRC8MHT_Cal(ResDat, 5);
    if (crc == 0)
    {
        TemBuf = (int16_t)(ResDat[1] << 8 | ResDat[0]);
        TemBuf = 400 + TemBuf / 25.6;//*10 结果*10倍 286即28.6℃
        *tem = TemBuf;

        CapBuf = (int16_t)(ResDat[3] << 8 | ResDat[2]);

        if ((OwHumA - OwHumB) != 0){
            CapBuf = ((float)CapBuf - OwHumB) * 600 / (OwHumA - OwHumB) + 300;//同样结果*10
        }
        else{
            CapBuf = 555;// 除数为零时，为湿度设置一个默认值
            ESP_LOGE(TAG, "OwHumA - OwHumB = 0,use as default");
        }
        // 根据提供的公式计算湿度
        //20℃为5个湿度点  即1℃为0.25个湿度点 0.1℃ 为0.025
        CapBuf = CapBuf + 25 * (TemBuf - 250) / 100;

        if (CapBuf > 999)
            CapBuf = 999;
        else if (CapBuf < 0)
            CapBuf = 0;
        *Cap = (uint16_t)CapBuf;
    }
    return crc;
}

// 任务函数，用于周期性读取传感器数据
void dht_task(void* pvParameter)
{
    int16_t temperature;
    uint16_t humidity;

    while (1)
    {
        // 从传感器读取温度和湿度数据
        if (ReadMDC04CapTem_onewire(&temperature, &humidity) == 0)
        {
            ESP_LOGI(TAG, "Temperature: %d.%d °C, Humidity: %d.%d%%", temperature / 10, temperature % 10, humidity / 10, humidity % 10);
        }
        else
        {
            ESP_LOGE(TAG, "CRC error occurred while reading sensor data");
        }
        DELAY_MS(2000); // 延迟 2 秒
    }
}

// 主函数入口
void app_main()
{
    // 初始化 GPIO
    esp_rom_gpio_pad_select_gpio(HDCSDA_PIN);
    HDCSDA_Output();

    // 创建任务用于读取传感器数据
    xTaskCreate(dht_task, "dht_task", 2048, NULL, 5, NULL);
}