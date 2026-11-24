/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ============================================================================
// ค่าตั้งต้นวิทยุ LoRa (สำคัญมาก: ต้องตั้งให้ตรงกับไฟล์ LoRa_functions ของฝั่งจรวดเป๊ะ!)
// ============================================================================
#define LORA_FREQUENCY_HZ        433000000UL // ความถี่ 433 MHz (สำหรับ Ra-01 / SX1278)
#define LORA_SYNC_WORD           0x12        // รหัสกลุ่มเครือข่าย (ค่าเริ่มต้น SX1278 = 0x12)
#define LORA_TX_POWER_REG        0x8F        // PA_BOOST ON + กำลังส่ง 17 dBm (Ra-01 ใช้ขา PA_BOOST)
#define LORA_SEND_PERIOD_MS      3000U       // ทดสอบส่งข้อความกลับไปหาจรวดทุกๆ 3 วินาที

// แอดเดรสรีจิสเตอร์หลักของชิป SX1278 (Ra-01)
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0B
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// โหมดการทำงานของ SX1278 (บิต 7 = 1 คือโหมด LoRa)
#define MODE_LORA_SLEEP          0x80
#define MODE_LORA_STDBY          0x81
#define MODE_LORA_TX             0x83
#define MODE_LORA_RX_CONTINUOUS  0x85

// บิตสถานะ Interrupt ใน REG_IRQ_FLAGS
#define IRQ_TX_DONE_MASK         0x08
#define IRQ_PAYLOAD_CRC_ERR_MASK 0x20
#define IRQ_RX_DONE_MASK         0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t  lora_rx_buf[255];      // บัฟเฟอร์เก็บข้อความที่รับมาจากจรวด (สูงสุด 255 เพื่อไม่ให้เกินค่า uint8_t)
char     lora_tx_buf[128];      // บัฟเฟอร์เก็บข้อความที่จะส่งไปหาจรวด
uint8_t  lora_ready = 0;        // สถานะเชื่อมต่อ Ra-01 (1 = พร้อม, 0 = ไม่เจอชิป)
uint32_t tx_counter = 0;        // ตัวนับจำนวนแพ็กเก็ตที่ส่งออกไป
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
void LoRa_WriteReg(uint8_t reg, uint8_t val);
uint8_t LoRa_ReadReg(uint8_t reg);
void LoRa_Reset(void);
void LoRa_SetFrequency(uint32_t freq_hz);
bool LoRa_Init(void);
void LoRa_StartReceive(void);
bool LoRa_Send(const uint8_t *data, uint8_t len, uint32_t timeout_ms);
uint8_t LoRa_Receive(uint8_t *buf, uint8_t max_len);
int16_t LoRa_GetRSSI(void);
float LoRa_GetSNR(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ฟังก์ชันพิมพ์ข้อความออก USART1 (ดูผ่าน Serial Monitor ที่ 115200 bps)
void myprintf(const char *fmt, ...)
{
    static char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}

// เขียนค่า 1 ไบต์ลงรีจิสเตอร์ของ Ra-01 ผ่าน SPI1
void LoRa_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80U), val }; // บิต 7 = 1 แปลว่าคำสั่ง "เขียน"
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET); // ดึง CS ลงเพื่อเลือกชิป
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);   // ปล่อย CS ขึ้นเมื่อจบ
}

// อ่านค่า 1 ไบต์จากรีจิสเตอร์ของ Ra-01 ผ่าน SPI1
uint8_t LoRa_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), 0x00U }; // บิต 7 = 0 แปลว่าคำสั่ง "อ่าน"
    uint8_t rx[2] = { 0 };
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
    return rx[1]; // ไบต์ที่ 2 คือข้อมูลที่ชิปตอบกลับมา
}

// รีเซ็ตฮาร์ดแวร์ Ra-01 ก่อนเริ่มตั้งค่า
void LoRa_Reset(void)
{
    HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

// คำนวณและตั้งค่าความถี่วิทยุ (สูตร SX1278: FRF = (Freq * 2^19) / 32MHz)
void LoRa_SetFrequency(uint32_t freq_hz)
{
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ULL;
    LoRa_WriteReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_WriteReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_WriteReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

// เริ่มต้นการทำงานและตั้งค่าพารามิเตอร์ทั้งหมดของ Ra-01
bool LoRa_Init(void)
{
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
    LoRa_Reset();

    // 1. ตรวจสอบเวอร์ชันชิป SX1278 ต้องอ่านได้ 0x12 (เหมือนโค้ดฝั่งจรวด)
    uint8_t version = LoRa_ReadReg(REG_VERSION);
    myprintf("[LoRa] RegVersion = 0x%02X (expected 0x12)\r\n", version);
    if (version != 0x12) {
        return false; // ต่อสาย SPI ผิด หรือไฟเลี้ยง 3.3V ไม่เข้า
    }

    // 2. เข้าโหมด Sleep ก่อนเปลี่ยนเป็นโหมด LoRa (ข้อบังคับของ SX1278)
    LoRa_WriteReg(REG_OP_MODE, MODE_LORA_SLEEP);
    HAL_Delay(10);

    // 3. ตั้งความถี่ 433 MHz
    LoRa_SetFrequency(LORA_FREQUENCY_HZ);

    // 4. ตั้งตำแหน่งเริ่มต้นของหน่วยความจำ FIFO สำหรับรับและส่งให้ใช้ได้เต็ม 256 ไบต์
    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
    LoRa_WriteReg(REG_FIFO_RX_BASE_ADDR, 0x00);

    // 5. เปิด LNA Boost (เพิ่มความไวภาครับ) และเปิด AGC (ปรับเกนอัตโนมัติ)
    LoRa_WriteReg(REG_LNA, LoRa_ReadReg(REG_LNA) | 0x03);
    LoRa_WriteReg(REG_MODEM_CONFIG_3, 0x04);

    // 6. ตั้งค่าโมเด็ม: BW = 125 kHz (0x70), CR = 4/5 (0x02), Explicit Header (0x00)
    LoRa_WriteReg(REG_MODEM_CONFIG_1, 0x72);

    // 7. ตั้งค่าโมเด็ม: SF = 7 (0x70), เปิดตรวจสอบความถูกต้องข้อมูล CRC (0x04)
    LoRa_WriteReg(REG_MODEM_CONFIG_2, 0x74);

    // 8. ตั้ง Preamble = 8 symbols และ Sync Word = 0x12
    LoRa_WriteReg(REG_PREAMBLE_MSB, 0x00);
    LoRa_WriteReg(REG_PREAMBLE_LSB, 0x08);
    LoRa_WriteReg(REG_SYNC_WORD, LORA_SYNC_WORD);

    // 9. ตั้งกำลังส่ง 17 dBm และตั้งระบบป้องกันกระแสเกิน (OCP) ที่ 100 mA
    LoRa_WriteReg(REG_OCP, 0x2B);
    LoRa_WriteReg(REG_PA_CONFIG, LORA_TX_POWER_REG);

    // 10. ล้างธงแจ้งเตือน (IRQ) ทั้งหมด แล้วสลับไปโหมด Standby
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_WriteReg(REG_OP_MODE, MODE_LORA_STDBY);
    return true;
}

// สั่งให้ Ra-01 เปิดหูรอรับข้อมูลตลอดเวลา (RX Continuous)
void LoRa_StartReceive(void)
{
    LoRa_WriteReg(REG_DIO_MAPPING_1, 0x00); // ให้ขา DIO0 แจ้งเตือนเมื่อ RxDone
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);     // ล้างธงเก่าทิ้ง
    LoRa_WriteReg(REG_OP_MODE, MODE_LORA_RX_CONTINUOUS);
}

// ฟังก์ชันส่งข้อมูลออกทาง LoRa (ส่งเสร็จจะสลับกลับมารอรับอัตโนมัติ)
bool LoRa_Send(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    if (len == 0) return false;

    // 1. พักภาครับเข้าโหมด Standby ก่อนเขียนข้อมูลลง FIFO
    LoRa_WriteReg(REG_OP_MODE, MODE_LORA_STDBY);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, 0x00);
    LoRa_WriteReg(REG_PAYLOAD_LENGTH, len);

    // 2. เขียนชุดข้อมูล (Burst Write) ลงใน FIFO
    uint8_t addr = REG_FIFO | 0x80U;
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100);
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);

    // 3. ล้างธงเก่าแล้วสั่งเริ่มส่งสัญญาณวิทยุ (TX Mode)
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_WriteReg(REG_OP_MODE, MODE_LORA_TX);

    // 4. รอจนกว่าบิต TxDone จะติด หรือครบเวลา timeout
    uint32_t start = HAL_GetTick();
    while ((LoRa_ReadReg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            LoRa_StartReceive(); // ส่งไม่ทันเวลา ให้กลับไปรอรับ
            return false;
        }
    }

    // 5. ส่งสำเร็จ -> ล้างธงแล้วกลับไปเปิดรอรับข้อมูลทันที
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_StartReceive();
    return true;
}

// ฟังก์ชันเช็กและอ่านข้อมูลที่รับเข้ามา (คืนค่าจำนวนไบต์ที่รับได้, 0 = ยังไม่มีข้อมูล)
uint8_t LoRa_Receive(uint8_t *buf, uint8_t max_len)
{
    if (max_len < 2) return 0;

    // เช็กว่ามีแพ็กเก็ตเข้ามารึยัง (อ่านจากรีจิสเตอร์โดยตรง ทำงานได้แม้ยังไม่ต่อขา DIO0)
    uint8_t irq = LoRa_ReadReg(REG_IRQ_FLAGS);
    if ((irq & IRQ_RX_DONE_MASK) == 0) {
        return 0;
    }

    // ล้างธงแจ้งเตือนเพื่อเตรียมรับแพ็กเก็ตถัดไป
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);

    // ถ้า CRC ผิดพลาด (สัญญาณเพี้ยน/รบกวน) ให้ทิ้งแพ็กเก็ตนี้ไป
    if (irq & IRQ_PAYLOAD_CRC_ERR_MASK) {
        myprintf("[LoRa] RX CRC Error!\r\n");
        return 0;
    }

    // อ่านความยาวข้อมูลและตำแหน่งเริ่มต้นใน FIFO
    uint8_t rx_len = LoRa_ReadReg(REG_RX_NB_BYTES);
    uint8_t rx_addr = LoRa_ReadReg(REG_FIFO_RX_CURRENT_ADDR);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, rx_addr);

    // กันเขียนเกินขนาดบัฟเฟอร์ (เผื่อ 1 ไบต์ไว้ปิดท้าย '\0' สำหรับพิมพ์เป็นข้อความ)
    uint8_t copy_len = (rx_len < (max_len - 1)) ? rx_len : (max_len - 1);

    // อ่านชุดข้อมูล (Burst Read) ออกจาก FIFO
    uint8_t addr = REG_FIFO & 0x7FU;
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
    HAL_SPI_Receive(&hspi1, buf, copy_len, 100);
    HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);

    buf[copy_len] = '\0'; // ปิดท้ายสตริงเพื่อให้ printf แสดงผลได้ทันที
    return copy_len;
}

// อ่านความแรงสัญญาณ RSSI (หน่วย dBm: ยิ่งใกล้ 0 ยิ่งแรง เช่น -60 แรงกว่า -110)
int16_t LoRa_GetRSSI(void)
{
    return (int16_t)LoRa_ReadReg(REG_PKT_RSSI_VALUE) - 164; // สูตรย่าน 433 MHz ของ SX1278
}

// อ่านอัตราส่วนสัญญาณต่อสัญญาณรบกวน SNR (หน่วย dB: ค่าบวกยิ่งดี)
float LoRa_GetSNR(void)
{
    return ((int8_t)LoRa_ReadReg(REG_PKT_SNR_VALUE)) / 4.0f;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  myprintf("\r\n=== STM32F411 GROUND STATION (Ra-01) ===\r\n");

  if (LoRa_Init()) {
      lora_ready = 1;
      LoRa_StartReceive(); // เริ่มเปิดรอรับข้อมูลจากบอร์ดจรวดทันที
      myprintf("[LoRa] Init OK! Listening at 433 MHz...\r\n");
  } else {
      lora_ready = 0;
      myprintf("[LoRa] Init FAILED! Check wiring (SPI1, CS=PA4, RST=PB0, 3.3V)\r\n");
  }

  uint32_t last_tx_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (lora_ready)
      {
          // 1. ภาครับ: ตรวจเช็กข้อมูลจากบอร์ดจรวดตลอดเวลา (ทำงานลื่นไม่หน่วง)
          uint8_t rx_len = LoRa_Receive(lora_rx_buf, sizeof(lora_rx_buf));
          if (rx_len > 0) {
              HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // กะพริบไฟ PC13 เมื่อรับข้อมูลได้
              myprintf("[RX %u B | RSSI:%d dBm | SNR:%.1f dB] %s\r\n",
                       rx_len, LoRa_GetRSSI(), LoRa_GetSNR(), (char *)lora_rx_buf);
          }

          // 2. ภาคส่ง: ส่งข้อความทดสอบกลับไปหาบอร์ดจรวดทุกๆ 3 วินาที (Non-Blocking)
          // (ถ้าอยากให้บอร์ดนี้เป็น "ตัวรับอย่างเดียว" ให้คอมเมนต์บล็อก if นี้ทิ้งได้เลย)
          if ((HAL_GetTick() - last_tx_tick) >= LORA_SEND_PERIOD_MS) {
              last_tx_tick = HAL_GetTick();
              tx_counter++;

              int len = snprintf(lora_tx_buf, sizeof(lora_tx_buf),
                                 "GS_PING #%lu", (unsigned long)tx_counter);
              if (len > 0 && LoRa_Send((uint8_t *)lora_tx_buf, (uint8_t)len, 1000U)) {
                  myprintf("[TX OK] %s\r\n", lora_tx_buf);
              } else {
                  myprintf("[TX TIMEOUT]\r\n");
              }
          }
      }
      else
      {
          // ถ้าเชื่อมต่อ Ra-01 ไม่ติดตอนเปิดเครื่อง ให้ลองใหม่ทุก 2 วินาที
          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
          HAL_Delay(2000);
          if (LoRa_Init()) {
              lora_ready = 1;
              LoRa_StartReceive();
              myprintf("[LoRa] Reconnected!\r\n");
          }
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_CS_Pin */
  GPIO_InitStruct.Pin = LoRa_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_RST_Pin */
  GPIO_InitStruct.Pin = LoRa_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_DIO0_Pin */
  GPIO_InitStruct.Pin = LoRa_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LoRa_DIO0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
