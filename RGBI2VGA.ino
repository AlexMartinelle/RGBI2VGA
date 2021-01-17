/*
  Created by Alexander Martinelle (support@cogitarecomputing.com) - www.cogitarecomputing.com
  Copyright (c) 2020-2021 Alexander Martinelle.
  All rights reserved.

  RGBI2VGA is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  RGBI2VGA is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with RGBI2VGA.  If not, see <http://www.gnu.org/licenses/>.
 */

//In fabglconf.h: Set the "#define FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE 0" instead of WIFI_TASK_CORE_ID
#include "fabgl.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "driver/ledc.h"
#include "soc/rtc_wdt.h"

#include "esp_bt.h"
#include <WiFi.h>

static const int s_clockSpeedMHz = 240;

//Input pin definitions
static const int s_hSyncInput = 25;
static const int s_vSyncInput = 26;
static const int s_RInput = 27;
static const int s_GInput = 14;
static const int s_BInput = 32;
static const int s_IInput = 33;
static const gpio_num_t s_i2sD0 = GPIO_NUM_27;
static const gpio_num_t s_i2sD1 = GPIO_NUM_14;
static const gpio_num_t s_i2sD2 = GPIO_NUM_32;
static const gpio_num_t s_i2sD3 = GPIO_NUM_33;
static const gpio_num_t s_i2sHsync = GPIO_NUM_25;
static const gpio_num_t s_i2sVsync = GPIO_NUM_26;
static const uint32_t s_hSyncPin25 = 0x2000000;
static const uint32_t s_vSyncPin26 = 0x4000000;
static const int s_constantHighInput = 0x38;
static const int s_constantLowInput = 0x30;

//FabGL
static fabgl::VGADirectController s_displayController;
static TaskHandle_t  s_mainTaskHandle;

//Pixel buffer 
static const int s_c128ScreenHeight = 200;
static const int s_linesBeforeRealScreenStarts = 58;
//Contains the entinre 640x200 RGBI screen packed into 320x200
static volatile uint8_t s_memBuffer[320*s_c128ScreenHeight]; 
//16 bit values -> FabGL colour conversions
static uint16_t s_colorConv[256];

//I2S Buffers.
static const uint8_t s_bufferCount = 2;
DMA_ATTR static volatile uint8_t* s_rowBuffer[s_bufferCount] __attribute__((aligned(128))); 
DMA_ATTR static lldesc_t s_dmaDesc[s_bufferCount];
static const uint16_t s_inputLineBufferSizeBytes = 3320;

//I2S interrupt
//Used to flag when the interrupt has happened
static volatile unsigned s_isrCount;
static const int ETS_I2S0_INUM = 13;
static intr_handle_t s_isrHandle;
static const uint32_t s_confResetFlags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;

//Queue used to send latest line processed to the other core
static QueueHandle_t s_lineQueue;


//---------------------------I2S setup---------------------------------------------
static esp_err_t DmaDescInit()
{
  //Create 2 buffers switch between every other line
  for (auto i = 0; i < s_bufferCount; ++i)
  {
    s_rowBuffer[i] = (volatile uint8_t *) heap_caps_malloc(s_inputLineBufferSizeBytes, MALLOC_CAP_DMA);
    s_dmaDesc[i].length = s_inputLineBufferSizeBytes;
    s_dmaDesc[i].size = s_inputLineBufferSizeBytes;
    s_dmaDesc[i].owner = 1;
    s_dmaDesc[i].sosf = 1;
    s_dmaDesc[i].buf = s_rowBuffer[i];
    s_dmaDesc[i].offset = 0;
    s_dmaDesc[i].empty = 0;
    s_dmaDesc[i].eof = 1;
    s_dmaDesc[i].qe.stqe_next = 0;
  }
  return ESP_OK;
}

static void IRAM_ATTR I2SIsr(void* arg)
{
  I2S0.int_clr.val = I2S0.int_raw.val;
  ++s_isrCount;
}

static inline void I2SConfReset()
{
  const uint32_t lc_s_confResetFlags = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
  I2S0.lc_conf.val |= lc_s_confResetFlags;
  I2S0.lc_conf.val &= ~lc_s_confResetFlags;
  
  I2S0.conf.val |= s_confResetFlags;
  I2S0.conf.val &= ~s_confResetFlags;
  while (I2S0.state.rx_fifo_reset_back);
}

static void I2SInit()
{
  pinMode(s_hSyncInput, INPUT);
  pinMode(s_vSyncInput, INPUT);
  pinMode(s_RInput, INPUT);
  pinMode(s_GInput, INPUT);
  pinMode(s_BInput, INPUT);
  pinMode(s_IInput, INPUT);

  gpio_num_t pins[] =
  {
          s_i2sD3,
          s_i2sD2,
          s_i2sD1,
          s_i2sD0,
          s_i2sVsync,
          s_i2sHsync
  };

  gpio_config_t conf;
  conf.mode = GPIO_MODE_INPUT;
  conf.pull_up_en = GPIO_PULLUP_DISABLE;
  conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  
  for (auto i = 0; i < sizeof(pins)/sizeof(gpio_num_t); ++i)
  {
      conf.pin_bit_mask = 1LL << pins[i];
      gpio_config(&conf);
  }

  //Setup input pins for I2S.
  gpio_matrix_in(s_i2sD0,    I2S0I_DATA_IN0_IDX, false);
  gpio_matrix_in(s_i2sD1,    I2S0I_DATA_IN1_IDX, false);
  gpio_matrix_in(s_i2sD2,    I2S0I_DATA_IN2_IDX, false);
  gpio_matrix_in(s_i2sD3,    I2S0I_DATA_IN3_IDX, false);

  gpio_matrix_in(s_i2sD0,    I2S0I_DATA_IN4_IDX, false);
  gpio_matrix_in(s_i2sD1,    I2S0I_DATA_IN5_IDX, false);
  gpio_matrix_in(s_i2sD2,    I2S0I_DATA_IN6_IDX, false);
  gpio_matrix_in(s_i2sD3,    I2S0I_DATA_IN7_IDX, false);
  gpio_matrix_in(s_i2sD0,    I2S0I_DATA_IN8_IDX, false);
  gpio_matrix_in(s_i2sD1,    I2S0I_DATA_IN9_IDX, false);
  gpio_matrix_in(s_i2sD2,    I2S0I_DATA_IN10_IDX, false);
  gpio_matrix_in(s_i2sD3,    I2S0I_DATA_IN11_IDX, false);
  gpio_matrix_in(s_i2sD0,    I2S0I_DATA_IN12_IDX, false);
  gpio_matrix_in(s_i2sD1,    I2S0I_DATA_IN13_IDX, false);
  gpio_matrix_in(s_i2sD2,    I2S0I_DATA_IN14_IDX, false);
  gpio_matrix_in(s_i2sD3,    I2S0I_DATA_IN15_IDX, false);

  gpio_matrix_in(s_i2sVsync, I2S0I_V_SYNC_IDX, true);
  gpio_matrix_in(s_i2sHsync, I2S0I_H_SYNC_IDX, true);
  gpio_matrix_in(s_constantHighInput,    I2S0I_H_ENABLE_IDX, false);

  periph_module_enable(PERIPH_I2S0_MODULE);

  I2SConfReset();

  I2S0.conf2.lcd_en = 1;
  I2S0.conf2.camera_en = 1;

  //32MHz sampling. Anything higher and the pixels start to wobble
  I2S0.clkm_conf.clkm_div_a = 4;
  I2S0.clkm_conf.clkm_div_b = 1;
  I2S0.clkm_conf.clkm_div_num = 1;
  I2S0.sample_rate_conf.rx_bck_div_num =1;

  I2S0.fifo_conf.dscr_en = 1;
  I2S0.fifo_conf.rx_fifo_mod = 1;
  I2S0.fifo_conf.dscr_en = 1;
  I2S0.fifo_conf.rx_fifo_mod_force_en = 1;

  I2S0.conf.rx_right_first = 0;
  I2S0.conf.rx_msb_right = 0;
  I2S0.conf.rx_msb_shift = 0;
  I2S0.conf.rx_mono = 1;
  I2S0.conf.rx_short_sync = 1;
  I2S0.conf_chan.rx_chan_mod = 1;
  I2S0.sample_rate_conf.rx_bits_mod = 16; //Number of bits here doesn't seem relevant.
  I2S0.timing.val = 0;

  //Setup interrupt
  esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM, I2SIsr, nullptr, &s_isrHandle);

  //Setup DMA buffer
  I2S0.rx_eof_num = s_inputLineBufferSizeBytes / 4;
  I2S0.in_link.addr = (uint32_t) &s_dmaDesc[0];
  I2S0.in_link.start = 1;

  //Enable in_done interrupt
  I2S0.int_clr.val = I2S0.int_raw.val;
  I2S0.int_ena.val = 0;
  I2S0.int_ena.in_done = 1;
  ESP_INTR_ENABLE(ETS_I2S0_INUM);
  I2S0.conf.rx_start = 1;
}

//---------------------------FabGL drawing---------------------------------------------
uint8_t PixelFromRGBI(uint8_t rgbi)
{   
  auto i = (rgbi & 8) == 8 ? 1 : 0;
  auto r = (rgbi & 1) == 1 ? 2+i : i;
  auto g = (rgbi & 2) == 2 ? 2+i : i;
  auto b = (rgbi & 4) == 4 ? 2+i : i;
  return s_displayController.createRawPixel(RGB222(r, g, b));
}

void IRAM_ATTR DrawScanline(void* arg, uint8_t* dest, int scanLine)
{
  auto width = s_displayController.getScreenWidth();
  if (scanLine < 200)
  {
    auto scanAddress = scanLine*320;
    auto src = &s_memBuffer[scanAddress];
    for (auto x = 0; x < width; x += 2)
    {
      auto color = s_colorConv[*(src++)];
      *((uint16_t*)&(VGA_PIXELINROW(dest, x))) = color;
    }
  }
  if (scanLine == s_displayController.getScreenHeight() - 1)
  {
    // signal end of screen
    vTaskNotifyGiveFromISR(s_mainTaskHandle, nullptr);
  }
}

//---------------------------I2S input read & convert ---------------------------------------------

void ConvertI2SToPackedMemoryBufferTask(void* parameters)
{
  uint8_t lline = 0;
  while(1)
  {
    auto rc = xQueueReceive(s_lineQueue, &lline, 0);
    if (rc == pdFALSE)
      continue;
      
    auto index = (lline & 1);
    auto memAddress2 = lline*320;
    auto src = s_rowBuffer[index]+760;
    auto endOfBuf = src+2560;
    auto destBuffer = &s_memBuffer[memAddress2];
    while(src < endOfBuf)
    {
      *(destBuffer++) = ((*src)&0xf) | (*(src+4)&0xf0);
      src += 8;
    }
  }
}
void HandleI2SInputTask(void* parameters)
{
  //Turn off watchdog..
  rtc_wdt_protect_off(); 
  rtc_wdt_disable();
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();
  vTaskSuspendAll();

  //Setup i2s, input pins and memory buffer
  DmaDescInit();
  I2SInit();

  static const auto totalLineCount = s_c128ScreenHeight+s_linesBeforeRealScreenStarts;
  portDISABLE_INTERRUPTS();
  auto loopTime = xthal_get_ccount();

  //Current line being processed by i2s loop
  uint32_t line = 0;

  while(true)
  {
    //Wait for vsync
    while(((REG_READ(GPIO_IN_REG)) & s_vSyncPin26) == 0);
    line = 0;

    //Start iterating and sampling each line
    for (int q=0; q != totalLineCount; ++q)
    {
      s_isrCount = 0;
      I2S0.conf.val |= s_confResetFlags;
      I2S0.conf.val &= ~s_confResetFlags;
      I2S0.in_link.addr = ((uint32_t) &s_dmaDesc[line & 1]);
      I2S0.in_link.start = 1;
      I2S0.int_clr.val = I2S0.int_raw.val;
      portENABLE_INTERRUPTS();
      //This is running one loop behind. So it starts sending line 58 when it reaches line 59
      if(q > s_linesBeforeRealScreenStarts) 
      {
        //There's only enough time to convert 300 bytes of the i2s buffer here. 
        //Send it to the other core for conversion
        ++line;
        xQueueSendToBackFromISR(s_lineQueue, (void*)&line, 0);
      }      
      //Not required, just here to point out that there's alot of free time here.
      delayMicroseconds(52);
      //Wait for i2s buffer to be filled
      while(s_isrCount == 0); 
      portDISABLE_INTERRUPTS();   
      //Check if some glitch caused the buffer fill to take longer than usual and compensate for it.
      auto t = xthal_get_ccount();
      int diff = (t-loopTime)/s_clockSpeedMHz;
      loopTime = t;
      delayMicroseconds(diff > 65 ? std::max(67 - diff, 0) : 3);
    }
  }
}

void setup()
{
  //Sometimes a glitch happens every 5-10 seconds. One of these seems to remove it.
  WiFi.mode(WIFI_OFF);
  btStop();
  esp_wifi_deinit();
  esp_bt_controller_deinit();

  //Setup FabGL display
  s_mainTaskHandle = xTaskGetCurrentTaskHandle();
  s_displayController.begin(GPIO_NUM_22, GPIO_NUM_21, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_4, GPIO_NUM_23, GPIO_NUM_15);
  s_displayController.setDrawScanlineCallback(DrawScanline);
  s_displayController.setResolution(VGA_640x200_70Hz);

  //Just setup up a simple pattern in the pixelbuffer
  memset((void*)s_memBuffer, 0x14, 320*200);
  for(auto i=0;i !=256;++i)
  {
    s_colorConv[i] = PixelFromRGBI(i) | (PixelFromRGBI(i>>4) << 8);
  }

  s_lineQueue = xQueueCreate(10, sizeof(uint8_t));
  xTaskCreatePinnedToCore(ConvertI2SToPackedMemoryBufferTask,
                          "I2SBuf2MemBuf",
                          2048, nullptr, tskIDLE_PRIORITY+2, nullptr, 0);

  xTaskCreatePinnedToCore(HandleI2SInputTask,
                          "I2SInputHandler",
                          2048, nullptr, tskIDLE_PRIORITY+2, nullptr, 1);


}

void loop()
{  
}
