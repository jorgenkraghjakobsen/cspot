#include "MA12070PAudioSink.h"

#include "driver/i2c.h"
#include "driver/i2s.h"
#include "esp_err.h"

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

  
MA12070PAudioSink::MA12070PAudioSink()
{ 
     i2s_config_t i2s_config = {

        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // Only TX
        .sample_rate = 44100,
        .bits_per_sample = (i2s_bits_per_sample_t)32,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, //2-channels
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0, //Default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = true,
        .tx_desc_auto_clear = true, //Auto clear tx descriptor on underflow
        .fixed_mclk = 0,
    }; 

    i2s_pin_config_t pin_config = {
        .bck_io_num = 32,
        .ws_io_num = 25,
        .data_out_num = 33,
        .data_in_num = -1 //Not used
    };
    i2s_driver_install((i2s_port_t)0, &i2s_config, 0, NULL);
    printf("Installed i2s driver\n");
    i2s_set_pin((i2s_port_t)0, &pin_config);
    gpio_set_drive_capability((gpio_num_t)32,(gpio_drive_cap_t)1);
    gpio_set_drive_capability((gpio_num_t)25,(gpio_drive_cap_t)1);
    gpio_set_drive_capability((gpio_num_t)33,(gpio_drive_cap_t)1);

    startI2sFeed();
    printf("Set volume and format\n");
    
    //int i2c_master_port = I2C_NUM_0;
    i2c_config_t i2c_cfg;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_io_num = 18;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_io_num = 5;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = 100000;
    i2c_cfg.clk_flags = 0;

    esp_err_t res = i2c_param_config(I2C_NUM_0, &i2c_cfg);
    printf("Driver param setup : %d\n", res);
    res = i2c_driver_install(I2C_NUM_0, i2c_cfg.mode, 0, 0, 0); 
    printf("Driver installed   : %d\n", res);
    
    ma_write_byte(0x20,1,53,8);          // Set i2s left justified, set audio_proc_enable
    ma_write_byte(0x20,1,64,0x38); 
    softwareVolumeControl = false;
}

MA12070PAudioSink::~MA12070PAudioSink()
{
}
void MA12070PAudioSink::volumeChanged(uint16_t volume) {
  uint8_t vol = volume / 512;   
  uint8_t ma_vol = (128 - vol) + 0x18; 
  printf("Volume : %d %d %d \n",volume, vol , ma_vol); 
  ma_write_byte(0x20,1,0x40,ma_vol);
}

esp_err_t MA12070PAudioSink::ma_write_byte(uint8_t i2c_addr, uint8_t prot, uint16_t address,
                        uint8_t value) 
{                        
  printf("%04x %02x\n", address, value);
  esp_err_t ret = 0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
  if (prot == 2) {
    i2c_master_write_byte(cmd, (uint8_t)((address & 0xff00) >> 8), ACK_VAL);
    i2c_master_write_byte(cmd, (uint8_t)(address & 0x00ff), ACK_VAL);
  } else {
    i2c_master_write_byte(cmd, (uint8_t)address, ACK_VAL);
  }
  i2c_master_write_byte(cmd, value, ACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret == ESP_FAIL) {
    printf("ESP_I2C_WRITE ERROR : %d\n", ret);
    return ret;
  }
  return ESP_OK;
}
