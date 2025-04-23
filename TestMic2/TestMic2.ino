#include <Arduino.h>
#include <driver/i2s_std.h>

//
// INMP441
//
#define I2S_MIC_SERIAL_CLOCK     GPIO_NUM_33
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_26
#define I2S_MIC_SERIAL_DATA      GPIO_NUM_25

#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 8000

i2s_chan_handle_t m_i2s_rx_handle;

int32_t raw_samples[SAMPLE_BUFFER_SIZE];

void setup() {
  // we need serial output for the monitor.
  Serial.begin(115200);
  
   // -------- I2S configuration -------------------------------------------------------------------------------------------
  //i2s_chan_config_t m_i2s_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_chan_config_t m_i2s_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
 
  /* Allocate a new RX channel and get the handle of this channel */
  ESP_ERROR_CHECK(i2s_new_channel(&m_i2s_chan_cfg, NULL, &m_i2s_rx_handle));

  i2s_std_config_t m_i2s_std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    //.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    //.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    //.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_MONO),
    //.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .slot_cfg = {
        .data_bit_width = I2S_DATA_BIT_WIDTH_24BIT,
        .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
        .slot_mode = I2S_SLOT_MODE_MONO,
        .slot_mask = I2S_STD_SLOT_LEFT,
        .ws_width = I2S_SLOT_BIT_WIDTH_32BIT,
        .ws_pol = false,
        .bit_shift = true,  // o false, no lo tengo claro
        .msb_right = false,
    },
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_MIC_SERIAL_CLOCK,
      .ws = I2S_MIC_LEFT_RIGHT_CLOCK,
      .dout = I2S_GPIO_UNUSED,
      .din = I2S_MIC_SERIAL_DATA,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  //m_i2s_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  ///* The 'mclk_multiple' should be the multiple of 3 while using 24-bit data width */
  //m_i2s_std_cfg.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_384;  
  
  /* Initialize the channel */
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(m_i2s_rx_handle, &m_i2s_std_cfg));

  /* Before reading data, start the RX channel first */
  ESP_ERROR_CHECK(i2s_channel_enable(m_i2s_rx_handle));

}

void loop() {
  // put your main code here, to run repeatedly:
  size_t bytes_read = 0;
  i2s_channel_read(m_i2s_rx_handle, &raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, 1000);
  int samples_read = bytes_read / sizeof(int32_t);
  // dump the samples out to the serial channel.
  for (int i = 0; i < samples_read; i++)
  {
    Serial.printf("%ld\n", raw_samples[i]);
  }
}
