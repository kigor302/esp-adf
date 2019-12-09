/*
 * ac101.c
 *
 * X-Powers AC101 codec driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */
 
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "ac101.h"
#include "board_pins_config.h"
#include "headphone_detect.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_hal.h"
#include "esp_system.h"
#include "board.h"

static const char *AC101_TAG = "AC101_DRIVER";

#define IIC_PORT    I2C_NUM_1

#define AC_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(AC101_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

static i2c_config_t ac_i2c_cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = (100000)
};


audio_hal_func_t AUDIO_CODEC_AC101_DEFAULT_HANDLE = {
    .audio_codec_initialize = ac101_init,
    .audio_codec_deinitialize = ac101_deinit,
    .audio_codec_ctrl = ac101_ctrl_state,
    .audio_codec_config_iface = ac101_config_i2s,
    .audio_codec_set_mute = ac101_set_voice_mute,
    .audio_codec_set_volume = ac101_set_voice_volume,
    .audio_codec_get_volume = ac101_get_voice_volume,
    .audio_codec_set_recvolume = ac101_set_recvoice_volume,
    .audio_codec_get_recvolume = ac101_get_recvoice_volume
};


static int i2c_init()
{
    int res;
    if (get_i2c_pins(IIC_PORT, &ac_i2c_cfg))
    {
        ESP_LOGE(AC101_TAG, "get_i2c_pins has failed");
        return -1;
    }
    res = i2c_param_config(IIC_PORT, &ac_i2c_cfg);
    res |= i2c_driver_install(IIC_PORT, ac_i2c_cfg.mode, 0, 0, 0);
    AC_ASSERT(res, "i2c_init error", -1);
    return res;
}

/*static esp_err_t AC101_Write_Reg(uint8_t reg, uint16_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret =0;
	uint8_t send_buff[4];
	send_buff[0] = (AC101_ADDR << 1);
	send_buff[1] = reg;
	send_buff[2] = (val>>8) & 0xff;
	send_buff[3] = val & 0xff;
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write(cmd, send_buff, 4, ACK_CHECK_EN);
    ret |= i2c_master_stop(cmd);
    ret |= i2c_master_cmd_begin(IIC_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret)
    {
        ESP_LOGW(AC101_TAG, "++write_res end =%d", ret);
    }
    AC_ASSERT(ret, "+++ac_write_reg error", -1);
    return ret;
}
*/

static int ac_write_reg(uint8_t reg_add, uint16_t data)
{
    uint8_t tx_data[4] = { (AC101_ADDR<<1), reg_add, ((data >> 8) & 0xff), (data & 0xff) };
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    int res = i2c_master_start(cmd);
    res |= i2c_master_write(cmd, tx_data, 4, ACK_CHECK_EN);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(IIC_PORT, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    AC_ASSERT(res, "ac_write_reg error", -1);
    return res;
}


static uint16_t ac_read_reg(uint8_t reg_add)
{
    uint8_t data[2] = {0, 0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    int res = i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, (AC101_ADDR<<1), ACK_CHECK_EN);
    res |= i2c_master_write_byte(cmd, reg_add, ACK_CHECK_EN);
    res |= i2c_master_start(cmd);  // re-start condition
    res |= i2c_master_write_byte(cmd, (AC101_ADDR<<1) | READ_BIT, ACK_CHECK_EN);
    res |= i2c_master_read(cmd, data, 2, ACK_VAL);
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(IIC_PORT, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    AC_ASSERT(res, "ac_read_reg error", 0);
    return ((data[0]<<8) | data[1]);
}

static esp_err_t set_codec_clk(audio_hal_iface_samples_t sampledata)
{
	uint16_t sample;
	switch (sampledata)
	{
	case AUDIO_HAL_08K_SAMPLES:
		sample = SIMPLE_RATE_8000;
		break;
	case AUDIO_HAL_11K_SAMPLES:
		sample = SIMPLE_RATE_11052;
		break;
	case AUDIO_HAL_16K_SAMPLES:
		sample = SIMPLE_RATE_16000;
		break;
	case AUDIO_HAL_22K_SAMPLES:
		sample = SIMPLE_RATE_22050;
		break;
	case AUDIO_HAL_24K_SAMPLES:
		sample = SIMPLE_RATE_24000;
		break;
	case AUDIO_HAL_32K_SAMPLES:
		sample = SIMPLE_RATE_32000;
		break;
	case AUDIO_HAL_44K_SAMPLES:
		sample = SIMPLE_RATE_44100;
		break;
	case AUDIO_HAL_48K_SAMPLES:
		sample = SIMPLE_RATE_48000;
		break;
	case AUDIO_HAL_96K_SAMPLES:
		sample = SIMPLE_RATE_96000;
		break;
	case AUDIO_HAL_192K_SAMPLES:
		sample = SIMPLE_RATE_192000;
		break;
	default:
        ESP_LOGW(AC101_TAG, "codec_clk sampledata:%d not supported, set default 44.1khz", sampledata);
		sample = SIMPLE_RATE_44100;
        break;
	}
	return ac_write_reg(I2S_SR_CTRL, sample);
}


esp_err_t ac101_init(audio_hal_codec_config_t* codec_cfg) 
{
	if (ESP_OK != i2c_init()) 
        return -1;
  
    //headphone_detect_init(get_headphone_detect_gpio());
          
	esp_err_t res = ac_write_reg(CHIP_AUDIO_RS, 0x123);
	if (ESP_OK != res) {
		ESP_LOGE(AC101_TAG, "ac101_init - reset failed!");
		return res;
    }
  	ESP_LOGI(AC101_TAG, "ac101_init - reset succeed");
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	res |= ac_write_reg(SPKOUT_CTRL, 0xe880); /* Disable L&R Speekers */

	//Enable the PLL from 256*44.1KHz MCLK source
    /* FOUT= FIN*N/(M*(2K+1)), N=N_i+N_f; */
	res |= ac_write_reg(PLL_CTRL1, 0x014f);   /* Factor=1, M=1. Work as PLL, loop bandwidth = 15 from 63 */  
    
	//res |= ac_write_reg(PLL_CTRL2, 0x83c0);
    /* FOUT= FIN*N/(M*(2K+1)), N=N_i+N_f; */
	res |= ac_write_reg(PLL_CTRL2, 0x8600);  /* PLL enable Predivider decimal N_i=0x60(96), fractional N_f=0*0.2 */

	//Clocking system
	res |= ac_write_reg(SYSCLK_CTRL, 0x8b08);  /* PLL clock src = MCLK1, PLL=en, I2SClk=en, I2S1-src-clk=PLL, SYSCLK=en */    
	res |= ac_write_reg(MOD_CLK_ENA, 0x800c);  /* I2S1 Clk=en, HPF:AGC+DRC clk=disable, ADC+DAC clk=en */ 
	res |= ac_write_reg(MOD_RST_CTRL,0x800c);  /* I2S1 Rst=en, HPF:AGC+DRC rst=disable, ADC+DAC rst=en */
	res |= ac_write_reg(I2S_SR_CTRL, 0x7000);  /* sample rate 44.1khz */

	//AIF config
	res |= ac_write_reg(I2S1LCK_CTRL, 0x8850);	  /* Slave=on, I2S1_BCLK_DIV=1/8, BCLK/LRCK=1/32, WORD_SZ=16bit, data-fmt=I2S, stereo mode */	//BCLK/LRCK
	res |= ac_write_reg(I2S1_SDOUT_CTRL, 0xc000); /* I2S1 ADC L/R time slot 0 = en, 1 = dis, time slot0 src = ADC_L/R, a-low/u-law disabled */
	res |= ac_write_reg(I2S1_SDIN_CTRL, 0xc000);  /* I2S1 DAC L/R time slot 0 = en, ==//==, Loopback disabled */
	res |= ac_write_reg(I2S1_MXR_SRC, 0x2200);    /* I2S1 timeslot 0 ADC_L/R selected, 1=all zero */

	res |= ac_write_reg(ADC_SRCBST_CTRL, 0xccc4); /* mic1/2 amp and boost = en, gain=0db */  
	res |= ac_write_reg(ADC_SRC, 0x2020);         /* right ADC=Mic1, left=Mic2 */
	res |= ac_write_reg(ADC_DIG_CTRL, 0x8000);    /* ADC digital = en */
	res |= ac_write_reg(ADC_APC_CTRL, 0xbbc3);    /* ADCR/L channels=en, input gain = 0db, bias+chopper=en  */

	//Path Configuration
	res |= ac_write_reg(DAC_MXR_SRC, 0xcc00);     /* DAC mixer src I2S1_DA0/1_L/R selected */
	res |= ac_write_reg(DAC_DIG_CTRL, 0x8000);    /* DAC digital part = en */
	res |= ac_write_reg(OMIXER_SR, 0x0081);       /* output mux R = DACL, output mux L = DACR (channels swapped) */ 
	res |= ac_write_reg(OMIXER_DACA_CTRL, 0xf080);/* enable analog internal DAC and mixers */

    //* Enable Speaker output
	res |= ac_write_reg(SPKOUT_CTRL, 0xeabd); /* R = select mixerR, Invert, Enable. L = select mixerL, Invert, Enable. Volume = 0x1D => 0db */

    ac101_pa_power(true);
    ESP_LOGI(AC101_TAG, "ac101_init done res=%d", res);
    
	return res;
}

int ac101_get_spk_volume(void)
{
    return  (ac_read_reg(SPKOUT_CTRL) & 0x1F) * 2;
}

esp_err_t ac101_set_spk_volume(uint8_t volume)
{
    uint16_t val = ac_read_reg(SPKOUT_CTRL) & ~0x1f;  
    return  ac_write_reg(SPKOUT_CTRL, val | ((volume/2) & 0x1f));
}

int ac101_get_earph_volume(void)
{
    int res = ac_read_reg(HPOUT_CTRL);
    return (res>>4) & 0x3f;
}

esp_err_t ac101_set_earph_volume(uint8_t volume)
{
    uint16_t val = ac_read_reg(HPOUT_CTRL) & ~(0x3f<<4);  
    return  ac_write_reg(HPOUT_CTRL, val | ((volume & 0x3f) << 4));
}

esp_err_t ac101_set_output_mixer_gain(ac_output_mixer_gain_t gain, ac_output_mixer_source_t source)
{
    uint8_t shift = (source == SRC_MIC1)? 6:
                    (source == SRC_MIC2)? 3:
                     0; /* SRC_LINEIN */
            
    uint16_t val = ac_read_reg(OMIXER_BST1_CTRL) & ~(0x7 << shift);
    return ac_write_reg(OMIXER_BST1_CTRL, val | ((gain & 0x7)<<shift) );
}

esp_err_t ac101_start(ac_module_t mode)
{
	esp_err_t res = 0;
    if (mode == AC_MODULE_LINE) {
		res |= ac_write_reg(ADC_SRC /*0x51*/, 0x0408); //select ADC source LineInL and LineInR
		res |= ac_write_reg(ADC_DIG_CTRL /*0x40*/, 0x8000); //Enable ADC digital side
		res |= ac_write_reg(ADC_APC_CTRL, 0xbb00);    /* ADCR/L channels=en, input gain = 0db, bias+chopper=dis  */
    }
    else if (mode == AC_MODULE_ADC)
    {
		res |= ac_write_reg(ADC_SRCBST_CTRL, 0xccc4); /* mic1/2 amp and boost = en, gain=0db */  
		res |= ac_write_reg(ADC_SRC, 0x2020);         /* right ADC=Mic1, left=Mic2 */
		res |= ac_write_reg(ADC_DIG_CTRL, 0x8000);    /* ADC digital = en */
		res |= ac_write_reg(ADC_APC_CTRL, 0xbbc3);    /* ADCR/L channels=en, input gain = 0db, bias+chopper=en  */
    }

    if (mode == AC_MODULE_ADC || mode == AC_MODULE_ADC_DAC || mode == AC_MODULE_LINE) {
		//I2S1_SDOUT_CTRL
		//res |= ac_write_reg(PLL_CTRL2, 0x8120);
    	res |= ac_write_reg(MOD_CLK_ENA,  0x800c); /* I2S1 and ADC + DAC */ 
    	res |= ac_write_reg(MOD_RST_CTRL, 0x800c); /* I2S1 and ADC + DAC */
    }
    if (mode == AC_MODULE_DAC || mode == AC_MODULE_ADC_DAC || mode == AC_MODULE_LINE) {
    	//* Enable Headphoe output   注意使用耳机时，最后开以下寄存器
		res |= ac_write_reg(OMIXER_DACA_CTRL, 0xff80); /* Enable DAC analog L&R and output mixer L&R */
    	res |= ac_write_reg(HPOUT_CTRL, 0xc3c1); /* headphone power amplifier select MxerL&R with mute PA disabled vol=0x3c(db) */  
    	res |= ac_write_reg(HPOUT_CTRL, 0xcb00); /* headphone power amplifier select MxerL&R with mute PA enabled vol=0x30(db) */
        vTaskDelay(100 / portTICK_PERIOD_MS);
		res |= ac_write_reg(HPOUT_CTRL, 0xfbc0); /* headphone power amplifier select MxerL&R with nomute PA enabled vol=0x3c(db) */
		
    	//* Enable Speaker output
		res |= ac_write_reg(SPKOUT_CTRL, 0xeabd);  /* R = select mixerR, Invert, Enable. L = select mixerL, Invert, Enable. Volume = 0x1D */
		vTaskDelay(10 / portTICK_PERIOD_MS);
		ac101_set_voice_volume(30);
    }
    
    ESP_LOGI(AC101_TAG, "ac101_start with mode:%d res=%d", mode, res);
    return res;
}

esp_err_t ac101_stop(ac_module_t mode)
{
	return   ac_write_reg(HPOUT_CTRL, 0x01) |		//disable earphone
	         ac_write_reg(SPKOUT_CTRL, 0xe880);		//disable speaker
}

esp_err_t ac101_deinit(void)
{
	return	ac_write_reg(CHIP_AUDIO_RS, 0x123);		//soft reset
}

esp_err_t ac101_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
	int ac_mode;
	esp_err_t res = 0;

	switch (mode) {
		case AUDIO_HAL_CODEC_MODE_ENCODE:
			ac_mode  = AC_MODULE_ADC;
			break;
		case AUDIO_HAL_CODEC_MODE_LINE_IN:
			ac_mode  = AC_MODULE_LINE;
			break;
		case AUDIO_HAL_CODEC_MODE_DECODE:
			ac_mode  = AC_MODULE_DAC;
			break;
		case AUDIO_HAL_CODEC_MODE_BOTH:
			ac_mode  = AC_MODULE_ADC_DAC;
			break;
		case AUDIO_HAL_CODEC_MODE_PASSTHROUGH:
			{
			uint16_t regval_src  = (ac_read_reg(DAC_MXR_SRC) & 0xcc00) | ((ctrl_state == AUDIO_HAL_CTRL_STOP)?0:0x1100);
			uint16_t regval_gain = (ac_read_reg(DAC_MXR_SRC) & 0xcc00) | ((ctrl_state == AUDIO_HAL_CTRL_STOP)?0:0x1100);
			res |= ac_write_reg(DAC_MXR_SRC, regval_src); /* Monitor Mix line-in to output */
			res |= ac_write_reg(DAC_MXR_GAIN, regval_gain); /* Monitor Set line-in mix gain to -6 db */
			}
			return res;
		default:
			ac_mode = AC_MODULE_DAC;
			ESP_LOGW(AC101_TAG, "Codec mode not support, default is decode mode");
			break;
	}
	
    return (AUDIO_HAL_CTRL_STOP == ctrl_state)?	
        ac101_stop (ac_mode):
        ac101_start(ac_mode);                
}


esp_err_t ac101_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t* iface)
{
	uint16_t bits, fmat, regval;

	switch (iface->bits)
	{
	case AUDIO_HAL_BIT_LENGTH_16BITS:
		bits = BIT_LENGTH_16_BITS;
		break;
	case AUDIO_HAL_BIT_LENGTH_24BITS:
		bits = BIT_LENGTH_24_BITS;
		break;
	default:
        ESP_LOGW(AC101_TAG, "interface bits:%d not supported, set default", iface->bits);
		bits = BIT_LENGTH_16_BITS;
        break;
	}

	switch (iface->fmt)
	{
	case AUDIO_HAL_I2S_NORMAL:
		fmat = 0x0;
		break;
	case AUDIO_HAL_I2S_LEFT:
		fmat = 0x01;
		break;
	case AUDIO_HAL_I2S_RIGHT:
		fmat = 0x02;
		break;
	case AUDIO_HAL_I2S_DSP:
		fmat = 0x03;
		break;
	default:
        ESP_LOGW(AC101_TAG, "interface fmt:%d not supported, set default", iface->fmt);
		fmat = 0x00;
		break;
	}

	regval = ac_read_reg(I2S1LCK_CTRL) & 0xffc3;
	regval |= (iface->mode << 15) | (bits << 4) | (fmat << 2);  

	return  ac_write_reg(I2S1LCK_CTRL, regval) |
	        set_codec_clk(iface->samples);
}

esp_err_t ac101_i2s_config_clock(ac_i2s_clock_t *cfg)
{
	uint16_t regval = ac_read_reg(I2S1LCK_CTRL) & 0xe03f;

	regval |= (cfg->bclk_div << 9) & 0xf;
	regval |= (cfg->lclk_div << 6) & 0x7;
	return ac_write_reg(I2S1LCK_CTRL, regval);
}

esp_err_t ac101_set_voice_volume(int volume)
{
	return ac101_set_earph_volume(volume) |
	       ac101_set_spk_volume(volume);
}

esp_err_t ac101_get_voice_volume(int* volume)
{
	*volume = ac101_get_earph_volume();
	return 0;
}

esp_err_t ac101_set_recvoice_volume(int volume)
{
	if (volume > 100)
		volume = 100;
	if (volume < 0)
		volume = 0;

	uint16_t regval = ac_read_reg(ADC_APC_CTRL) & (~0x7700);
	uint16_t val = (((uint8_t)volume+7)/14);
	return ac_write_reg(ADC_APC_CTRL, regval | (val<<8) | (val<<12));
}

esp_err_t ac101_get_recvoice_volume(int* volume)
{
	uint16_t regval = ac_read_reg(ADC_APC_CTRL) & 0x7700;
	*volume = ((regval>>8) & 0x7) * 14;
	return 0;
}

/**
 * @brief Configure AC101 DAC mute or not. Basically you can use this function to mute the output or unmute
 *
 * @param enable: enable or disable
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
esp_err_t ac101_set_voice_mute(bool enable)
{
    return ac_write_reg(DAC_MXR_SRC, (enable? 0xcc00: 0x0000));     /* DAC mixer src I2S1_DA0/1_L/R selected */
}

esp_err_t ac101_get_voice_mute(void)
{
    return !!ac_read_reg(DAC_MXR_SRC);
}

void ac101_pa_power(bool enable)
{
    gpio_config_t  io_conf = { .pin_bit_mask = BIT(get_pa_enable_gpio()),
                               .mode = GPIO_MODE_OUTPUT,
                               .pull_up_en = 0,
                               .pull_down_en = 0,
                               .intr_type = GPIO_PIN_INTR_DISABLE
                             };
    
    gpio_config(&io_conf);
    gpio_set_level(get_pa_enable_gpio(), !!enable);
}
