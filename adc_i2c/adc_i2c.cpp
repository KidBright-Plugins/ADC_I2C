#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "kidbright32.h"
#include "adc_i2c.h"

#undef __DEBUG__

#ifdef __DEBUG__
#include "esp_log.h"
static const char *TAG = "adc_i2c";
#endif

// adc_i2c registers
#define adc_i2c_MEAS_HIGHREP         0x2400


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
   // #define ADS1015_ADDRESS                 (0x48)    // 1001 000 (ADDR = GND) //Default address Pg 11
#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
#define ADS1115_ADDRESS ADS1115_ADDRESS_ADDR_VDD

/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
   // #define ADS1015_CONVERSIONDELAY         (1)
    #define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    //#define ADS1015_REG_POINTER_MASK        (0x03)
    #define ADS1115_REG_POINTER_CONVERT     (0x00) //* Pg 18
    #define ADS1115_REG_POINTER_CONFIG      (0x01) //*
    #define ADS1115_REG_POINTER_LOWTHRESH   (0x02) //*
    #define ADS1115_REG_POINTER_HITHRESH    (0x03) //*
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1115_REG_CONFIG_OS_MASK      (0x8000)  // No effect
    #define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion Pg 18
    #define ADS1115_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
    #define ADS1115_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS1115_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default) Pg 19
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3 //0001
    #define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3 //0010
    #define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3 //0011
    #define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0 //0100
    #define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1 //0101
    #define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2 //0110
    #define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3 //0111

    #define ADS1115_REG_CONFIG_PGA_MASK     (0x0E00) //+ - .256V //1110
    #define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3 Pg 19
    #define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

    #define ADS1115_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode Pg 19
    #define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS1115_REG_CONFIG_DR_8SPS     (0x0000)  // 8 samples per second
    #define ADS1115_REG_CONFIG_DR_16SPS    (0x0020)  // 16 samples per second
    #define ADS1115_REG_CONFIG_DR_32SPS    (0x0040)  // 32 samples per second
    #define ADS1115_REG_CONFIG_DR_64SPS    (0x0060)  // 64 samples per second
    #define ADS1115_REG_CONFIG_DR_128SPS   (0x0080)  // 128 samples per second (default)
    #define ADS1115_REG_CONFIG_DR_250SPS   (0x00A0)  // 250 samples per second
    #define ADS1115_REG_CONFIG_DR_475SPS   (0x00C0)  // 475 samples per second
    #define ADS1115_REG_CONFIG_DR_860SPS   (0x00E0)  // 860 samples per second


    #define ADS1115_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default) Pg 19
    #define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS1115_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default) Pg 19
    #define ADS1115_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS1115_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default) Pg 19
    #define ADS1115_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS1115_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1115_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions Pg 19
    #define ADS1115_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS1115_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS1115_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

adc_i2c::adc_i2c(int bus_ch, int dev_addr) {
	channel = bus_ch;
	address = dev_addr;
	polling_ms = adc_i2c_POLLING_MS;
}

void adc_i2c::init(void) {
	state = s_detect;
	analog_ch = 0;
}

int adc_i2c::prop_count(void) {
	return 0;
}

bool adc_i2c::prop_name(int index, char *name) {
	// not supported
	return false;
}

bool adc_i2c::prop_unit(int index, char *unit) {
	// not supported
	return false;
}

bool adc_i2c::prop_attr(int index, char *attr) {
	// not supported
	return false;
}

bool adc_i2c::prop_read(int index, char *value) {
	// not supported
	return false;
}

bool adc_i2c::prop_write(int index, char *value) {
	// not supported
	return false;
}

void adc_i2c::process(Driver *drv) {
	I2CDev *i2c = (I2CDev *)drv;
	uint8_t data[6];

	uint16_t config = 	ADS1115_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
						ADS1115_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
						ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low (default val)
						ADS1115_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
						ADS1115_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)

	// Set PGA/voltage range
	config |= 0x0000;//m_gain;
	// Set SPS value 
	config |= 0x00E0;//m_sps;
	// Set single-ended input channel

	if(analog_ch==0)
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
	else if (analog_ch==1)
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
	else if (analog_ch==2)
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
	else if (analog_ch==3)
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;

	// Set 'start single-conversion' bit
	config |= ADS1115_REG_CONFIG_OS_SINGLE;

	switch (state) {
		case s_detect:
			// stamp polling tickcnt
			polling_tickcnt = get_tickcnt();
			// detect i2c device
			if (i2c->detect(channel, address) == ESP_OK) {
				//printf("detected\n");
				state = s_read;// change state
				//result = 8;
			}
			else {
				state = s_error;
			}
			break;

		case s_read:			
			if (is_tickcnt_elapsed(tickcnt, 10)) {
				data[0] = 0x01 ;
				data[1] =config>>8 ;// 0xc3;
				data[2] = config & 0xff ;//0x03;
				//printf("%02x\n",data[0]);
				if (i2c->write(channel, address, data, 3) == ESP_OK){
					data[0] = ADS1115_REG_POINTER_CONVERT ;
					//printf("%02x\n",data[0]);
					if (i2c->write(channel, address, data, 1) == ESP_OK){
						if (i2c->read(channel, address, NULL, 0, data, 2) == ESP_OK) {
								/*
						        printf(": %02x:%02x:%02x:%02x:%02x:%02x\n",
							         data[0], data[1] , data[2] ,
							         data[3] , data[4] , data[5] );
								printf("\n");
								*/
								uint16_t newresult = (data[0]<<8) | data[1];
								//printf("ADC I2C %d\n",newresult);
								read_value = newresult;
								initialized = true;
								// load polling tickcnt
								tickcnt = polling_tickcnt;
								// goto wait and retry with detect state
								state = s_wait;
						}
					}

				}
				else{
					state = s_error;
				}
			}

			break;
		case s_error:
			// set error flag
			error = true;
			// clear initialized flag
			initialized = false;
			// get current tickcnt
			tickcnt = get_tickcnt();
			// goto wait and retry with detect state
			state = s_wait;
			break;
		case s_wait:
			// wait polling_ms timeout
			if (is_tickcnt_elapsed(tickcnt, polling_ms)) {
				state = s_detect;
			}
			break;
	}
}

double adc_i2c::getRawValue(int a_ch) {
	analog_ch = a_ch;
	return read_value;
}

