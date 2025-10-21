/***********************************************************************
*
*  FILE        : sample02.c
*  DATE        : 2025-04-11
*  DESCRIPTION : Main Program
*
*  NOTE:THIS IS A TYPICAL EXAMPLE.
*
***********************************************************************/
#include "platform.h"
#include "r_smc_entry.h"
#include "r_cmt_rx_if.h"
#include "r_rspi_rx_if.h"
#include "r_rspi_rx_pinset.h"
#include "sample02.h"
#include <stdio.h>

#define DEMO_DATA_SIZE        (128)
#define RSPI_CFG_USE_GPIO_SSL (1)

#if RSPI_CFG_USE_GPIO_SSL ==  (1)
#define RSPI_CFG_SS_GPIO_PODR (PORTA.PODR.BIT.B4)
#define PORT_LOW              (0)
#define PORT_HIGH             (1)
#endif

void main(void);
static void my_cmt_callback(void *pdata);
static void my_rspi_callback(void *p_data);

static void rspi_rx65n_init_ports(void);
static void init_demo_data(void);
static void tst_trap(void);
static void spi_init_oled(void);
static void spi_init_data(uint16_t col);
static void spi_write(uint16_t length);
void wait_loop(uint32_t loop);
void delay(uint32_t ms);

#if RSPI_CFG_USE_GPIO_SSL == (1)
void set_port(uint8_t level);
#endif

static volatile bool transfer_busy = false;
static bool verify_data(uint8_t * p_data_source, uint8_t * p_data_dest, uint16_t length);
static uint8_t demo_data_source[DEMO_DATA_SIZE] = {0};
static uint8_t demo_data_dest[DEMO_DATA_SIZE]   = {0};
static uint8_t spi_data[DEMO_DATA_SIZE] = {0};

static rspi_command_word_t  my_rspi_command;
static rspi_err_t           my_rspi_err;
static rspi_evt_t           my_rspi_evt;
static rspi_handle_t        my_rspi_handle;
static rspi_chnl_settings_t my_rspi_setting;
static uint32_t             my_version;
static uint8_t              version_str[12] = {0};

static void initialize(void);
static unsigned char read_sw(void);
static unsigned char remainder(uint8_t data);
static void write_led(unsigned char data);

unsigned char state     = 0x00;
unsigned char sw        = 0x00;
unsigned char sw_prev   = 0x00;
uint8_t       cnt_us    = 0x00;
uint8_t       cnt_ms    = 0x00;
uint16_t      cnt_col   = 0x00;
unsigned char data_led  = 0x00;
unsigned char flag_copy = 0x00;

void main(void)
{
	bool     cmt_result;
	uint32_t cmt_handle;

	my_rspi_command.cpha          = RSPI_SPCMD_CPHA_SAMPLE_EVEN;
	my_rspi_command.cpol          = RSPI_SPCMD_CPOL_IDLE_HI;
	my_rspi_command.br_div        = RSPI_SPCMD_BR_DIV_1;
	my_rspi_command.ssl_assert    = RSPI_SPCMD_ASSERT_SSL0;
	my_rspi_command.ssl_negate    = RSPI_SPCMD_SSL_KEEP;
	my_rspi_command.bit_length    = RSPI_SPCMD_BIT_LENGTH_8;
	my_rspi_command.bit_order     = RSPI_SPCMD_ORDER_MSB_FIRST;
	my_rspi_command.next_delay    = RSPI_SPCMD_NEXT_DLY_SSLND;
	my_rspi_command.ssl_neg_delay = RSPI_SPCMD_SSL_NEG_DLY_SSLND;
	my_rspi_command.clock_delay   = RSPI_SPCMD_CLK_DLY_SPCKD;
	my_rspi_command.dummy         = RSPI_SPCMD_DUMMY;

	my_version = R_RSPI_GetVersion();
	sprintf((char *)version_str, "RSPI v%1.0hu.%2.2hu\r\n", ((my_version >> 16) & 0xF), (my_version & 0xFF));
	printf((char *)version_str);

	init_demo_data();

	#if RSPI_CFG_USE_GPIO_SSL == (1)
	set_port(PORT_HIGH);
	#endif

	my_rspi_setting.bps_target = 250000;
	my_rspi_setting.master_slave_mode = RSPI_MS_MODE_MASTER;

	#if RSPI_CFG_USE_GPIO_SSL == (0)
	my_rspi_setting.gpio_ssl = RSPI_IF_MODE_4WIRE;
	#else
	my_rspi_setting.gpio_ssl = RSPI_IF_MODE_3WIRE;
	#endif

	my_rspi_err = R_RSPI_Open(0, &my_rspi_setting, my_rspi_command, &my_rspi_callback, &my_rspi_handle);
	if(RSPI_SUCCESS != my_rspi_err)
	{
		printf("Create Failed.%d\n", my_rspi_err);
		tst_trap();
	}
	else
	{
		printf("Create Success!\n");
	}

	cmt_result = R_CMT_CreatePeriodic(48, my_cmt_callback, &cmt_handle);
	if(true != cmt_result)
	{
		R_BSP_NOP();
	}

	rspi_rx65n_init_ports();
	initialize();

	#if RSPI_CFG_USE_GPIO_SSL == (1)
	set_port(PORT_LOW);
	#endif

	//Bring Data/Command control logic low.
	PORT1.PODR.BIT.B5 = 0x00;
	//Bring the Reset pin logic high.
	PORT1.PODR.BIT.B7 = 0x01;
	//Bring the Vcc Enable logic low.
	PORTC.PODR.BIT.B3 = 0x00;
	//Bring Pmod Enable to logic high and delay 20ms to allow the 3.3V rail to become stable.
	PORTC.PODR.BIT.B2 = 0x01;
	delay(20);
	//Bring RES logic low, wait for at least 3us, and then bring it back to logic high to reset the display controller.
	PORT1.PODR.BIT.B7 = 0x00;
	delay(1);
	PORT1.PODR.BIT.B7 = 0x01;
	delay(1);

	spi_init_oled();
	//Draw Rectangle
	/*
	spi_data[0]  = 0x22;
	spi_data[1]  = 0x00;
	spi_data[2]  = 0x00;
	spi_data[3]  = 0x5F;
	spi_data[4]  = 0x3F;
	spi_data[5]  = 0xFF;
	spi_data[6]  = 0xFF;
	spi_data[7]  = 0xFF;
	spi_data[8]  = 0x00;
	spi_data[9]  = 0x00;
	spi_data[10] = 0x00;
	spi_write(11);
	*/
	//Set Column Address
	spi_data[0]  = 0x15;
	spi_data[1]  = 0x5F;
	spi_data[2]  = 0x5F;
	spi_write(3);
	//Turn the display on.
	PORTC.PODR.BIT.B3 = 0x01;
	delay(25);
	spi_data[0] = 0xAF;
	spi_write(1);

	for(;;)
	{
		if(flag_copy == 0x01)
		{
			flag_copy = 0x00;
			sw = read_sw();
			data_led = remainder(cnt_ms);
			write_led(data_led);

			if(((sw & 0x01) == 0x00) && ((sw_prev & 0x01) == 0x01))
			{
				state ^= 0x01;
			}

			if(1)
			{
				PORT1.PODR.BIT.B5 = 0x00;
				//Copy
				spi_data[0] = 0x23;
				spi_data[1] = 0x01;
				spi_data[2] = 0x00;
				spi_data[3] = 0x5F;
				spi_data[4] = 0x3F;
				spi_data[5] = 0x00;
				spi_data[6] = 0x00;
				spi_write(7);

				PORT1.PODR.BIT.B5 = 0x01;
				spi_init_data(cnt_col);
				spi_write(128);

				cnt_col++;
				if(cnt_col == 384)
				{
					cnt_col = 0;
				}

				cnt_us++;
				if(cnt_us == 0x0C)
				{
					cnt_us = 0x00;
					cnt_ms++;
				}
			}

			sw_prev = sw;
		}
	}

	verify_data(demo_data_source, demo_data_dest, 1);
	R_RSPI_Close(my_rspi_handle);

	#if RSPI_CFG_USE_GPIO_SSL == (1)
	set_port(PORT_HIGH);
	#endif
}

static void my_cmt_callback(void *pdata)
{
	uint32_t channel_num;
	channel_num = *((uint32_t *)pdata);

	switch(channel_num)
	{
	case 0:
		flag_copy = 0x01;
		break;
	case 1:
		break;
	default:
		break;
	}
}

static void my_rspi_callback(void *p_data)
{
	my_rspi_evt = (*(rspi_callback_data_t *)p_data).event_code;
	transfer_busy = false;
}

static void rspi_rx65n_init_ports(void)
{
	R_RSPI_PinSet_RSPI0();
	return;
}

static void init_demo_data(void)
{
	uint16_t i;
	for(i = 0; i < DEMO_DATA_SIZE; i++)
	{
		demo_data_source[i] = 0x00;
		demo_data_dest[i]   = 0x00;
		spi_data[i]         = 0x00;
	}

	/*
	uint16_t j;
	uint16_t k;
	for(j = 0; j < 384; j++)
	{
		for(k = 0; k < 64; k++)
		{
			if(j < 96)
			{
				draw_data[j][k][0] = 33 + (j * 2);
				draw_data[j][k][1] = k * 4;
				draw_data[j][k][2] = 33;
			}
			else if(j < 192)
			{
				draw_data[j][k][0] = 223;
				draw_data[j][k][1] = k * 4;
				draw_data[j][k][2] = (j * 2) - 159;
			}
			else if(j < 288)
			{
				draw_data[j][k][0] = 607 - (j * 2);
				draw_data[j][k][1] = k * 4;
				draw_data[j][k][2] = 223;
			}
			else
			{
				draw_data[j][k][0] = 33;
				draw_data[j][k][1] = k * 4;
				draw_data[j][k][2] = 799 - (j * 2);
			}
		}
	}
	*/
}

static void tst_trap(void)
{
	while(1)
	{
		/*Do nothing*/
	}
}

static void spi_init_oled(void)
{
	//Enable the driver IC to accept commands by sending the unlock command.
	spi_data[0] = 0xFD;
	spi_data[1] = 0x12;
	spi_write(2);
	//Send the display off command.
	spi_data[0] = 0xAE;
	spi_write(1);
	//Set the Remap and Display formats.
	spi_data[0] = 0xA0;
	spi_data[1] = 0x72;
	spi_write(2);
	//Set the Display start Line to the top line.
	spi_data[0] = 0xA1;
	spi_data[1] = 0x00;
	spi_write(2);
	//Set the Display Offset to no vertical offset.
	spi_data[0] = 0xA2;
	spi_data[1] = 0x00;
	spi_write(2);
	//Make it a normal display with no color inversion or forcing the pixels on/off.
	spi_data[0] = 0xA4;
	spi_write(1);
	//Set the Multiplex Ratio to enable all of the common pins calculated by 1+register value.
	spi_data[0] = 0xA8;
	spi_data[1] = 0x3F;
	spi_write(2);
	//Set Master Configuration to use a required external Vcc supply.
	spi_data[0] = 0xAD;
	spi_data[1] = 0x8E;
	spi_write(2);
	//Disable Power Saving Mode.
	spi_data[0] = 0xB0;
	spi_data[1] = 0x0B;
	spi_write(2);
	//Set the Phase Length of the charge and discharge rates of an OLED pixel in units of the display clock.
	spi_data[0] = 0xB1;
	spi_data[1] = 0x31;
	spi_write(2);
	//Set the Display Clock Divide Ratio and Oscillator Frequency, setting the clock divider ratio to 1 and the internal oscillator frequency to ~890kHz.
	//See Figure 28 on page 46.
	spi_data[0] = 0xB3;
	spi_data[1] = 0xF0;
	spi_write(2);
	//Set the Second Pre-Charge Speed of Color A to drive the color(red by default) to a target driving voltage.
	spi_data[0] = 0x8A;
	spi_data[1] = 0x64;
	spi_write(2);
	//Set the Second Pre-Charge Speed of Color B to drive the color(green by default) to a target driving voltage.
	spi_data[0] = 0x8B;
	spi_data[1] = 0x78;
	spi_write(2);
	//Set the Second Pre-Charge Speed of Color C to drive the color(blue by default) to a target driving voltage.
	spi_data[0] = 0x8C;
	spi_data[1] = 0x64;
	spi_write(2);
	//Set the Pre-Charge Voltage to approximately 45% of Vcc to drive each color to a target driving voltage.
	spi_data[0] = 0xBB;
	spi_data[1] = 0x3A;
	spi_write(2);
	//Set the VCOMH Deselect Level, which is the minimum voltage level to be registered as logic high, to 83% of Vcc.
	spi_data[0] = 0xBE;
	spi_data[1] = 0x3E;
	spi_write(2);
	//Set Master Current Attenuation Factor to set a reference current for the segment drivers.
	spi_data[0] = 0x87;
	spi_data[1] = 0x06;
	spi_write(2);
	//Set the Contrast for Color A(default red), effectively setting the brightness level.
	spi_data[0] = 0x81;
	spi_data[1] = 0x91;
	spi_write(2);
	//Set the Contrast for Color B(default green), effectively setting the brightness level.
	spi_data[0] = 0x82;
	spi_data[1] = 0x50;
	spi_write(2);
	//Set the Contrast for Color C(default blue), effectively setting the brightness level.
	spi_data[0] = 0x83;
	spi_data[1] = 0x7D;
	spi_write(2);
	//Disable Scrolling.
	spi_data[0] = 0x2E;
	spi_write(1);
	//Clear the screen by sending the clear command and the dimensions of the window to clear(top column, top row, bottom column, bottom row).
	spi_data[0] = 0x25;
	spi_data[1] = 0x00;
	spi_data[2] = 0x00;
	spi_data[3] = 0x5F;
	spi_data[4] = 0x3F;
	spi_write(5);
	//Enable Fill for Draw Rectangle.
	spi_data[0] = 0x26;
	spi_data[1] = 0x01;
	spi_write(2);
}

static void spi_init_data(uint16_t col)
{
	uint16_t i;
	for(i = 0; i < DEMO_DATA_SIZE; i++)
	{
		uint8_t temp_a = 0;
		uint8_t temp_b = 0;
		uint8_t temp_c = 0;

		temp_a = draw_data[col][i/2][0] / 8;
		temp_b = draw_data[col][i/2][1] / 4;
		temp_c = draw_data[col][i/2][2] / 8;

		if((i % 2) == 0)
		{
			spi_data[i] = (temp_a * 8) + (temp_b / 8);
		}
		else
		{
			spi_data[i] = ((temp_b % 8) * 32) + temp_c;
		}
	}
}

static void spi_write(uint16_t length)
{
	wait_loop(100);
	transfer_busy = true;

	my_rspi_err = R_RSPI_Write(my_rspi_handle, my_rspi_command, spi_data, length);

	if(RSPI_SUCCESS != my_rspi_err)
	{
		printf("Write Failed.%d\n", my_rspi_err);
		tst_trap();
	}

	while(transfer_busy)
	{
		R_BSP_NOP();
	}

	if(RSPI_EVT_TRANSFER_COMPLETE != my_rspi_evt)
	{
		printf("RSPI_EVT_TRANSFER_ERR\n");
		tst_trap();
	}

	wait_loop(100);
	transfer_busy = true;

	printf("Transmit Success!\n");
	wait_loop(100);
}

void wait_loop(uint32_t loop)
{
	uint32_t i = 0;
	for(i = 0; i < loop; i++)
	{
		R_BSP_NOP();
	}
}

void delay(uint32_t ms)
{
	uint32_t i = 0;
	for(i = 0; i < ms; i++)
	{
		R_BSP_NOP();
	}
}

#if RSPI_CFG_USE_GPIO_SSL == (1)
void set_port(uint8_t level)
{
	RSPI_CFG_SS_GPIO_PODR = level;
	wait_loop(100000);
}
#endif

static bool verify_data(uint8_t * p_data_source, uint8_t * p_data_dest, uint16_t length)
{
	uint16_t i;
	for(i = 0; i < length; i++)
	{
		if(p_data_source[i] != p_data_dest[i])
		{
			return false;
		}
	}
	return true;
}

static void initialize(void)
{
	PORT1.PDR.BIT.B5 = 0x01;
	PORT1.PDR.BIT.B7 = 0x01;

	#if RSPI_CFG_USE_GPIO_SSL == (1)
	PORTA.PDR.BIT.B4 = 0x01;
	PORTA.PMR.BIT.B4 = 0x00;
	#endif

	PORTB.PDR.BIT.B0 = 0x00;
	PORTB.PDR.BIT.B1 = 0x00;

	PORTC.PDR.BIT.B2 = 0x01;
	PORTC.PDR.BIT.B3 = 0x01;
}

static unsigned char read_sw(void)
{
	return PORTB.PIDR.BYTE;
}

static unsigned char remainder(uint8_t data)
{
	if(data % 8 == 0)
	{
		return 0x01;
	}
	else if(data % 8 == 1)
	{
		return 0x02;
	}
	else if(data % 8 == 2)
	{
		return 0x04;
	}
	else if(data % 8 == 3)
	{
		return 0x08;
	}
	else if(data % 8 == 4)
	{
		return 0x10;
	}
	else if(data % 8 == 5)
	{
		return 0x20;
	}
	else if(data % 8 == 6)
	{
		return 0x40;
	}
	else if(data % 8 == 7)
	{
		return 0x80;
	}
	else
	{
		return 0x00;
	}
	return 0x00;
}

static void write_led(unsigned char data)
{
	unsigned char led;

	led = data ^ 0xFF;
	PORTD.PODR.BYTE = led;
}
