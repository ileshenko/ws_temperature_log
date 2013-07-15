/*
 * main.c
 */
#include <msp430g2553.h>
#include <config.h>
#include <leds.h>
#include <uart.h>
#include <clock.h>
#include <timer_lib.h>
#include <string.h>

//#define DEBUG_TEMP
/* delay will be overriden 1 second, if DEBUG_TEMP is defined */
#define DELAY_MINUTES 15

#define STR_VALUE(arg)      #arg
#define STR_MACRO(num) STR_VALUE(num)
#define DEAY_STR  STR_MACRO(DELAY_MINUTES)

#ifdef DEBUG_TEMP
#define DELAY_JIFFIES 1
#else
#define DELAY_JIFFIES (DELAY_MINUTES * 60)
#endif

unsigned long jiffies;
#define CAL_ADC_15T30 (*(( unsigned int *)( 0x10DA+0x08)))
#define CAL_ADC_15T85 (*(( unsigned int *) (0x10DA+0x0A)))

/* t = (55*x + 30*T85 - 85*T30)/(T85 - T30)
 * t = (x - a)*b/1024
 * b = 55*1024/(T85 - T30)
 * a = (17*T30 - 6*T85)/11 */
static int A;
static int B;

#define BSIZE 8

unsigned int adc_buff[BSIZE];

void adc10_temp_init(void)
{
    ADC10CTL0 &= ~ENC;

	ADC10CTL0 = SREF_1 | ADC10SHT_3 | ADC10SR | MSC | REFON | ADC10ON + ADC10IE;
	ADC10CTL1 = INCH_10 | ADC10DIV_7 | /*ADC10SSEL_3 |*/ CONSEQ_2;         // Temp Sensor ADC10CLK/4 SMCLK
	ADC10DTC1 = BSIZE;
	ADC10SA = (unsigned int)adc_buff;

	B = 55*128u/(CAL_ADC_15T85 - CAL_ADC_15T30);
	A = ((17*CAL_ADC_15T30 - 6*CAL_ADC_15T85)*8 + 6)/11; /* +6 - for correct alignment (11/2) */
}

static char to_print[20];
static unsigned char log[350];
static char report_stage;
static int write_idx, rep_idx, stop_idx;
unsigned long timestamp;

unsigned int IntDegC;

static void clear_buff(void)
{
	memset(log, 0, sizeof(log));
	write_idx = 0;
}

static void report(void)
{
	unsigned long delay = (jiffies - timestamp +30)/60;

	cat_str(cat_ul(cat_str(to_print,"@ "), delay), " " STR_MACRO(DELAY_MINUTES) "\n\r");
	 uart_text(to_print);
	 stop_idx = rep_idx = write_idx;
	 report_stage = 1;
}

static void rx_cb(unsigned char sym)
{
	switch (sym)
	{
	case 'p':
		uart_text("P\n\r");
		break;
	case 'r':
		report();
		break;
	case 'c':
		clear_buff();
		break;
	default:
		break;
	}
}

void tx_ready_cb(void)
{
	if (report_stage == 0)
		return;

	if (report_stage == 1)
	{
		IntDegC = log[rep_idx];

		cat_str(cat_ul(to_print, IntDegC),"\n\r");
		uart_text(to_print);

		if (--rep_idx < 0)
			rep_idx = sizeof(log) - 1;
		if (!log[rep_idx] || rep_idx == stop_idx)
			report_stage = 2;
		return;
	}
	if (report_stage == 2)
	{
		uart_text("==\n\r");
		report_stage = 0;
	}
}

void main(void)
{
	int i;

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	report_stage = 0;

//	default_state();
	clock_init(); /* DCO, MCLK and SMCLK - 1MHz */
	timer_init();
    jiffies = 0;

	uart_init();
	uart_rx_cb(rx_cb);
	uart_tx_ready_cb(tx_ready_cb);

	leds_init();
	adc10_temp_init();

	for (;;)
	{
		timestamp = jiffies;
		led_toggle();

		ADC10CTL0 &= ~ENC;
		while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active

		ADC10SA = 0;
		ADC10DTC1 = 0;
		ADC10DTC1 = BSIZE;
		ADC10SA = (unsigned int)adc_buff;

		ADC10CTL0 |=( ENC | ADC10SC);             // Sampling and conversion start
		//	    while (!(ADC10CTL0 & ADC10IFG));               // Wait if ADC10 core is active
		_BIS_SR(LPM0_bits + GIE);
		ADC10CTL0 &= ~ENC;						// Stop conversation
		ADC10CTL0 &= ~ADC10IFG;					// Clear interrupt flag

		IntDegC = 0;
		for (i = 0; i < BSIZE; i++)
			IntDegC += adc_buff[i];

		IntDegC += BSIZE>>1;
		IntDegC = (((IntDegC - A) * B)+512) >>10;

// 		IntDegC = (temp - CAL_ADC_15T30) * (85-30)/(CAL_ADC_15T85 - CAL_ADC_15T30) + 30;
//		temp >>= 3;
//		qq = (55*temp + 30*CAL_ADC_15T85 - 85*CAL_ADC_15T30)/(CAL_ADC_15T85 - CAL_ADC_15T30);

		if (++write_idx>=sizeof(log))
			write_idx = 0;
		log[write_idx] = (char)IntDegC;

#ifdef DEBUG_TEMP
		cat_str(cat_ul(to_print, IntDegC),"\n\r");
		uart_text(to_print);
#endif

		_BIS_SR(LPM0_bits + GIE);
	}
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void main_timer(void)
{
	switch(TA0IV)
	{
	case 10: /* TAIFG, timer overflow */
		break;
	case 2: /* TACCR1 CCIFG, compare 1 */
	case 4: /* TACCR2 CCIFG, compare 2 */
	default: /* impossible! */
		for (;;);
	}

	jiffies++;
	if (jiffies - timestamp >= DELAY_JIFFIES)
		_BIC_SR_IRQ(LPM0_bits);
}

#pragma vector=ADC10_VECTOR
__interrupt void adc10_isr (void)
{
	_BIC_SR_IRQ(LPM0_bits);
}
