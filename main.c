/* Authors: Matthew Bailey, Patrick Gipson
 *
 * Design a 4 DIP switch controlled fixed point equalizer:
 * 		- SW0 controls bypass signal
 * 		- SW1 controls LPF
 * 		- SW2 controls BPF
 * 		- SW3 controls HPF
 *10/17/19
 */



//-------------------------------------------------------------------
#include "dsk6713.h"
#include "dsk6713_dip.h"
#include "dsk6713_led.h"
#include "dsk6713_aic23.h"  	  						//codec-dsk support file
//#include "dsk6713_aic23_setfreq.h"
#include "csl_edma.h"
#include "app_codec.h"
#include "shifted_filter_coefficients.h"

//-------------------------------------------------------------------
//-------------------------------------------------------------------

//-------------------------------------------------------------------
// need to have 128 elements in our buffer with larger sampling
#define BUFLEN 0x80				// if you change this number, you have to go to the EDMA parameters
								// and change the number of reloads, the skip forward, and the skip backward
#define BUFWIDTH 2 // left and right
#define LEFT  1                  //data structure for union of 32-bit data
#define RIGHT 0                  //into two 16-bit data

#define SAMPLE_SIZE 81 		// filter sample size


#pragma DATA_SECTION (rx_ping, ".buffers");
#pragma DATA_ALIGN   (rx_ping, 0x80);
short rx_ping[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (rx_pong, ".buffers");
#pragma DATA_ALIGN   (rx_pong, 0x80);
short rx_pong[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (rx_pang, ".buffers");
#pragma DATA_ALIGN   (rx_pang, 0x80);
short rx_pang[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (tx_ping, ".buffers");
#pragma DATA_ALIGN   (tx_ping, 0x80);
short tx_ping[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (tx_pong, ".buffers");
#pragma DATA_ALIGN   (tx_pong, 0x80);
short tx_pong[BUFWIDTH][BUFLEN];

#pragma DATA_SECTION (tx_pang, ".buffers");
#pragma DATA_ALIGN   (tx_pang, 0x80);
short tx_pang[BUFWIDTH][BUFLEN];

//-------------------------------------------------------------------
typedef struct _Globals
{
	volatile int move_ping;
	volatile int move_pong;
	volatile int move_pang;
} Globals;

Globals Glo = {FALSE, FALSE, FALSE};

//-------------------------------------------------------------------
// declare the CSL objects
EDMA_Handle hEdma_rx;     // Handle for the EDMA channel
EDMA_Handle hEdmaPing_rx; // Handle for the ping EDMA reload parameters
EDMA_Handle hEdmaPong_rx; // Handle for the pong EDMA reload parameters
EDMA_Handle hEdmaPang_rx; // Handle for the pong EDMA reload parameters
EDMA_Config cfgEdma_rx;   //EDMA configuration structure

EDMA_Handle hEdma_tx;
EDMA_Handle hEdmaPing_tx;
EDMA_Handle hEdmaPong_tx;
EDMA_Handle hEdmaPang_tx;
EDMA_Config cfgEdma_tx;

#include "app_edma.h"      // structures are filled in the header


//-------------------------------------------------------------------
void main(void);
void forever_loop(void);
void output_sample(int out_data);
void init(void);

void Display_Switches(volatile char Switches[4]);
// added switch array to keep track of filtering decision
short filtering(char sample_size, int cur_iteration, char L_R, volatile char curSwitches[4]);

//-------------------------------------------------------------------

//volatile char updateSample = 0;
Int32 sample_size = 0;
volatile Int32 output = 0;

//-------------------------------------------------------------------
extern far void vectors();         //external function
//-------------------------------------------------------------------


//-------------------------------------------------------------------

//-------------------------------------------------------------------
void main(void)
{
	init();
	forever_loop();
} // void main(void)
//-------------------------------------------------------------------


//-------------------------------------------------------------------
void forever_loop(void)
{
int i, j, k;

// keep track of switches
volatile char curSwitches[4];
//char prevSwitches[4];

int index;
long sum;
int a;
	for(;;) // ever
	  {


	  	if(Glo.move_ping == TRUE) // tx from tx pang
	  	  {
	  		// check state of push buttons first
	  		for(k = 0; k < 4; k++){
	  			int temp = DSK6713_DIP_get(k);
	  			curSwitches[k] = ~(0xFE | temp);
	  		}

	  		Display_Switches(curSwitches); // update the LEDs to reflect switches

	  		Glo.move_ping = FALSE;
	  	  	for(i=0;i<BUFLEN;i++) // for entire buffer collumn length
	  	  	  {
	  	  		if(curSwitches[0]){ // bypass
	  	  			tx_pang[RIGHT][i] = rx_ping[RIGHT][i]; // [0][col]
	  	  			tx_pang[LEFT][i] = rx_ping[LEFT][i]; // [1][col]
	  	  		} else { // filter
	  	  			for(k = 0; k < 2; k++){ // LEFT AND RIGHT
	  	  				sum = 0;
	  	  				for(j = 0; j < SAMPLE_SIZE; j++){
	  	  					index = i - j;

	  	  					if(index >= 0){ // in current buffer
	  	  						sum =	(curSwitches[1])? _lsadd(sum,_smpy(rx_ping[k][index],LPF[j])) :
	  	  								(curSwitches[2])? _lsadd(sum,_smpy(rx_ping[k][index],BPF[j])) :
	  	  								(curSwitches[3])? _lsadd(sum,_smpy(rx_ping[k][index],HPF[j])) : 0;
	  	  					} else {
	  	  						sum =	(curSwitches[1])? _lsadd(sum,_smpy(rx_pang[k][BUFLEN + index],LPF[j])) :
	  	  						  	  	(curSwitches[2])? _lsadd(sum,_smpy(rx_pang[k][BUFLEN + index],BPF[j])) :
	  	  						  	  	(curSwitches[3])? _lsadd(sum,_smpy(rx_pang[k][BUFLEN + index],HPF[j])) : 0;
	  	  					}
	  	  				}
	  	  				if(sum >= 0){
	  						_lsadd(sum,16384); // add 2^14 for DP
	  					} else {
	  						_lsadd(sum,(-16384)); // subtract 2^14 for negative DP
	  					}
	  	  				sum = sum>>15; // downshift 15 bits that filters were scaled by
	  	  				tx_pang[k][i] = sum;
	  	  			}

	  	  		}
	  	  	  }
	  	  }
	  	if(Glo.move_pong == TRUE) // tx from tx ping
	  	  {
	  		Glo.move_pong = FALSE;
	  		for(i=0;i<BUFLEN;i++) // for entire buffer collumn length
	  		 {
	  			if(curSwitches[0]){ // bypass
	  				tx_ping[RIGHT][i] = rx_pong[RIGHT][i]; // [0][col]
	  				tx_ping[LEFT][i] = rx_pong[LEFT][i]; // [1][col]
	  			} else { // filter
	  				for(k = 0; k < 2; k++){ // LEFT AND RIGHT
	  					sum = 0;
	  					for(j = 0; j < SAMPLE_SIZE; j++){
	  						index = i - j;
	  						if(index >= 0){ // in current buffer
	  							sum =	(curSwitches[1])? _lsadd(sum,_smpy(rx_pong[k][index],LPF[j])) :
	  								  	(curSwitches[2])? _lsadd(sum,_smpy(rx_pong[k][index],BPF[j])) :
	  								  	(curSwitches[3])? _lsadd(sum,_smpy(rx_pong[k][index],HPF[j])) : 0;
	  						} else {
	  							sum =	(curSwitches[1])? _lsadd(sum,_smpy(rx_ping[k][BUFLEN + index],LPF[j])) :
	  							  	  	(curSwitches[2])? _lsadd(sum,_smpy(rx_ping[k][BUFLEN + index],BPF[j])) :
	  							  	  	(curSwitches[3])? _lsadd(sum,_smpy(rx_ping[k][BUFLEN + index],HPF[j])) : 0;
	  			  	  		}
	  			  	  	}
	  					if(sum >= 0){
	  						_lsadd(sum,16384); // add 2^14 for DP
	  					} else {
	  						_lsadd(sum,(-16384)); // subtract 2^14 for negative DP
	  					}
	  					sum = sum>>15; // downshift 15 bits that filters were scaled by
	  					tx_ping[k][i] = sum;
	  				}

	  			}
	  		}
	  	  }
	  	if(Glo.move_pang == TRUE) // tx from tx pong
	  	  {
	  		Glo.move_pang = FALSE;
	  		for(i=0;i<BUFLEN;i++) // for entire buffer collumn length
	  		 {
	  			if(curSwitches[0]){ // bypass
	  				tx_pong[RIGHT][i] = rx_pang[RIGHT][i]; // [0][col]
	  				tx_pong[LEFT][i] = rx_pang[LEFT][i]; // [1][col]
	  			} else { // filter
	  				for(k = 0; k < 2; k++){ // LEFT AND RIGHT
	  					sum = 0;
	  					for(j = 0; j < SAMPLE_SIZE; j++){
	  						index = i - j;
	  						if(index >= 0){ // in current buffer
	  							sum =	(curSwitches[1])? _lsadd(sum,_smpy(rx_pang[k][index],LPF[j])) :
	  								  	(curSwitches[2])? _lsadd(sum,_smpy(rx_pang[k][index],BPF[j])) :
	  								  	(curSwitches[3])? _lsadd(sum,_smpy(rx_pang[k][index],HPF[j])) : 0;
	  						} else {
	  							sum =	(curSwitches[1])? _lsadd(sum,_smpy(rx_pong[k][BUFLEN + index],LPF[j])) :
	  								  	(curSwitches[2])? _lsadd(sum,_smpy(rx_pong[k][BUFLEN + index],BPF[j])) :
	  								  	(curSwitches[3])? _lsadd(sum,_smpy(rx_pong[k][BUFLEN + index],HPF[j])) : 0;
	  			  			}
	  			   	  	}
	  					if(sum >= 0){
	  						_lsadd(sum,16384); // add 2^14 for DP
	  					} else {
	  						_lsadd(sum,(-16384)); // subtract 2^14 for negative DP
	  					}
	  					sum = sum>>15; // downshift 15 bits that filters were scaled by
	  			 		tx_pong[k][i] = sum;
	  				}

	  			  }
	  		}
	  	  }
	  } // ever


} // void forever_loop(void)
//-------------------------------------------------------------------

//
void Display_Switches(volatile char Switches[4]){
	int i;
	for(i = 0; i < 4; i++){ // 4 LSb's represent the 4 switches being pressed
		if(Switches[i]){
			DSK6713_LED_on(i); // if the bit is true, turn the LED on
		} else {
 			DSK6713_LED_off(i); // if the bit is false, turn the LED off
		}
	}
}
// filtering

//-------------------------------------------------------------------
void output_sample(int out_data)    //for out to Left and Right channels
{

	AIC_data.uint=out_data;          //32-bit data -->data structure

    MCBSP_write(DSK6713_AIC23_DATAHANDLE, AIC_data.uint);//write/output data

} // void output_sample(int out_data)
//-------------------------------------------------------------------

// cur_iteration = iteration + 32 * buffer_number (0,1,2)

//-------------------------------------------------------------------
interrupt void c_int8()            // interrupt service routine
{
	if(EDMA_intTest(0xF))          // rx_ping is full
	{
		EDMA_intClear(0xF);
		Glo.move_ping = TRUE;
	}

	if(EDMA_intTest(0xE))          // tx_ping has been emptied
	{
		EDMA_intClear(0xE);
	}

	if(EDMA_intTest(0xD))          // rx_pong is full
	{
		EDMA_intClear(0xD);
		Glo.move_pong = TRUE;
	}

	if(EDMA_intTest(0xC))          // tx_pong has been emptied
	{
		EDMA_intClear(0xC);
	}

	if(EDMA_intTest(0xB))          // rx_pang is full
	{
		EDMA_intClear(0xB);
		Glo.move_pang = TRUE;
	}

	if(EDMA_intTest(0xA))          // tx_pang has been emptied
	{
		EDMA_intClear(0xA);
	}
	return;

} // c_int8
//-------------------------------------------------------------------


//-------------------------------------------------------------------
void init(void)	     	 	//for communication/init using interrupt
{
	int i;
	for(i=0; i<BUFLEN; i++)
	  {
	  	rx_ping[RIGHT][i] = 0;
	  	rx_ping[LEFT][i] = 0xdddd;
	  	rx_pong[RIGHT][i] = 0;
	  	rx_pong[LEFT][i] = 0xffff;
	  	rx_pang[RIGHT][i] = 0;
	  	rx_pang[LEFT][i] = 0xffff;
	  	tx_ping[RIGHT][i] = 0;
	  	tx_ping[LEFT][i] = 0xffff;
	  	tx_pong[RIGHT][i] = 0;
	  	tx_pong[LEFT][i] = 0xffff;
	  	tx_pang[RIGHT][i] = 0;
	  	tx_pang[LEFT][i] = 0xffff;
	  }
	DSK6713_init();                   	//call BSL to init DSK-EMIF,PLL)
	CSL_init();
	// initialize dip switches and LED switches
	DSK6713_DIP_init(); // doesn't do anything

	DSK6713_LED_init(); // initialize LED state machine
	// end of matt's initialization
    IRQ_globalDisable();           	    //disable interrupts
   	hCodec = DSK6713_AIC23_openCodec(0, &config1);
   	DSK6713_AIC23_setFreq(hCodec, DSK6713_AIC23_FREQ_8KHZ); // set frequency to 8kHz

                                        // codec has to be open before handle is referenced
   	EDMA_clearPram(0x00000000);

    hEdma_rx = EDMA_open(EDMA_CHA_REVT1, EDMA_OPEN_RESET);
    hEdmaPing_rx = EDMA_allocTable(-1);
    hEdmaPong_rx = EDMA_allocTable(-1);
    hEdmaPang_rx = EDMA_allocTable(-1);

    cfgEdmaPing_rx.rld = EDMA_RLD_RMK(2,hEdmaPong_rx);
    cfgEdmaPong_rx.rld = EDMA_RLD_RMK(2,hEdmaPang_rx);
    cfgEdmaPang_rx.rld = EDMA_RLD_RMK(2,hEdmaPing_rx);

    cfgEdma_rx = cfgEdmaPing_rx;        // start with ping

    EDMA_config(hEdma_rx, &cfgEdma_rx);
    EDMA_config(hEdmaPing_rx, &cfgEdmaPing_rx);
    EDMA_config(hEdmaPong_rx, &cfgEdmaPong_rx);
    EDMA_config(hEdmaPang_rx, &cfgEdmaPang_rx);

    hEdma_tx = EDMA_open(EDMA_CHA_XEVT1, EDMA_OPEN_RESET);
    hEdmaPing_tx = EDMA_allocTable(-1);
    hEdmaPong_tx = EDMA_allocTable(-1);
    hEdmaPang_tx = EDMA_allocTable(-1);

    cfgEdmaPing_tx.rld = EDMA_RLD_RMK(2,hEdmaPong_tx);
    cfgEdmaPong_tx.rld = EDMA_RLD_RMK(2,hEdmaPang_tx);
    cfgEdmaPang_tx.rld = EDMA_RLD_RMK(2,hEdmaPing_tx);

    cfgEdma_tx = cfgEdmaPing_tx;        // start with ping

    EDMA_config(hEdma_tx, &cfgEdma_tx);
    EDMA_config(hEdmaPing_tx, &cfgEdmaPing_tx);
    EDMA_config(hEdmaPong_tx, &cfgEdmaPong_tx);
    EDMA_config(hEdmaPang_tx, &cfgEdmaPang_tx);

    EDMA_intDisable(0xF);               // use 4 EDMA events
    EDMA_intClear(0xF);                 // on one EDMA interrupt
    EDMA_intEnable(0xF);

    EDMA_intDisable(0xE);
    EDMA_intClear(0xE);
    EDMA_intEnable(0xE);

    EDMA_intDisable(0xD);
    EDMA_intClear(0xD);
    EDMA_intEnable(0xD);

    EDMA_intDisable(0xC);
    EDMA_intClear(0xC);
    EDMA_intEnable(0xC);

    EDMA_intDisable(0xB);
    EDMA_intClear(0xB);
    EDMA_intEnable(0xB);

    EDMA_intDisable(0xA);
    EDMA_intClear(0xA);
    EDMA_intEnable(0xA);

    EDMA_enableChannel(hEdma_rx);       // start on the copy of ping

    EDMA_enableChannel(hEdma_tx);       // start on the copy of ping


  	IRQ_setVecs(vectors);     			//point to the IRQ vector table
    IRQ_map(IRQ_EVT_EDMAINT, 8);
    IRQ_reset(IRQ_EVT_EDMAINT);
    IRQ_enable(IRQ_EVT_EDMAINT);        // c_int8


    IRQ_globalEnable();       			//globally enable interrupts
  	IRQ_nmiEnable();          			//enable NMI interrupt


  	output_sample(0);                   // start TX to launch EDMA
} // init
//-------------------------------------------------------------------

// EOF

