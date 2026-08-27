// Pi1541 - A Commodore 1541 disk drive emulator
// Copyright(C) 2018 Stephen White
//
// This file is part of Pi1541.
// 
// Pi1541 is free software : you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Pi1541 is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Pi1541. If not, see <http://www.gnu.org/licenses/>.

#ifndef IEC_BUS_H
#define IEC_BUS_H

#include "defs.h"
#include "debug.h"
#include "m6522.h"

#include "rpi-gpio.h"
#include "rpihardware.h"


#define INPUT_BUTTON_DEBOUNCE_THRESHOLD 20000
#define INPUT_BUTTON_REPEAT_THRESHOLD 460000

/* moved to variables in InputMapping
#define INPUT_BUTTON_ENTER 0
#define INPUT_BUTTON_UP 1
#define INPUT_BUTTON_DOWN 2
#define INPUT_BUTTON_BACK 3
#define INPUT_BUTTON_INSERT 4
*/

// DIN ATN is inverted and then connected to pb7 and ca1
//	- also input to xor with ATNAout pb4
//		- output of xor is inverted and connected to DIN pin 5 DATAout (OC so can only pull low)
// VIA ATNin pb7 input to xor
// VIA ATNin ca1 output from inverted DIN 3 ATN
// DIN DATA is inverted and then connected to pb1
// VIA DATAin pb0 output from inverted DIN 5 DATA
// VIA DATAout pb1 inverted and then connected to DIN DATA
// DIN CLK is inverted and then connected to pb2
// VIA CLKin pb2 output from inverted DIN 4 CLK
// VIA CLKout pb3 inverted and then connected to DIN CLK

// $1800
// PB 0		data in
// PB 1		data out
// PB 2		clock in
// PB 3		clock out
// PB 4		ATNA
// PB 5,6	device address
// PB 7,CA1	ATN IN

// If ATN and ATNA are out of sync
//	- VIA's DATA IN will automatically set high and hence the DATA line will be pulled low (activated)
// If ATN and ATNA are in sync
//	- the output from the XOR gate will be low and the output of its inverter will go high
//	- when this occurs the DATA line must be still able to be pulled low via the PC or VIA's inverted PB1 (DATA OUT)
//
// Therefore in the same vein if PB7 is set to output it could cause the input of the XOR to be pulled low
//

// NOTE ABOUT SRQ
// SRQ is a little bit different.
// The 1581 does not pull it high. Only the 128 pulls it high.
// 
#if defined(HAS_40PINS)
enum PIGPIO
{
	// Original Non-split lines
	PIGPIO_ATN = 2,			// 3
	PIGPIO_CLOCK = 17,		// 11
	PIGPIO_DATA = 18,		// 12
	PIGPIO_SRQ = 19,		// 35
	PIGPIO_RESET = 3,		// 5


	// Pinout for those that want to split the lines (and the common ones like buttons, sound and LED)
	// 0 IDSC			//28
	// 1 IDSD			//27
	// 2 I2C_SDA		//3
	// 3 I2C_CLK		//5
	PIGPIO_IN_BUTTON4 = 4,	// 07 Common
	PIGPIO_IN_BUTTON5 = 5,	// 29 Common
	//PIGPIO_OUT_RESET = 6,	// 31 
	PIGPIO_OUT_SPI0_RS = 6,	// 31 
	// 7 SPI0_CS1		//26
	// 8 SPI0_CS0		//24
	// 9 SPI0_MISO		//21
	// 10 SPI0_MOSI		//19
	// 11 SPI0_CLK		//23
	PIGPIO_OUT_ATN = 12,	// 32
	PIGPIO_OUT_SOUND = 13,	// 33 Common
	// 14 TX			//8
	// 15 RX			//10
	PIGPIO_OUT_LED = 16,	// 36 Common
	PIGPIO_OUT_CLOCK = 17,	// 11
	PIGPIO_OUT_DATA = 18,	// 12
	PIGPIO_OUT_SRQ = 19,	// 35
	PIGPIO_IN_RESET = 20,	// 38
	PIGPIO_IN_SRQ = 21,		// 40
	PIGPIO_IN_BUTTON2 = 22,	// 15 Common
	PIGPIO_IN_BUTTON3 = 23,	// 16 Common
	PIGPIO_IN_ATN = 24,		// 18
	PIGPIO_IN_DATA = 25,	// 22
	PIGPIO_IN_CLOCK = 26,	// 37
	PIGPIO_IN_BUTTON1 = 27	// 13 Common
};
#else
//Added GPIO bindings for Raspberry Pi 1B Rev 1/2 (only 26 I/O ports)
enum PIGPIO
{
	// Original Non-split lines
	// Raspberry Pi 1B Rev 2 has GPIO0/1 in place of GPIO2/3
#if defined(RPI1BREV1)	
	PIGPIO_ATN = 0,			// 3
	PIGPIO_RESET = 1,		// 5
#else
	PIGPIO_ATN = 2,			// 3
	PIGPIO_RESET = 3,		// 5
#endif
	PIGPIO_CLOCK = 17,		// 11
	PIGPIO_DATA = 18,		// 12
	PIGPIO_SRQ = 19,		// 35  not connected yet

	// Pinout for those that want to split the lines (and the common ones like buttons, sound and LED)
	// Funktion = 	GPIO	// Hardware PIN
	// 0 IDSC				// 28
	// 1 IDSD				// 27
	// 2 I2C_SDA			// 3
	// 3 I2C_CLK			// 5
	PIGPIO_IN_BUTTON4 = 4,	// 07 Common
	//GPIO = 5,				// 29 Common
	//PIGPIO_OUT_RESET = 6,	// 31
	PIGPIO_OUT_SPI0_RS = 6,	// 31 not connected yet
	// 7 SPI0_CS1			// 26
	PIGPIO_IN_BUTTON5 =  9,	// 21
	PIGPIO_IN_RESET = 20,	// 38
	PIGPIO_IN_CLOCK = 10,	// 19
	PIGPIO_OUT_LED = 7,	    // 26
	PIGPIO_OUT_ATN = 12,	// 32 not connected yet
	PIGPIO_OUT_SOUND = 11,  // 23
	// 14 TX				// 8
	// 15 RX  				// 10
	//GPIO = 16,			// 36 Common
	PIGPIO_OUT_CLOCK = 17,	// 11
	PIGPIO_OUT_DATA = 18,	// 12
	PIGPIO_OUT_SRQ = 19,	// 35 not connected yet
	//GPIO 20,				// 38
	PIGPIO_IN_SRQ = 21,		// 40 not connected yet
	PIGPIO_IN_BUTTON2 = 22,	// 15 Common
	PIGPIO_IN_BUTTON3 = 23,	// 16 Common
	PIGPIO_IN_ATN = 24,		// 18
	PIGPIO_IN_DATA = 25,	// 22
	//GPIO 26,				// 37
#if defined(RPI1BREV1)
	PIGPIO_IN_BUTTON1 = 21	// 13 Common
#else
	PIGPIO_IN_BUTTON1 = 27	// 13 Common
#endif
};
#endif



enum PIGPIOMasks
{
	// Non-split lines
	//PIGPIO_MASK_IN_ATN = 1 << PIGPIO_ATN,
	//PIGPIO_MASK_IN_DATA = 1 << PIGPIO_DATA,
	//PIGPIO_MASK_IN_CLOCK = 1 << PIGPIO_CLOCK,
	//PIGPIO_MASK_IN_SRQ = 1 << PIGPIO_SRQ,
	//PIGPIO_MASK_IN_RESET = 1 << PIGPIO_RESET,

	// Split lines
	//PIGPIO_MASK_IN_ATN = 1 << PIGPIO_IN_ATN,
	//PIGPIO_MASK_IN_DATA = 1 << PIGPIO_IN_DATA,
	//PIGPIO_MASK_IN_CLOCK = 1 << PIGPIO_IN_CLOCK,
	//PIGPIO_MASK_IN_SRQ = 1 << PIGPIO_IN_SRQ,
	//PIGPIO_MASK_IN_RESET = 1 << PIGPIO_IN_RESET,

	PIGPIO_MASK_IN_BUTTON1 = 1 << PIGPIO_IN_BUTTON1,
	PIGPIO_MASK_IN_BUTTON2 = 1 << PIGPIO_IN_BUTTON2,
	PIGPIO_MASK_IN_BUTTON3 = 1 << PIGPIO_IN_BUTTON3,
	PIGPIO_MASK_IN_BUTTON4 = 1 << PIGPIO_IN_BUTTON4,
	PIGPIO_MASK_IN_BUTTON5 = 1 << PIGPIO_IN_BUTTON5,
	PIGPIO_MASK_ANY_BUTTON = PIGPIO_MASK_IN_BUTTON1 | PIGPIO_MASK_IN_BUTTON2 | PIGPIO_MASK_IN_BUTTON3 | PIGPIO_MASK_IN_BUTTON4 | PIGPIO_MASK_IN_BUTTON5
};

static const unsigned ButtonPinFlags[5] = { PIGPIO_MASK_IN_BUTTON1, PIGPIO_MASK_IN_BUTTON2, PIGPIO_MASK_IN_BUTTON3, PIGPIO_MASK_IN_BUTTON4, PIGPIO_MASK_IN_BUTTON5 };

///////////////////////////////////////////////////////////////////////////////////////////////
// Original Non-split lines
///////////////////////////////////////////////////////////////////////////////////////////////
// I kind of stuffed up selecting what pins should be what.
// Originally I wanted the hardware interface to be as simple as possible and was focused on all the connections down one end of the Pi header.
// Also, originally I was only worried about ATN, DATA, CLOCK and RESET. Buttons were optional. With DATA and CLOCK only needing to go output.
// With hindsight (now wanting to support NIBTools ATN should have been put in the same pin group as DATA and CLOCK);
// This now requires 2 GPIO writes as PIN2 is in SEL0 and pins 17 and 18 are in SEL1.
//GPFSEL0
//	9	8	7	6	5	4	3	2	1	0
// 000 000 000 000 000 000 000 001 000 000
// To support NIBTools ATN can be made an output as well
static const u32 PI_OUTPUT_MASK_GPFSEL0 = ~((1 << (PIGPIO_ATN * 3)));
//GPFSEL1           RX  TX
//	19	18	17	16	15	14	13	12	11	10
// 000 001 001 000 000 000 000 000 000 000	(bits 21 and 24 for GPIO 17 and 18 as outputs)
//static const u32 PI_OUTPUT_MASK_GPFSEL1 = ~((1 << 21) | (1 << 24));
// 000 001 001 001 000 000 001 000 000 000	(bits 21 and 24 for GPIO 17 and 18 as outputs)
//static const u32 PI_OUTPUT_MASK_GPFSEL1 = ~((1 << ((PIGPIO_OUT_SOUND - 10) * 3)) | (1 << ((PIGPIO_OUT_LED - 10) * 3)) | (1 << ((PIGPIO_CLOCK - 10) * 3)) | (1 << ((PIGPIO_DATA - 10) * 3)));
static const u32 PI_OUTPUT_MASK_GPFSEL1 = ~((1 << ((PIGPIO_CLOCK - 10) * 3)) | (1 << ((PIGPIO_DATA - 10) * 3)));
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
// Pinout for those that want to split the lines (and the common ones like buttons, sound and LED)
///////////////////////////////////////////////////////////////////////////////////////////////
// OUT (need 7)
//  CLK
//  DATA
//  SRQ
//  ATN
//  RST
//  LED
//  SND

// IN (need 10)
//  CLK
//  DATA
//  ATN
//  RST
//  SRQ 
//  BTN1
//  BTN2
//  BTN3
//  BTN4
//  BTN5

//GPFSEL0
//  S   S   S                    
//  P   P   P               I   I
//  I   I   I               2   2
//  0   0   0               C   C
//  M   C   C               S   S   I   I
//  I   E   E               C   D   D   D
//  S   0   1   W   R   R   L   A   S   S
//  O           RST B5  B4  1   1   D   C
//	9	8	7	6	5	4	3	2	1	0
//GPFSEL1
//                  RX  TX          S   S
//                  RX  TX          P   P
//                  RX  TX          I   I
//                  RX  TX          0   0
//                  RX  TX              M
//                  RX  TX          C   O
//  W   W   W   W   RX  TX  W   W   L   S
//  SRQ DTA CLK LED RX  TX  SND ATN K   I
//	19	18	17	16	15	14	13	12	11	10
//GPFSEL2
//  X   X                           
//  X   X                           
//  X   X                           
//  X   X                           
//  X   X                           
//  X   X                           
//  X   X   R   R   R   R    R   R  R   R 
//  X   X   B1  CLK DTA ATN B3  B2  SRQ RST  
//	29	28	27	26	25	24	23	22	21	20
// 000 000 000 000 000 000 000 000 000 000
///////////////////////////////////////////////////////////////////////////////////////////////

enum VIAPortPins
{
	VIAPORTPINS_DATAIN = 0x01,	//pb0
	VIAPORTPINS_DATAOUT = 0x02,	//pb1
	VIAPORTPINS_CLOCKIN = 0x04,	//pb2
	VIAPORTPINS_CLOCKOUT = 0x08,//pb3
	VIAPORTPINS_ATNAOUT = 0x10,	//pb4
	VIAPORTPINS_ATNOUT = 0x40,	//pb6 - CMD HD only. A 1541 uses pb5/pb6 as
								//device number jumpers, but the CMD HD keeps
								//its device number on disk, so CMD wired pb6
								//as an ATN *output*. HDOS asserts it (and
								//expects to see its own assertion back on CA1)
								//while performing the front panel SWAP.
	VIAPORTPINS_ATNIN = 0x80	//bp7
};

typedef bool(*CheckStatus)();

class IEC_Bus
{
public:
	static inline void Initialise(void)
	{
		volatile int index; // Force a real delay in the loop below.

		// Clear all outputs to 0
		write32(ARM_GPIO_GPCLR0, 0xFFFFFFFF);

		if (!splitIECLines)
		{
			// This means that when any pin is turn to output it will output a 0 and pull lines low (ie an activation state on the IEC bus)
			// Note: on the IEC bus you never output a 1 you simply tri state and it will be pulled up to a 1 (ie inactive state on the IEC bus) if no one else is pulling it low.

			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_SOUND, FS_OUTPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_LED, FS_OUTPUT);

			// Take a copy of GPFSEL1 now that the pin functions are set the way
			// this build wants them.
			//
			// RefreshOutsCMDHD rewrites the whole register on this wiring -
			// that is how it drives DATA and CLOCK, by switching them between
			// input and output - and it builds the new value from this copy.
			// The copy was left at 0, the assignment that would have filled it
			// in having been commented out, so every refresh wrote zeroes into
			// the function bits of GPIO 10-19 and put SOUND (13), LED (16) and
			// the UART pins back to input, undoing the two calls immediately
			// above. The activity LED and the sound output had therefore never
			// worked on non split wiring.
			myOutsGPFSEL1 = read32(ARM_GPIO_GPFSEL1);
		}
		else
		{
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_BUTTON4, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_BUTTON5, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_RESET, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_SRQ, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_BUTTON2, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_BUTTON3, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_ATN, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_DATA, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_CLOCK, FS_INPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_IN_BUTTON1, FS_INPUT);


			//RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_RESET, FS_OUTPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_SPI0_RS, FS_OUTPUT);
			
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_ATN, FS_OUTPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_SOUND, FS_OUTPUT);
			// Note: the ATN output pin is deliberately NOT configured here. It
			// is open collector and idles as an input (released); RefreshAtnOut
			// briefly turns it into an output to pull ATN low. See there.
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_LED, FS_OUTPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_CLOCK, FS_OUTPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_DATA, FS_OUTPUT);
			RPI_SetGpioPinFunction((rpi_gpio_pin_t)PIGPIO_OUT_SRQ, FS_OUTPUT);
		}
	
#if not defined(EXPERIMENTALZERO)
		// Set up audio.
		write32(CM_PWMDIV, CM_PASSWORD + 0x2000);
		write32(CM_PWMCTL, CM_PASSWORD + CM_ENAB + CM_SRC_OSCILLATOR);	// Use Default 100MHz Clock
		// Set PWM
		write32(PWM_RNG1, 0x1B4);	// 8bit 44100Hz Mono
		write32(PWM_RNG2, 0x1B4);
		write32(PWM_CTL, PWM_USEF2 + PWM_PWEN2 + PWM_USEF1 + PWM_PWEN1 + PWM_CLRF1);
#endif

		for (index = 0; index < buttonCount; ++index)
		{
			InputButton[index] = false;
			InputButtonPrev[index] = false;
			buttonPressed[index] = false;
			inputRepeatThreshold[index] = INPUT_BUTTON_REPEAT_THRESHOLD;
			inputRepeat[index] = 0;
			inputRepeatPrev[index] = 0;
		}

		// Enable the internal pullups for the input button pins using the method described in BCM2835-ARM-Peripherals manual.
		RPI_GpioBase->GPPUD = 2;
		for (index = 0; index < 150; ++index)
		{
		}
		RPI_GpioBase->GPPUDCLK0 = PIGPIO_MASK_IN_BUTTON1 | PIGPIO_MASK_IN_BUTTON2 | PIGPIO_MASK_IN_BUTTON3 | PIGPIO_MASK_IN_BUTTON4 | PIGPIO_MASK_IN_BUTTON5;
		for (index = 0; index < 150; ++index)
		{
		}
		RPI_GpioBase->GPPUD = 0;
		RPI_GpioBase->GPPUDCLK0 = 0;
	}

	static inline void LetSRQBePulledHigh()
	{
		SRQSetToOut = IEC_Bus::invertIECInputs;
		RefreshOutsCMDHD();
	}

#if defined(EXPERIMENTALZERO)
	static inline bool AnyButtonPressed()
	{
		return ((gplev0 & PIGPIO_MASK_ANY_BUTTON) != PIGPIO_MASK_ANY_BUTTON);
	}
#endif

	// Debounce/repeat timing here is measured in real elapsed microseconds
	// (via ARM_SYSTIMER_CLO), not in call count. INPUT_BUTTON_DEBOUNCE_THRESHOLD
	// and INPUT_BUTTON_REPEAT_THRESHOLD were written as raw counter increments
	// on the assumption that this is called about once per microsecond, and by
	// now nothing calls it that often: the emulation loop polls the front panel
	// on its slow tick, one iteration in SLOW_TICK_EVERY, and the file browser
	// polls slower still and not even regularly, since it does directory
	// scanning and screen drawing in the same loop. Counting calls, a press
	// never accumulated anywhere near 20000 of them and every button on the
	// front panel appeared dead. Elapsed time keeps 20000 meaning 20ms and
	// 460000 meaning 460ms whatever the caller's rate is, so no caller has to
	// tell this function how often it runs and decimating a loop cannot break
	// the front panel again.
	//
	// nowUs is passed in rather than read here so that one ARM_SYSTIMER_CLO
	// read serves all five buttons. That read is an uncached peripheral access
	// and blocks for about 69 cycles, so five of them would be most of the cost
	// of a poll for no benefit. All five buttons also then share one timestamp,
	// which is marginally more correct than five staggered reads.
	//
	// The subtraction below is deliberately unsigned: ARM_SYSTIMER_CLO is the
	// low 32 bits of the 1MHz counter and wraps every ~71.6 minutes, and u32
	// wraparound makes the elapsed time come out right across that boundary.
	// It looks like a bug and is not - please leave it alone.
	static void UpdateButton(int index, unsigned gplev0, u32 nowUs)
	{
		bool inputcurrent = (gplev0 & ButtonPinFlags[index]) == 0;

		InputButtonPrev[index] = InputButton[index];
		inputRepeatPrev[index] = inputRepeat[index];

		if (inputcurrent)
		{
			if (!buttonPressed[index])
			{
				buttonPressed[index] = true;
				pressStartTime[index] = nowUs;		// start of this press streak
			}

			u32 heldUs = nowUs - pressStartTime[index];

			if (!InputButton[index] && heldUs >= INPUT_BUTTON_DEBOUNCE_THRESHOLD)
			{
				InputButton[index] = true;
				inputRepeatThreshold[index] = INPUT_BUTTON_DEBOUNCE_THRESHOLD + INPUT_BUTTON_REPEAT_THRESHOLD;
				inputRepeat[index]++;
			}

			if (InputButton[index] && heldUs >= inputRepeatThreshold[index])
			{
				inputRepeat[index]++;
				inputRepeatThreshold[index] += INPUT_BUTTON_REPEAT_THRESHOLD / inputRepeat[index];
			}
		}
		else
		{
			InputButton[index] = false;
			buttonPressed[index] = false;
			inputRepeatThreshold[index] = INPUT_BUTTON_REPEAT_THRESHOLD;
			inputRepeat[index] = 0;
			inputRepeatPrev[index] = 0;
		}
	}

	


	static void ReadBrowseMode(void);
	static void ReadGPIOUserInput(void);
	static void ReadEmulationModeCMDHD(void);
	static void ReadBusInputsCMDHD(void);

	static void WaitUntilReset(void)
	{
		unsigned gplev0;
		do
		{
			gplev0 = read32(ARM_GPIO_GPLEV0);
			Resetting = !ignoreReset && ((gplev0 & PIGPIO_MASK_IN_RESET) == \
				 (invertIECInputs ? PIGPIO_MASK_IN_RESET : 0));

			if (Resetting)
				IEC_Bus::WaitMicroSeconds(100);
		}
		while (Resetting);
	}

	// Out going
	static void PortB_OnPortOut(void* pUserData, unsigned char status);

	// Refresh all the outputs: DATA, CLOCK, SRQ (fast serial), the activity
	// LED, the sound pin and ATN out.
	//
	// This runs once per loop and - event driven - from PortB_OnPortOut(),
	// which calls it as soon as one of the output flags actually moves, so a
	// CPU write to U10 ORB/DDRB reaches the wire inside the same emulated
	// cycle instead of waiting up to ~1us for the next periodic refresh. That
	// ~1us of slack was enough to occasionally misalign GEOS's
	// 2-bit-per-sample turbo protocol, whose symbol gaps run only ~8-11us: a
	// whole microsecond of latency is a large fraction of one symbol, and the
	// receiver can sample the wrong half of a bit pair. Nearly everything below
	// is a write to device memory, which is posted and does not stall the
	// caller; the one exception is the read-modify-write of GPFSEL inside
	// RefreshAtnOut, which that function skips unless AtnSetToOut has actually
	// changed. So running this twice inside a microsecond is affordable.
	static inline void RefreshOutsCMDHD(void)
	{
		unsigned set = 0;
		unsigned clear = 0;
		unsigned tmp;

		if (!splitIECLines)
		{
			unsigned outputs = 0;

			if (AtnaDataSetToOut || DataSetToOut) outputs |= (FS_OUTPUT << ((PIGPIO_DATA - 10) * 3));
			if (ClockSetToOut) outputs |= (FS_OUTPUT << ((PIGPIO_CLOCK - 10) * 3));
			//if (SRQSetToOut) outputs |= (FS_OUTPUT << ((PIGPIO_SRQ - 10) * 3));			// For Option A hardware we should not support pulling more than 2 lines low at any one time!

			unsigned nValue = (myOutsGPFSEL1 & PI_OUTPUT_MASK_GPFSEL1) | outputs;
			write32(ARM_GPIO_GPFSEL1, nValue);

			// One GPFSEL1 write moves both lines at once, so the transient the
			// split path below has to order around cannot arise on this wiring.
			RefreshAtnOut();
		}
		else
		{
			// DATA and CLOCK are kept in bus terms - asserted means "pulling
			// the line low" - for as long as possible. Which physical level
			// that is depends on the buffer: with an inverting one (7406,
			// invertIECOutputs = 1) asserting drives the pin high; with a non
			// inverting one (7407, which options.txt also documents) it drives
			// it low. Choosing the registers from the polarity and then
			// deciding the write order in bus terms is what keeps the ordering
			// below correct on both, instead of only on the wiring it was
			// developed against.
			unsigned assertBits = 0;
			unsigned releaseBits = 0;

			if (AtnaDataSetToOut || DataSetToOut) assertBits |= 1 << PIGPIO_OUT_DATA;
			else releaseBits |= 1 << PIGPIO_OUT_DATA;

			if (ClockSetToOut) assertBits |= 1 << PIGPIO_OUT_CLOCK;
			else releaseBits |= 1 << PIGPIO_OUT_CLOCK;

			u32 assertReg = invertIECOutputs ? ARM_GPIO_GPSET0 : ARM_GPIO_GPCLR0;
			u32 releaseReg = invertIECOutputs ? ARM_GPIO_GPCLR0 : ARM_GPIO_GPSET0;

			// GPSET0/GPCLR0 are two separate register writes, not one atomic
			// change. If this call moves one line from released to asserted
			// and the other from asserted to released at the same time,
			// writing them in the "wrong" order leaves a few-cycle window in
			// which the old level of one line is combined with the new level
			// of the other - exactly the kind of transient a 2-bit-per-sample
			// GEOS turbo read can latch as a corrupted nibble. Releasing first
			// makes that window one where both lines are released, which no
			// receiver reads as a symbol; asserting first would make it one
			// where both are asserted, which is a symbol. So the order is
			// swapped only when both changes happen together, and the original
			// assert-then-release order stands the rest of the time.
			bool newlyAsserted = (assertBits & ~oldAssertBits) != 0;
			bool newlyReleased = (releaseBits & ~oldReleaseBits) != 0;

			if (newlyAsserted && newlyReleased)
			{
				write32(releaseReg, releaseBits);
				write32(assertReg, assertBits);
			}
			else
			{
				write32(assertReg, assertBits);
				write32(releaseReg, releaseBits);
			}

			oldAssertBits = assertBits;
			oldReleaseBits = releaseBits;

			// SRQ is the fast serial clock rather than one of the two lines
			// the ordering above protects, so it stays in the single set/clear
			// pair that the LED and the sound pin already share and adds no
			// register writes of its own.
			if (SRQSetToOut) set |= 1 << PIGPIO_OUT_SRQ;	// fast clock is pulled high but we have an inverter in our hardware so to compensate we invert in software now
			else clear |= 1 << PIGPIO_OUT_SRQ;

			if (!invertIECOutputs) {
				tmp = set;
				set = clear;
				clear = tmp;
			}
		}

		if (OutputLED) set |= 1 << PIGPIO_OUT_LED;
		else clear |= 1 << PIGPIO_OUT_LED;

		if (OutputSound) set |= 1 << PIGPIO_OUT_SOUND;
		else clear |= 1 << PIGPIO_OUT_SOUND;

		write32(ARM_GPIO_GPSET0, set);
		write32(ARM_GPIO_GPCLR0, clear);

		// ATN out on this wiring too. The non split arm above does it on its
		// way through, so either way an event driven refresh pushes ATN in the
		// same cycle as DATA and CLOCK. It is nearly free: RefreshAtnOut only
		// touches GPFSEL when AtnSetToOut has actually changed, which happens
		// for about a millisecond during a SWAP and never otherwise.
		if (splitIECLines)
			RefreshAtnOut();
	}

	// The CMD HD can drive ATN itself (U10 pb6) while swapping device numbers,
	// which no Commodore floppy drive ever does. ATN is open collector, so the
	// pin is driven the same way Pi1541 drives the non split lines: its output
	// latch is always 0, so making it an output pulls the line low and making
	// it an input again releases it. That is safe on any wiring because the pin
	// is never driven high, so it cannot fight the host or another device.
	//
	// On a Pi1541io the ATN line reaches the Pi through a BSS138 bidirectional
	// level shifter, so the very pin used to read ATN (GPIO 24) can drive it
	// too - no board modification needed.
	static inline void RefreshAtnOut(void)
	{
		if (!atnOutGPIO)
			return;

		static bool lastAtnSetToOut = false;
		if (AtnSetToOut == lastAtnSetToOut)
			return;					// GPFSEL is shared, so only touch it on a change
		lastAtnSetToOut = AtnSetToOut;

		u32 reg = ARM_GPIO_GPFSEL0 + ((atnOutGPIO / 10) * 4);
		u32 shift = (atnOutGPIO % 10) * 3;
		u32 value = read32(reg) & ~(7 << shift);
		if (AtnSetToOut)
			value |= (FS_OUTPUT << shift);
		write32(reg, value);
	}

	static void WaitMicroSeconds(u32 amount)
	{
		u32 count;

		for (count = 0; count < amount; ++count)
		{
			unsigned before;
			unsigned after;
			// We try to update every micro second and use as a rough timer to count micro seconds
			before = read32(ARM_SYSTIMER_CLO);
			do
			{
				after = read32(ARM_SYSTIMER_CLO);
			} while (after == before);
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Fast Serial
	static inline void SetFastSerialData(bool value)
	{
		DataSetToOut = value;
	}
	static inline void SetFastSerialSRQ(bool value)
	{
		SRQSetToOut = value;
	}


	static inline bool GetPI_SRQ() { return PI_SRQ; }
	static inline bool GetPI_Atn() { return PI_Atn; }
	static inline bool IsAtnAsserted() { return PI_Atn; }
	static inline bool IsAtnReleased() { return !PI_Atn; }
	static inline bool GetPI_Data() { return PI_Data; }
	static inline bool IsDataAsserted() { return PI_Data; }
	static inline bool IsDataReleased() { return !PI_Data; }
	static inline bool GetPI_Clock() { return PI_Clock; }
	static inline bool IsClockAsserted() { return PI_Clock; }
	static inline bool IsClockReleased() { return !PI_Clock; }
	static inline bool GetPI_Reset() { return PI_Reset; }
	static inline bool IsDataSetToOut() { return DataSetToOut; }
	//static inline bool IsAtnaDataSetToOut() { return AtnaDataSetToOut; }
	static inline bool IsClockSetToOut() { return ClockSetToOut; }
	static inline bool IsAtnSetToOut() { return AtnSetToOut; }
	static inline bool IsReset() { return Resetting; }

	static inline void WaitWhileAtnAsserted()
	{
		while (IsAtnAsserted())
		{
			ReadBrowseMode();
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////////////

	static inline void SetSplitIECLines(bool value)
	{
		splitIECLines = value;
		if (splitIECLines)
		{
			PIGPIO_MASK_IN_ATN = 1 << PIGPIO_IN_ATN;
			PIGPIO_MASK_IN_DATA = 1 << PIGPIO_IN_DATA;
			PIGPIO_MASK_IN_CLOCK = 1 << PIGPIO_IN_CLOCK;
			PIGPIO_MASK_IN_SRQ = 1 << PIGPIO_IN_SRQ;
			PIGPIO_MASK_IN_RESET = 1 << PIGPIO_IN_RESET;
		}
	}

	static inline void SetInvertIECInputs(bool value) 
	{
		invertIECInputs = value;
		if (value)
		{
			PI_Atn = !PI_Atn;
			PI_Data = !PI_Data;
			PI_Clock = !PI_Clock;
			PI_SRQ = !PI_SRQ;
			PI_Reset = !PI_Reset;
		}
	}

	static inline void SetInvertIECOutputs(bool value)
	{
		invertIECOutputs = value;
	}

	static inline void SetIgnoreReset(bool value)
	{
		ignoreReset = value;
	}

	// Which GPIO drives the IEC ATN line, so the emulated CMD HD can take
	// control of the bus while swapping device numbers. Board specific; 0
	// disables it (the drive then simply never drives ATN).
	static inline void SetAtnOutGPIO(u32 gpio)
	{
		// Range check it here, once, rather than trusting options.txt.
		// RefreshAtnOut turns this straight into a register address -
		// ARM_GPIO_GPFSEL0 + (gpio / 10) * 4 - so a typo like 240 in the
		// config file walks out of the GPIO bank entirely and starts
		// read-modify-writing whatever peripheral registers happen to follow.
		// The BCM283x has GPIO 0-53; anything else disables the function,
		// which is what 0 already means.
		atnOutGPIO = (gpio <= 53) ? gpio : 0;
	}

	// CA1 input ATN
	// If CA1 is ever set to output
	//	- CA1 will start to drive pb7
	static m6522* VIA;
	static IOPort* port;

	static void Reset(void);

	static bool GetInputButtonPressed(int buttonIndex) { return InputButton[buttonIndex] && !InputButtonPrev[buttonIndex]; }
	static bool GetInputButtonReleased(int buttonIndex) { return InputButton[buttonIndex] == false; }
	static bool GetInputButton(int buttonIndex) { return InputButton[buttonIndex]; }
	static bool GetInputButtonRepeating(int buttonIndex) { return inputRepeat[buttonIndex] != inputRepeatPrev[buttonIndex]; }
	static bool GetInputButtonHeld(int buttonIndex) { return inputRepeatThreshold[buttonIndex] >= INPUT_BUTTON_DEBOUNCE_THRESHOLD + (INPUT_BUTTON_REPEAT_THRESHOLD * 2); }

	static bool OutputLED;
	static bool OutputSound;

private:
	// Previous DATA/CLOCK state in bus terms (asserted = pulling low), not raw
	// pin levels - see RefreshOutsCMDHD(). Only ever covers PIGPIO_OUT_DATA
	// and PIGPIO_OUT_CLOCK; nothing else writes them.
	static u32 oldReleaseBits;
	static u32 oldAssertBits;

	static bool splitIECLines;
	static bool invertIECInputs;
	static bool invertIECOutputs;
	static bool ignoreReset;

	static u32 PIGPIO_MASK_IN_ATN;
	static u32 PIGPIO_MASK_IN_DATA;
	static u32 PIGPIO_MASK_IN_CLOCK;
	static u32 PIGPIO_MASK_IN_SRQ;
	static u32 PIGPIO_MASK_IN_RESET;

	static u32 emulationModeCheckButtonIndex;

	static unsigned gplev0;

	static bool PI_Atn;
	static bool PI_Data;
	static bool PI_Clock;
	static bool PI_SRQ;
	static bool PI_Reset;

	static bool VIA_Atna;
	static bool VIA_Data;
	static bool VIA_Clock;

	static bool DataSetToOut;
	static bool AtnaDataSetToOut;
	// Snapshot of the three above, taken at the end of each full sample, for the
	// trimmed mid-loop sample to use instead of the live flags.
	static bool sampledDataSetToOut;
	static bool sampledAtnaDataSetToOut;
	static bool sampledClockSetToOut;
	static bool ClockSetToOut;
	static bool SRQSetToOut;
	static bool AtnSetToOut;
	static u32 atnOutGPIO;		// 0 = the drive cannot drive ATN
	static bool Resetting;

	static int buttonCount;

	static u32 myOutsGPFSEL0;
	static u32 myOutsGPFSEL1;
	static bool InputButton[5];
	static bool InputButtonPrev[5];
	// True while the button reads pressed, so UpdateButton can tell the first
	// call of a press streak (when pressStartTime is stamped) from the rest.
	static bool buttonPressed[5];
	static u32 inputRepeatThreshold[5];
	static u32 inputRepeat[5];
	static u32 inputRepeatPrev[5];
	static u32 pressStartTime[5];

};
#endif
