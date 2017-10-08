#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "MMA8451.h"
#include <util/delay.h>
#include <math.h>


#define DEBUG_PRINTF
#ifdef DEBUG_PRINTF
#include "uart.h"
#endif

#define ORIENTATION_PORTS_AT_RIGHT
// Calculate the value needed for
// the CTC match value in OCR1A.
#define CTC_MATCH_OVERFLOW ((F_CPU / 1000) / 8)

volatile unsigned long timer1_millis;
long milliseconds_since;

unsigned long start;

ISR (TIMER3_COMPA_vect)
{
	timer1_millis++;
}

unsigned long millis (void)
{
	unsigned long millis_return;
	// Ensure this cannot be disrupted
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		millis_return = timer1_millis;
	}
	return millis_return;
}

// accelerometer input history item, for gathering calibration data
typedef struct AccHist
{

	// reading for this entry
	float x;
	float y;

	// distance from previous entry
	float d;

	// total and count of samples averaged over this period
	float xtot, ytot;
	int cnt;
} AccHist_t;

typedef struct AccPos
{
	int x;
	int y;
} AccPos_t;

float distance(AccHist_t *cur, AccHist_t *prev)
{
	return sqrt(square(prev->x - cur->x) + square(prev->y - cur->y));
}


void set(AccHist_t *cur, float x, float y, AccHist_t *prv)
{
	// save the raw position
	cur->x = x;
	cur->y = y;
	cur->d = distance(cur, prv);
}


int cnt=0;

void clearAvg(AccHist_t *cur) { cur->xtot = cur->ytot = 0.0; cur->cnt = 0; }
void addAvg(AccHist_t *cur, float x, float y) { cur->xtot += x; cur->ytot += y; cur->cnt=cur->cnt+1; }
const float xAvg(AccHist_t cur) { return cur.xtot/cur.cnt; }
const float yAvg(AccHist_t cur) { return cur.ytot/cur.cnt; }

/*************************************************/

// --------------------------------------------------------------------------
//
// Set up mappings for the joystick X and Y reports based on the mounting
// orientation of the KL25Z in the cabinet.  Visual Pinball and other
// pinball software effectively use video coordinates to define the axes:
// positive X is to the right of the table, negative Y to the left, positive
// Y toward the front of the table, negative Y toward the back.  The KL25Z
// accelerometer is mounted on the board with positive Y toward the USB
// ports and positive X toward the right side of the board with the USB
// ports pointing up.  It's a simple matter to remap the KL25Z coordinate
// system to match VP's coordinate system for mounting orientations at
// 90-degree increments...
//
#if defined(ORIENTATION_PORTS_AT_FRONT)
# define JOY_X(x, y)   (y)
# define JOY_Y(x, y)   (x)
#elif defined(ORIENTATION_PORTS_AT_LEFT)
# define JOY_X(x, y)   (-(x))
# define JOY_Y(x, y)   (y)
#elif defined(ORIENTATION_PORTS_AT_RIGHT)
# define JOY_X(x, y)   (x)
# define JOY_Y(x, y)   (-(y))
#elif defined(ORIENTATION_PORTS_AT_REAR)
# define JOY_X(x, y)   (-(y))
# define JOY_Y(x, y)   (-(x))
#else
# error Please define one of the ORIENTATION_PORTS_AT_xxx macros to establish the accelerometer orientation in your cabinet
#endif

// --------------------------------------------------------------------------
//
// Joystick axis report range - we report from -JOYMAX to +JOYMAX
//
#define JOYMAX 4096


// last raw acceleration readings
float ax_, ay_, az_;

// integrated velocity reading since last get()
float vx_, vy_;

// timer for measuring time between get() samples
unsigned long tGet_;

// timer for measuring time between interrupts
unsigned long tInt_;

// Calibration reference point for accelerometer.  This is the
// average reading on the accelerometer when in the neutral position
// at rest.
float cx_, cy_;

// timer for atuo-centering
unsigned long tCenter_;

// Auto-centering history.  This is a separate history list that
// records results spaced out sparesely over time, so that we can
// watch for long-lasting periods of rest.  When we observe nearly
// no motion for an extended period (on the order of 5 seconds), we
// take this to mean that the cabinet is at rest in its neutral
// position, so we take this as the calibration zero point for the
// accelerometer.  We update this history continuously, which allows
// us to continuously re-calibrate the accelerometer.  This ensures
// that we'll automatically adjust to any actual changes in the
// cabinet's orientation (e.g., if it gets moved slightly by an
// especially strong nudge) as well as any systematic drift in the
// accelerometer measurement bias (e.g., from temperature changes).
int iAccPrv_, nAccPrv_;
#define maxAccPrv  5
AccHist_t accPrv_[maxAccPrv];


// interrupt router
////InterruptIn intIn_;

void reset(void)
{
	// clear the center point
	cx_ = cy_ = 0.0;

	// start the calibration timer
	tCenter_=millis();
	iAccPrv_ = nAccPrv_ = 0;

	// reset and initialize the MMA8451Q
	if (!MMA8451_begin(MMA8451_DEFAULT_ADDRESS, MMA8451_RANGE_2_G)) {
		//FIXME LIMITER LA BOUCLE
		while (1) ;
	}

	// set the initial integrated velocity reading to zero
	vx_ = vy_ = 0;

	// read the current registers to clear the data ready flag
	MMA8451_read();

	// start our timers
	tGet_=millis();
	tInt_=millis();
}

// adjust a raw acceleration figure to a usb report value
int rawToReport(float v)
{
	// scale to the joystick report range and round to integer
	int i = ((int)(round(v*JOYMAX)));

	// if it's near the center, scale it roughly as 20*(i/20)^2,
	// to suppress noise near the rest position
	static const int filter[] = {
		-18, -16, -14, -13, -11, -10, -8, -7, -6, -5, -4, -3, -2, -2, -1, -1, 0, 0, 0, 0,
		0,
		0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 10, 11, 13, 14, 16, 18
	};
	return (i > 20 || i < -20 ? i : filter[i+20]);
}

void get(AccPos_t *pos)
{
	// read the shared data and store locally for calculations
	float ax = ax_, ay = ay_;
	float vx = vx_, vy = vy_;

	// reset the velocity sum for the next run
	vx_ = vy_ = 0;

	// get the time since the last get() sample
	float dt = (millis()-tGet_)/1.0e3;
	tGet_=millis();

	// adjust the readings for the integration time
	vx /= dt;
	vy /= dt;

	// add this sample to the current calibration interval's running total
	AccHist_t *p = accPrv_ + iAccPrv_;
	addAvg(p, ax, ay);

	// check for auto-centering every so often
	if ((millis() - tCenter_) > 1000)
	{
		// add the latest raw sample to the history list
		AccHist_t *prv = p;
		iAccPrv_ = (iAccPrv_ + 1) % maxAccPrv;
		p = accPrv_ + iAccPrv_;
		set(p, ax, ay, prv);

		// if we have a full complement, check for stability
		if (nAccPrv_ >= maxAccPrv)
		{
			// check if we've been stable for all recent samples
			static const float accTol = .01;
			AccHist_t *p0 = accPrv_;
			if (p0[0].d < accTol
					&& p0[1].d < accTol
					&& p0[2].d < accTol
					&& p0[3].d < accTol
					&& p0[4].d < accTol)
			{
				// Figure the new calibration point as the average of
				// the samples over the rest period
				cx_ = (xAvg(p0[0]) + xAvg(p0[1]) + xAvg(p0[2]) + xAvg(p0[3]) + xAvg(p0[4]))/5.0;
				cy_ = (yAvg(p0[0]) + yAvg(p0[1]) + yAvg(p0[2]) + yAvg(p0[3]) + yAvg(p0[4]))/5.0;
			}
		}
		else
		{
			// not enough samples yet; just up the count
			++nAccPrv_;
		}

		// clear the new item's running totals
		clearAvg(p);

		// reset the timer
		tCenter_=millis();
	}

	// report our integrated velocity reading in x,y
	pos->x = rawToReport(vx);
	pos->y = rawToReport(vy);

#ifdef DEBUG_PRINTF2
	if (pos->x != 0 || pos->y != 0)
		printf("=> %f %f %d %d %f\r\n", vx, vy, pos->x, pos->y, dt);
#endif
}

void UpdateMMA8451_NUDGE(void){
	// Read the axes.  Note that we have to read all three axes
	// (even though we only really use x and y) in order to clear
	// the "data ready" status bit in the accelerometer.  The
	// interrupt only occurs when the "ready" bit transitions from
	// off to on, so we have to make sure it's off.
	MMA8451_read();
	float x, y, z;
	x = MMA8451_x_g;
	y = MMA8451_y_g;
	z = MMA8451_z_g;
	// calculate the time since the last interrupt
	float dt = (millis()-tInt_)/1.0e3;
	tInt_=millis();

	// integrate the time slice from the previous reading to this reading
	vx_ += (x + ax_ - 2*cx_)*dt/2;
	vy_ += (y + ay_ - 2*cy_)*dt/2;

	// store the updates
	ax_ = x;
	ay_ = y;
	az_ = z;
}

int8_t map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int main(void)
{
	// CTC mode, Clock/8
	TCCR3B |= (1 << WGM12) | (1 << CS31);

	// Load the high byte, then the low byte
	// into the output compare
	OCR3AH = (CTC_MATCH_OVERFLOW >> 8);
	OCR3AL = CTC_MATCH_OVERFLOW;

	// Enable the compare match interrupt
	TIMSK3 |= (1 << OCIE3A);
	// Now enable global interrupts
	sei();

#ifdef DEBUG_PRINTF
	uart_init();
	stdout = &uart_output;
	stdin  = &uart_input;
	puts("Hello world!");
	puts("Adafruit MMA8451 test!");
#endif


	if (!MMA8451_begin(MMA8451_DEFAULT_ADDRESS, MMA8451_RANGE_2_G)) {
#ifdef DEBUG_PRINTF
		puts("Couldnt start");
#endif
		while (1) ;
	}
#ifdef DEBUG_PRINTF
	puts("MMA8451 found!");

	printf("Range = "); printf("%d",(2 << MMA8451_getRange()));
	puts("G");
	printf("DataRate = "); printf("%d\n",(MMA8451_getDataRate()));
#endif
	_delay_ms(250);
	MMA8451_read();

	reset();

	// last accelerometer report, in joystick units (we report the nudge
	// acceleration via the joystick x & y axes, per the VP convention)
	int x = 0, y = 0;

	start = millis();
	int center = 1;
	while (1)
	{
			UpdateMMA8451_NUDGE();
			AccPos_t pos;pos.x=pos.y=0;
			get(&pos);
			int xa=pos.x, ya=pos.y;
			// confine the results to our joystick axis range
			if (xa < -JOYMAX) xa = -JOYMAX;
			if (xa > JOYMAX) xa = JOYMAX;
			if (ya < -JOYMAX) ya = -JOYMAX;
			if (ya > JOYMAX) ya = JOYMAX;

			// store the updated accelerometer coordinates
			x = xa;
			y = ya;

			int8_t joy_x = 0;
			int8_t joy_y = 0;

			if((millis()-start>10000)&&(abs(x) > 100 || abs(y)>100)){
				joy_x=map(x,-JOYMAX,JOYMAX,-128,127);
				joy_y=map(-y,-JOYMAX,JOYMAX,-128,127);
#ifdef DEBUG_PRINTF
				printf("X/Y X/Y : %d %d %d %d\n",x,y,joy_x,joy_y);
#endif
				center=1;
			}else if(center){
#ifdef DEBUG_PRINTF
				printf("X/Y : %d %d\n",0,0);
#endif
				center=0;
			}
	}
}	
