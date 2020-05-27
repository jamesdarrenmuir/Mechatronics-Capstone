/*
 * Copyright (c) 2015 Prof Garbini
 * Modified 2020 James Muir
 */

// Based on Lab #8 R&D

#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include "TimerIRQ.h"
#include "ctable2.h"
#include "Encoder.h"
#include <unistd.h>
#include "matlabfiles.h"
#include "math.h"

//#define M_PI	3.14159265358979323846
// Kvi and Kt are now defined in single_loop_controller.h
// #define Kvi		0.41		// Amplifier gain:  A/V
// #define Kt		0.11		// DC motor torque constant:  N-m/A
#define VDAmax	+7.5		// max D/A converter voltage: V
#define VDAmin	-7.5		// min D/A converter voltage: V
#define	ntot	5000

extern NiFpga_Session myrio_session;

typedef struct {double xfa; double v; double a; double d;} seg;
typedef struct	{						// Resources for the timer thread
    NiFpga_IrqContext	irqContext;		// IRQ context reserved by Irq_ReserveContext()
    table 				*a_table;		// table
    seg					*profile;		// profile
    int					nseg;			// number of segments in profile
    NiFpga_Bool irqThreadRdy;           // IRQ thread ready flag
} ThreadResource;

struct	biquad	{
	double  b0; double  b1; double  b2;		// numerator
	double  a0; double  a1; double  a2;		// denominator
	double  x0; double  x1; double  x2;		// input
	double  y1; double  y2;					// output
};
#include "single_loop_controller.h"

// Prototypes
double	cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax );
double	pos(MyRio_Encoder *channel, int *startP);
void 	*Timer_Irq_Thread(void* resource);
int  	Sramps(seg *segs, int nseg, int *iseg, int *itime, double T, double *xa);


void *Timer_Irq_Thread(void* resource)
{
/*  This Timer Thread controls the motor and acquires data */

    ThreadResource* threadResource = (ThreadResource*) resource;
    uint32_t 	irqAssert = 0;
    MATFILE 	*mf;
    MyRio_Aio 	CI0, CO0;
    MyRio_Encoder 	encC0;
    double	PathRef[ntot], Position[ntot], Torque[ntot];
    int 	isave=0;
    double 	T, error, VDAout;
    int		j, err, startP;
    double	t[ntot];
    double	*pref 	= &((threadResource->a_table+0)->value); //Convenient pointer names for the table values
    double	*pact 	= &((threadResource->a_table+1)->value);
    double	*VDAmV 	= &((threadResource->a_table+2)->value);
    int		iseg=-1, itime=-1, nsamp, done;
    seg		*mySegs = threadResource->profile;
    int		nseg = threadResource->nseg;

    //  Initialize interfaces before allowing IRQ
    	AIO_initialize(&CI0, &CO0);						// initialize analog I/O
    	Aio_Write(&CO0, 0.0);							// stop motor
    	EncoderC_initialize( myrio_session, &encC0);	// initialize encoder

    printf("timeoutValue %g\n",(double)timeoutValue);

   	while (threadResource->irqThreadRdy)    {
		T = timeoutValue/1.e6;		// sample period - s (BTI length)
        Irq_Wait(	threadResource->irqContext,
        			TIMERIRQNO,  	// wait for IRQ to assert or signal sent
        			&irqAssert,
        			(NiFpga_Bool*) &(threadResource->irqThreadRdy));
        NiFpga_WriteU32(	myrio_session,
        					IRQTIMERWRITE,
        					timeoutValue);	/* write timer register */
        NiFpga_WriteBool(	myrio_session,
        					IRQTIMERSETTIME,
        					NiFpga_True);	/* toggle to reset the timer */

        if(irqAssert) {
        	// compute the next profile value
			done = Sramps(	mySegs,
							nseg,
							&iseg,
							&itime,
							T,
							pref); // reference position (revs)
			if(done) nsamp = done;

			// compute error signal
			*pact = pos(&encC0, &startP)/2000.;	// current position (revs)
			error = (*pref - *pact)*2*M_PI;		// error signal (radians)

			/* compute control signal */
        	VDAout = cascade(	error,
        						PIDF,
        						PIDF_ns,
        						VDAmin,
        						VDAmax );		// Vda
			*VDAmV = trunc(1000.*VDAout);		// table show values
			Aio_Write(&CO0, VDAout);			// output control value

			/* save data */
			if ( isave < ntot ) {
				Position[isave] = *pact*2*M_PI;		// radians
				PathRef[isave]	= *pref*2*M_PI;		// radians
				Torque[isave] 	= VDAout*Kt*Kvi;	// N-m	--- NEW AMPLIFIER
				isave++;
			}
			Irq_Acknowledge(irqAssert);	 /* Acknowledge the IRQ(s) the assertion. */
        }
    }
	Aio_Write(&CO0, 0.0);						// stop motor
    printf("nsamp: %g\n",(double) nsamp);
    //---Save Data to a .mat file in MKS units
   	printf("Write MATLAB file\n");
    mf = openmatfile("Lab8.mat", &err);
    if(!mf)   printf("Can't open mat file error %d\n", err);
    for (j=0; j<nsamp; j++) t[j]=(double)j*T;
    err = matfile_addmatrix(mf, "t", 		t, 					nsamp, 	1,	0);
    err = matfile_addmatrix(mf, "mySegs",	(double *) mySegs,	nseg,	4,	0);
    err = matfile_addstring(mf, "headerTime", 	headerTime);

    err = matfile_addstring(mf, "myName", 	"Prof. J. Garbini");
    err = matfile_addmatrix(mf, "pathref",	PathRef,			nsamp,	1,	0);
    err = matfile_addmatrix(mf, "position",	Position,			nsamp,	1,	0);
    err = matfile_addmatrix(mf, "torque",	Torque,				nsamp,	1,	0);
    err = matfile_addmatrix(mf, "pidf",		(double *) PIDF,	6,		1,	0);
    err = matfile_addmatrix(mf, "T",		&T,					1,		1,	0);
    matfile_close(mf);

    pthread_exit(NULL);	 /* Exit the new thread. */
    return NULL;
}
/*--------------------------------------------------------------
 Function cascade
 Purpose:		implements cascade of biquad sections
 Parameters:
 xin -			current input to the cascade
 fa -			the  array of type biquad, each element
 	 	 	 	contains the filter coefficients, input
 	 	 	 	and output history variables
 ns -			number of biquad sections
 ymin -			minimum output saturation limit
 ymax -			maximum output saturation limit
 Returns:		Current value, y0, of the final biquad
 *--------------------------------------------------------------*/
#define SATURATE(x,lo,hi)	((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))

double   cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax )
{

	char i;   /* biquad section index */
	double y0;
	struct	biquad *f;	/* declare a pointer to a variable of type biquad */

	f=fa;						// point to  the first biquad
	y0 = xin;					// pass the input to the first biquad in the cascade
	for (i=0; i<ns; i++) {		// loop through the "ns" biquads
		f->x0 = y0;				// pass the output to the next biquad
		y0 = ( f->b0*f->x0 + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2 )/f->a0;
		if (i == ns-1) y0 = SATURATE(y0, ymin, ymax);
		f->x2 = f->x1;			// Update the input history of this biquad
		f->x1 = f->x0;
		f->y2 = f->y1;			// Update the output history of this biquad
		f->y1 = y0;
		f++;					// point to the next biquad
	}
	return y0;					// return the output of the cascade
}
/*--------------------------------------------------------------
 Function pos
	Purpose		Read the encoder counter, compute the current
			estimate of the motor velocity.
	Parameters:	none
	Returns: 	encoder speed (BDI/BTI)
*--------------------------------------------------------------*/
double	pos(MyRio_Encoder *channel, int *startP) {
	static	int	deltaP;
	static	int currentP;		// current position (sizeof(int) = 4 bytes
	double	position;			// speed estimate
	static  int		first = 1;			// first time calling vel();

	currentP = Encoder_Counter(channel);
	if(first)  {
		*startP = currentP;
		first = 0;
	};
	deltaP = currentP - *startP;
	position = (double)deltaP;			// BDI - displacement from starting position
	return	position;
}

int  Sramps(seg *segs, int nseg, int *iseg, int *itime, double T, double *xa)
{
    // Computes the next position, *xa, of a uniform sampled position profile.
    // The profile is composed of an array of segments (type: seg)
    // Each segment consists of:
    //      xfa:    final position
    //      v:      maximum velocity
    //      a:      maximum acceleration
    //      d:      dwell time at the final position
    //  Called from a loop, the profile proceeds from the current position,
    //  through each segment in turn, and then repeats.
    // Inputs:
    //  seg *segs:  - segments array
    //  int nseg:   - number of segments in the profile
    //  int *iseg:  - variable hold segment index
    //  int *itime  - time index within a segment (= -1 at segment beginning)
    //  double T:   - time increment
    // Outputs:
    //  double *xa: - next position in profile
    // Returns:     n - number of samples in the profile, 0 otherwise
	//
	//  Call with *itime = -1, *iseg = -1, outside the loop to initialize.

    double t, t1=0, t2=1, tf=1, tramp, x1=1, xramp, xfr=1, xr, d;
    static double x0, dir;
    static int ntot;
    double vmax=1, amax=1;
    int n;

    if (*itime==-1) {
        (*iseg)++;
        if(*iseg==nseg) {
        	*iseg=0;
        	ntot = 0;
        }
        *itime=0;
        x0=*xa;
    }
    vmax=segs[*iseg].v;
    amax=segs[*iseg].a;
    d=segs[*iseg].d;
    xfr=segs[*iseg].xfa-x0;
    dir=1.0;
    if(xfr<0){
        dir=-1.;
        xfr=-xfr;
    }
    t1 = vmax/amax;
    x1 = 1./2.*amax*t1*t1;
    if (x1<xfr/2) {
        xramp = xfr-2.*x1;
        tramp = xramp/vmax;
        t2 = t1+tramp;
        tf = t2+t1;
    } else {
        x1 = xfr/2;
        t1 = sqrt(2*x1/amax);
        t2 = t1;
        tf = 2.*t1;
    }
    n = trunc((tf+d)/T)+1;

    t = *itime*T;
    if(t<t1) {
        xr = 1./2.*amax*t*t;
    } else if (t>=t1 && t<t2) {
        xr = x1+vmax*(t-t1);
    } else if (t>=t2 && t<tf) {
        xr = xfr-1./2.*amax*(tf-t)*(tf-t);
    } else {
        xr = xfr;
    }
    *xa=x0+dir*xr;
    (*itime)++;
    if(*itime==n+1) {
    	ntot = ntot + *itime - 1;
        *itime=-1;
        if(*iseg==nseg-1) {
        	return ntot;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    int32_t status;
    MyRio_IrqTimer	irqTimer0;
    ThreadResource	irqThread0;
    pthread_t 		thread;
	// timeoutValue is now defined in single_loop_controller.h
    // uint32_t 		timeoutValue;	/* time interval - us */
    // double			bti=0.5;
    double			vmax, amax, dwell;
    int				nseg;

    char *Table_Title ="Position Controller";
    table my_table[] = {
		{"X_ref: rev  ",  0, 0.0     },
		{"X_act: rev  ",  0, 0.0     },
		{"VDAout: mV  ",  0, 0.0     }
    };

    vmax=50.;
    amax=20.;
    dwell=1.0;
    seg mySegs[8]={				// revolutions
		{10.125,	vmax,	amax,	dwell},
		{20.250,	vmax,	amax,	dwell},
		{30.375,	vmax,	amax,	dwell},
		{40.500,	vmax,	amax,	dwell},
		{30.625,	vmax,	amax,	dwell},
		{20.750,	vmax,	amax,	dwell},
		{10.875,	vmax,	amax,	dwell},
		{ 0.000,	vmax,	amax,	dwell}
    };
    nseg = 8;

    /*  registers corresponding to the IRQ channel     */
    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;
    // timeoutValue = bti*1000.;

    /* Open the myRIO NiFpga Session. */
    status = MyRio_Open();
    if (MyRio_IsNotSuccess(status)){ return status; }

    /* Configure the timer IRQ. */
    status = Irq_RegisterTimerIrq(	&irqTimer0,
    								&irqThread0.irqContext,
    								timeoutValue);
    if (status != NiMyrio_Status_Success)  {	/* Terminate the process if it is unsuccessful */
        printf("Status: %d, Configuration of Timer IRQ failed.", status);
        return status;
    }

    /* Create new thread to catch the timer IRQ */
    irqThread0.irqThreadRdy = NiFpga_True;	 /* Set the indicator to allow the new thread.*/
    irqThread0.a_table = my_table;
    irqThread0.profile = mySegs;
    irqThread0.nseg = nseg;
    status = pthread_create(	&thread,
    							NULL,
    							Timer_Irq_Thread,	// name of timer thread
    							&irqThread0);		// thread resource
    if (status != NiMyrio_Status_Success)   {
        printf("Status: %d, Failed to create irq thread!",  status);
        return status;
    }
//
    ctable2(Table_Title, my_table, 3);			// start the table editor
//
//	All Done.  Terminate Timer Thread
	irqThread0.irqThreadRdy = NiFpga_False;		/* Set  indicator to end the timer thread.*/
    pthread_join(thread, NULL);					/* Wait for the end of the IRQ thread. */
	printf("Timer thread ends.\n");	printf_lcd("Timer thread ends.\n");

    // Disable timer interrupt, so you can configure this I/O next time.
    status = Irq_UnregisterTimerIrq(	&irqTimer0,
    									irqThread0.irqContext);
    if (status != NiMyrio_Status_Success)	{
        printf("Status: %d\nClear Timer IRQ failed.\n", status);
        return status;
    }
       status = MyRio_Close();	// Close the myRIO NiFpga Session.
    return status;				// Returns 0 if successful.
}

