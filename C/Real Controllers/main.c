// Single/Double Loop Controller Design Switch
#define SINGLE_LOOP
#define DOUBLE_LOOP

/*
 * Copyright (c) 2015 Prof Garbini
 * Modified 2020 James Muir
 * 
 * Double Loop Controller for SEA
 */

#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include "TimerIRQ.h"
#include "ctable2.h"
#include "Encoder.h"
#include <unistd.h>
#include "matlabfiles.h"
#include "math.h"

#ifdef SINGLE_LOOP
#include "single_loop_controller.h"
#endif /* SINGLE_LOOP */

#ifdef DOUBLE_LOOP
#include "double_loop_controller.h"
#endif /* DOUBLE_LOOP */

#define VDAmax +7.5 // max D/A converter voltage: V
#define VDAmin -7.5 // min D/A converter voltage: V
#define Tmax +0.5   // max motor output torque: N-m
#define Tmin -0.5   // min motor output torque: N-m

#define ntot 5000 // number of data points to save

extern NiFpga_Session myrio_session;

typedef struct
{
    double xfa;
    double v;
    double a;
    double d;
} seg;

typedef struct
{                                 // Resources for the timer thread
    NiFpga_IrqContext irqContext; // IRQ context reserved by Irq_ReserveContext()
    table *a_table;               // table
    seg *profile;                 // profile
    int nseg;                     // number of segments in profile
    NiFpga_Bool irqThreadRdy;     // IRQ thread ready flag
} ThreadResource;

struct biquad
{
    double b0;
    double b1;
    double b2; // numerator
    double a0;
    double a1;
    double a2; // denominator
    double x0;
    double x1;
    double x2; // input
    double y1;
    double y2; // output
};

// Prototypes
double cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax);
double pos(MyRio_Encoder *channel);
double diff(MyRio_Encoder *ch0, MyRio_Encoder *ch0);
void *Timer_Irq_Thread(void *resource);
int Sramps(seg *segs, int nseg, int *iseg, int *itime, double T, double *xa);

/*  This Timer Thread controls the motor and acquires data */
void *Timer_Irq_Thread(void *resource)
{

    ThreadResource *threadResource = (ThreadResource *)resource;
    uint32_t irqAssert = 0;
    MATFILE *mf;
    MyRio_Aio CI0, CO0;
    MyRio_Encoder encC0;
    MyRio_Encoder encC1;
    double PathRef[ntot], Position[ntot], Torque[ntot];
    int isave = 0;
    double T, perror, terror;
    VDAout;
    int j, err;
    double t[ntot];
    double *pref = &((threadResource->a_table + 0)->value); //Convenient pointer names for the table values
    double *pact = &((threadResource->a_table + 1)->value);
    double *VDAmV = &((threadResource->a_table + 2)->value);
    int iseg = -1, itime = -1, nsamp, done;
    seg *mySegs = threadResource->profile;
    int nseg = threadResource->nseg;

    double tout;
    double tact;

    //  Initialize interfaces before allowing IRQ
    AIO_initialize(&CI0, &CO0);                        // initialize analog I/O
    Aio_Write(&CO0, 0.0);                              // stop motor
    conC_Encoder_initialize(myrio_session, &encC0, 0); // initialize encoder 0
    conC_Encoder_initialize(myrio_session, &encC1, 1); // initialize encoder 1

    // printf("timeoutValue %g\n",(double)timeoutValue); // debut output

    while (threadResource->irqThreadRdy)
    {
        T = timeoutValue / 1.e6; // sample period - s (BTI length)
        Irq_Wait(threadResource->irqContext,
                 TIMERIRQNO, // wait for IRQ to assert or signal sent
                 &irqAssert,
                 (NiFpga_Bool *)&(threadResource->irqThreadRdy));
        NiFpga_WriteU32(myrio_session,
                        IRQTIMERWRITE,
                        timeoutValue); /* write timer register */
        NiFpga_WriteBool(myrio_session,
                         IRQTIMERSETTIME,
                         NiFpga_True); /* toggle to reset the timer */

        if (irqAssert)
        {
            // compute the next profile value
            done = Sramps(mySegs,
                          nseg,
                          &iseg,
                          &itime,
                          T,
                          pref); // reference position (revs)
            if (done)
                nsamp = done;

            #ifdef SINGLE_LOOP
            // compute error signal
            *pact = pos(&encC0) / BDI_per_rev.; // current position BDI to (revs)
            error = (*pref - *pact) * 2 * M_PI; // error signal revs to (radians)

            /* compute control signal */
            VDAout = cascade(error,
                             single_loop_controller,
                             single_loop_controller_ns,
                             VDAmin,
                             VDAmax);       // Vda
            *VDAmV = trunc(1000. * VDAout); // table show values
            Aio_Write(&CO0, VDAout);        // output control value
            #endif /* SINGLE_LOOP */

            #ifdef DOUBLE_LOOP
            // outer loop (position control)
            *pact = pos(&encC0) / BDI_per_rev.;  // current position BDI to (revs)
            perror = (*pref - *pact) * 2 * M_PI; // error signal revs to (radians)

            tout = cascade(perror, outer_loop_controller, outer_loop_controller_ns, Tmax, Tmin);

            // inner loop (torque control)
            tact = diff(&encC0, &encC1, BDI_per_rev, BDI_per_rev) * 2 * M_PI * Krot;      // current output torque (N-m)
            terror = (tout - tact); // torque error (N-m)

            VDAout = cascade(terror,
                             inner_loop_controller,
                             inner_loop_controller_ns,
                             VDAmin,
                             VDAmax);       // Vda
            *VDAmV = trunc(1000. * VDAout); // table show values
            Aio_Write(&CO0, VDAout);        // output control value
            #endif /* DOUBLE_LOOP */

            /* save data */
            if (isave < ntot)
            {
                Position[isave] = *pact * 2 * M_PI; // radians
                PathRef[isave] = *pref * 2 * M_PI;  // radians
                Torque[isave] = VDAout * Kt * Kvi;  // N-m	--- NEW AMPLIFIER
                isave++;
            }
            Irq_Acknowledge(irqAssert); /* Acknowledge the IRQ(s) the assertion. */
        }
    }
    Aio_Write(&CO0, 0.0); // stop motor
    // printf("nsamp: %g\n",(double) nsamp); // debug print statement
    //---Save Data to a .mat file in MKS units
    printf("Write MATLAB file\n");
    mf = openmatfile("Lab8.mat", &err);
    if (!mf)
        printf("Can't open mat file error %d\n", err);
    for (j = 0; j < nsamp; j++)
        t[j] = (double)j * T;
    err = matfile_addmatrix(mf, "t", t, nsamp, 1, 0);
    err = matfile_addmatrix(mf, "mySegs", (double *)mySegs, nseg, 4, 0);
    err = matfile_addstring(mf, "headerTime", headerTime);

    err = matfile_addstring(mf, "myName", "Prof. J. Garbini");
    err = matfile_addmatrix(mf, "pathref", PathRef, nsamp, 1, 0);
    err = matfile_addmatrix(mf, "position", Position, nsamp, 1, 0);
    err = matfile_addmatrix(mf, "torque", Torque, nsamp, 1, 0);
    err = matfile_addmatrix(mf, "single_loop_controller", (double *)single_loop_controller, 6, 1, 0); // TODO: make sure 6 is the right size for my controller
    err = matfile_addmatrix(mf, "T", &T, 1, 1, 0);
    matfile_close(mf);

    pthread_exit(NULL); /* Exit the new thread. */
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
#define SATURATE(x, lo, hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))
double cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax)
{

    char i; /* biquad section index */
    double y0;
    struct biquad *f; /* declare a pointer to a variable of type biquad */

    f = fa;   // point to  the first biquad
    y0 = xin; // pass the input to the first biquad in the cascade
    for (i = 0; i < ns; i++)
    {               // loop through the "ns" biquads
        f->x0 = y0; // pass the output to the next biquad
        y0 = (f->b0 * f->x0 + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2) / f->a0;
        if (i == ns - 1)
            y0 = SATURATE(y0, ymin, ymax);
        f->x2 = f->x1; // Update the input history of this biquad
        f->x1 = f->x0;
        f->y2 = f->y1; // Update the output history of this biquad
        f->y1 = y0;
        f++; // point to the next biquad
    }
    return y0; // return the output of the cascade
}

/*--------------------------------------------------------------
 Function pos
	Purpose		Read the encoder counter, compute the current
			estimate of the motor position.
	Parameters:	encoder channel
	Returns: 	encoder position (BDI)
*--------------------------------------------------------------*/
double pos(MyRio_Encoder *channel)
{
    int currentP; // current position (sizeof(int) = 4 bytes

    static int startP;
    static int first = 1; // first time calling pos();

    int deltaP;
    double position; // position estimate

    currentP = Encoder_Counter(channel);
    // initialization
    if (first)
    {
        startP = currentP;
        first = 0;
    };

    deltaP = currentP - startP;
    position = (double)deltaP; // BDI - displacement from starting position
    return position;
}

/*--------------------------------------------------------------
 Function diff
	Purpose		Return the difference between the angles of the two shafts
	Parameters:	
        ch0 - encoder channel 0
        ch1 - encoder channel 1
        tpr0 - ticks per revolution for encoder 0
        tpr1 - ticks per revolution for encoder 1
	Returns: 	encoder position (BDI)
*--------------------------------------------------------------*/
double diff(MyRio_Encoder *ch0, MyRio_Encoder *ch1, double tpr0, double tpr1)
{
    return ((pos(ch1) / tpr1) - (pos(ch0) / tpr0)) // (rev)
}

int main(int argc, char **argv)
{
    int32_t status;
    MyRio_IrqTimer irqTimer0;
    ThreadResource irqThread0;
    pthread_t thread;
    uint32_t timeoutValue; /* time interval - us */
    double bti = 0.5;
    double vmax, amax, dwell;
    int nseg;

    char *Table_Title = "Position Controller";
    table my_table[] = {
        {"X_ref: rev  ", 0, 0.0},
        {"X_act: rev  ", 0, 0.0},
        {"VDAout: mV  ", 0, 0.0}};

    vmax = 10.;
    amax = 10.;
    dwell = 5.0;
    seg mySegs[4] = {// revolutions
                     {1.0, vmax, amax, dwell},
                     {0.0, vmax, amax, dwell},
                     {-1.0, vmax, amax, dwell},
                     {0.0, vmax, amax, dwell}};
    nseg = 4;

    /*  registers corresponding to the IRQ channel     */
    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;
    timeoutValue = bti * 1000.;

    /* Open the myRIO NiFpga Session. */
    status = MyRio_Open();
    if (MyRio_IsNotSuccess(status))
    {
        return status;
    }

    /* Configure the timer IRQ. */
    status = Irq_RegisterTimerIrq(&irqTimer0,
                                  &irqThread0.irqContext,
                                  timeoutValue);
    if (status != NiMyrio_Status_Success)
    { /* Terminate the process if it is unsuccessful */
        printf("Status: %d, Configuration of Timer IRQ failed.", status);
        return status;
    }

    /* Create new thread to catch the timer IRQ */
    irqThread0.irqThreadRdy = NiFpga_True; /* Set the indicator to allow the new thread.*/
    irqThread0.a_table = my_table;
    irqThread0.profile = mySegs;
    irqThread0.nseg = nseg;
    status = pthread_create(&thread,
                            NULL,
                            Timer_Irq_Thread, // name of timer thread
                            &irqThread0);     // thread resource
    if (status != NiMyrio_Status_Success)
    {
        printf("Status: %d, Failed to create irq thread!", status);
        return status;
    }

    ctable2(Table_Title, my_table, 3); // start the table editor

    //	All Done.  Terminate Timer Thread
    irqThread0.irqThreadRdy = NiFpga_False; /* Set  indicator to end the timer thread.*/
    pthread_join(thread, NULL);             /* Wait for the end of the IRQ thread. */
    printf("Timer thread ends.\n");
    printf_lcd("Timer thread ends.\n");

    // Disable timer interrupt, so you can configure this I/O next time.
    status = Irq_UnregisterTimerIrq(&irqTimer0,
                                    irqThread0.irqContext);
    if (status != NiMyrio_Status_Success)
    {
        printf("Status: %d\nClear Timer IRQ failed.\n", status);
        return status;
    }
    status = MyRio_Close(); // Close the myRIO NiFpga Session.
    return status;          // Returns 0 if successful.
}
