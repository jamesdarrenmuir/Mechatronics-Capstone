// Single/Double Loop Controller Design Switch

// #define SINGLE_LOOP
#define DOUBLE_LOOP
// #define TORQUE

#define LOGGING

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
#ifdef LOGGING
#include "matlabfiles.h"
#endif /* LOGGING */
#include "math.h"

// Absolute limits from Prof. Garbini
// #define VDAmax +7.5 // max D/A converter voltage: V
// #define VDAmin -7.5 // min D/A converter voltage: V
// Our limits to avoid stretching fishing line
#define VDAmax +5.0 // max D/A converter voltage: V
#define VDAmin -5.0 // min D/A converter voltage: V
#ifdef DOUBLE_LOOP
#define Tmax +0.5   // max motor output torque: N-m
#define Tmin -0.5   // min motor output torque: N-m
#endif /* DOUBLE_LOOP */

#ifdef LOGGING
#define ntot 3000 // number of data points to save
#endif /* LOGGING */

extern NiFpga_Session myrio_session;

typedef struct
{  // segment for position profile
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
{ // second-order section
    double b0; double b1; double b2; // numerator
    double a0; double a1; double a2; // denominator
    double x0; double x1; double x2; // input
    double y1; double y2; // output
};

#ifdef SINGLE_LOOP
#include "single_loop_controller.h"
#endif /* SINGLE_LOOP */

#ifndef SINGLE_LOOP
#include "double_loop_controller.h"
#endif /* !SINGLE_LOOP */

// Prototypes
void *Timer_Irq_Thread(void *resource);
double cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax);
double pos(MyRio_Encoder *channel, int *startP);
int Sramps(seg *segs, int nseg, int *iseg, int *itime, double T, double *xa);
NiFpga_Status conC_Encoder_initialize(NiFpga_Session myrio_session, MyRio_Encoder *encCp, int iE);


/*  This Timer Thread controls the motor and acquires data */
void *Timer_Irq_Thread(void *resource)
{

    ThreadResource *threadResource = (ThreadResource *)resource;
    uint32_t irqAssert = 0;
    #ifdef LOGGING
    MATFILE *mf;
    int j, err, isave = 0, nsave;
    double P2Ref[ntot], P2Act[ntot], TMG[ntot], P1Act[ntot], TsRef[ntot], TsAct[ntot], t[ntot]; // time vector
    #endif /* LOGGING */
    MyRio_Aio CI0, CO0;
    MyRio_Encoder encC1, encC2;
    int startP1, startP2;

    double VDAout;

    double *P2_ref = &((threadResource->a_table + 0)->value); //Convenient pointer names for the table values
    double *P2_act = &((threadResource->a_table + 1)->value);
    double *VDA_out_mV = &((threadResource->a_table + 2)->value); // mV
    double *P1_act = &((threadResource->a_table + 3)->value);
    double *Ts_act = &((threadResource->a_table + 4)->value);
    double *Ts_ref = &((threadResource->a_table + 5)->value);

    int iseg = -1, itime = -1, nsamp, done;
    seg *mySegs = threadResource->profile;
    int nseg = threadResource->nseg;

    double T; // sample period (s)
    #ifndef TORQUE
    double P2_err; // output position error (rad)
    #endif /* !TORQUE */

    #ifndef SINGLE_LOOP
    double Ts_err; // torque error (N-m)
    #endif /* !SINGLE_LOOP */

    //  Initialize interfaces before allowing IRQ
    AIO_initialize(&CI0, &CO0);                        // initialize analog I/O
    Aio_Write(&CO0, 0.0);                              // stop motor
    conC_Encoder_initialize(myrio_session, &encC1, 0); // initialize encoder 0
    conC_Encoder_initialize(myrio_session, &encC2, 1); // initialize encoder 1

    printf("timeoutValue %g\n",(double)timeoutValue); // debug output

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
            #ifndef TORQUE
            // compute the next position profile value
            done = Sramps(mySegs,
                          nseg,
                          &iseg,
                          &itime,
                          T,
                          P2_ref); // reference position (revs)
            #endif /* !TORQUE */

            #ifdef TORQUE
            // compute the next torque profile value
            done = Sramps(mySegs,
                          nseg,
                          &iseg,
                          &itime,
                          T,
                          Ts_ref); // reference torque (N-m)
            #endif /* TORQUE */

            if (done)
                nsamp = done;
                // printf("nsamp (inside timer thread): %g\n",(double) nsamp); //DEBUG

            // current positions
            *P2_act = pos(&encC2, &startP2) / BPRL;  // current position BDI to (revs)
            *P1_act = pos(&encC1, &startP1) / BPRM / Rg;  // current position BDI to (revs)
          
            // current output torque (N-m)
            *Ts_act = (*P1_act - *P2_act) * 2 * M_PI * Krot;

            #ifndef TORQUE
             // compute position error
            P2_err = (*P2_ref - *P2_act) * 2 * M_PI; // error signal revs to (rad)
            #endif /* !TORQUE */

            #ifdef DOUBLE_LOOP
            // outer loop (position control)
            *Ts_ref = cascade(P2_err, olc, olc_ns, Tmin, Tmax);
            #endif /* DOUBLE_LOOP */

            #ifndef SINGLE_LOOP
            Ts_err = (*Ts_ref - *Ts_act); // torque error (N-m)
            #endif /* !SINGLE_LOOP */
            
            #ifdef SINGLE_LOOP
            /* compute control signal */
            VDAout = cascade(P2_err,
                             slc,
                             slc_ns,
                             VDAmin,
                             VDAmax);       // Vda
            #endif /* SINGLE_LOOP */

            #ifndef SINGLE_LOOP
            // inner loop (torque control)
            VDAout = cascade(Ts_err,
                             ilc,
                             ilc_ns,
                             VDAmin,
                             VDAmax);       // Vda
            #endif /* !SINGLE_LOOP */
            
            *VDA_out_mV = trunc(1000. * VDAout); // table show values
            Aio_Write(&CO0, VDAout);        // output control value

            #ifdef LOGGING
            /* save data */
            if (isave < ntot)
            {
                P2Act[isave] = *P2_act * 2 * M_PI; // radians
                P2Ref[isave] = *P2_ref * 2 * M_PI;  // radians
                TMG[isave] = VDAout * Kt * Kvi * Rg;  // N-m
                P1Act[isave] = *P1_act * 2 * M_PI; // rad
                TsAct[isave] = *Ts_act; // N-m
                TsRef[isave] = *Ts_ref; // N-m
                isave++;
            }
            #endif /* LOGGING */

            Irq_Acknowledge(irqAssert); /* Acknowledge the IRQ(s) the assertion. */
        }
    }
    Aio_Write(&CO0, 0.0); // stop motor
    printf("nsamp: %g\n",(double) nsamp); // debug print statement
    #ifdef LOGGING
    nsave = (nsamp < ntot) ? nsamp : ntot; // minimum
    printf("nsave: %g\n",(double) nsave); // debug print statement
    //---Save Data to a .mat file in MKS units
    printf("Write MATLAB file\n");
    mf = openmatfile("SEA.mat", &err);
    if (!mf)
        printf("Can't open mat file error %d\n", err);
    for (j = 0; j < nsave; j++)
        t[j] = (double)j * T;
    err = matfile_addmatrix(mf, "time", t, nsave, 1, 0);
    err = matfile_addmatrix(mf, "controller_segments", (double *)mySegs, nseg, 4, 0);

    err = matfile_addstring(mf, "name", "SEA Team");
    err = matfile_addmatrix(mf, "reference_position", P2Ref, nsave, 1, 0);
    err = matfile_addmatrix(mf, "actual_position", P2Act, nsave, 1, 0);
    err = matfile_addmatrix(mf, "motor_torque", TMG, nsave, 1, 0);
    err = matfile_addmatrix(mf, "motor_position", P1Act, nsave, 1, 0);
    err = matfile_addmatrix(mf, "actual_spring_torque", TsAct, nsave, 1, 0);
    err = matfile_addmatrix(mf, "reference_spring_torque", TsRef, nsave, 1, 0);
    err = matfile_addmatrix(mf, "T", &T, 1, 1, 0);
    #ifdef SINGLE_LOOP
    err = matfile_addmatrix(mf, "slc", (double *)slc, 6, 1, 0); // TODO: make sure 6 is the right size for my controller
    #endif /* SINGLE_LOOP */
    #ifndef SINGLE_LOOP
    err = matfile_addmatrix(mf, "ilc", (double *)ilc, 6, 1, 0); // TODO: make sure 6 is the right size for my controller
    err = matfile_addmatrix(mf, "olc", (double *)olc, 6, 1, 0); // TODO: make sure 6 is the right size for my controller
    #endif /* !SINGLE_LOOP */
    matfile_close(mf);
    #endif /* LOGGING */

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
	Parameters:
        - channel: the encoder
        - startP: the starting position of the encoder
	Returns: 	encoder position (BDI)
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

int main(int argc, char **argv)
{
    int32_t status;
    MyRio_IrqTimer irqTimer0;
    ThreadResource irqThread0;
    pthread_t thread;
    double vmax, amax, dwell;
    int nseg;

    char *Table_Title = "Position Controller";
 
    table my_table[] = {
        {"P2_ref: rev  ", 0, 0.0}, // output pulley reference position
        {"P2_act: rev  ", 0, 0.0}, // output pulley actual position
        {"VDA_out: mV  ", 0, 0.0}, // myRIO output voltage
        {"P1_act: rev  ", 0, 0.0}, // motor pulley actual position
        {"Ts_act: N-m  ", 0, 0.0}  // spring actual torque
        #ifndef SINGLE_LOOP
        ,{"Ts_ref: N-m  ", 0, 0.0} // spring reference torque
        #endif /* !SINGLE_LOOP */
        };
    #ifdef SINGLE_LOOP
    int table_entries = 5;
    #endif /* SINGLE_LOOP */
    #ifndef SINGLE_LOOP
    int table_entries = 6;
    #endif /* !SINGLE_LOOP */

    #ifndef TORQUE
    vmax = 0.1; // (rev/s)
    amax = 1.0; // (rev/s^2)
    dwell = 4.0; // (s)
    seg mySegs[4] = {// (rev)
                     {0.15, vmax, amax, dwell},
                     {0.0, vmax, amax, dwell},
                     {-0.15, vmax, amax, dwell},
                     {0.0, vmax, amax, dwell}};
    nseg = 4;
    #endif /* !TORQUE */

    #ifdef TORQUE
    vmax = 0.25; // (N-m/s)
    amax = 2.; // (N-m/s^2)
    dwell = 1.0; // (s)
    seg mySegs[4] = {// (N-m)
                     {0.25, vmax, amax, dwell},
                     {0.0, vmax, amax, dwell},
                     {-0.25, vmax, amax, dwell},
                     {0.0, vmax, amax, dwell}};
    nseg = 4;
    #endif /* TORQUE */

    /*  registers corresponding to the IRQ channel     */
    irqTimer0.timerWrite = IRQTIMERWRITE;
    irqTimer0.timerSet = IRQTIMERSETTIME;

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

    ctable2(Table_Title, my_table, table_entries); // start the table editor

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
