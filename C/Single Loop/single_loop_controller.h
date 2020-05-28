//---Single loop PDF controller for SEA device
//---28-May-2020 16:33:39
    char        headerTime[] = "28-May-2020 16:33:39";
    int         single_loop_controller_ns = 1;              // number of sections
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad single_loop_controller[]={   // define the array of floating point biquads
        {1.000000e+00, -9.981241e-01, 0.000000e+00, 1.000000e+00, -1.088331e-05, 0.000000e+00, 0, 0, 0, 0, 0}
        };
