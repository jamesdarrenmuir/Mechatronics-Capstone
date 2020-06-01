//---Single loop PDF controller for SEA device
//---31-May-2020 18:14:57
    char        headerTime[] = "31-May-2020 18:14:57";
    int         single_loop_controller_ns = 1;              // number of sections
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad single_loop_controller[]={   // define the array of floating point biquads
        {1.000000e+00, -9.982318e-01, 0.000000e+00, 1.000000e+00, -5.743681e-02, 0.000000e+00, 0, 0, 0, 0, 0}
        };
