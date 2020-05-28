//---Outer loop PDF controller for SEA device
//---28-May-2020 13:21:45
    char        headerTime[] = "28-May-2020 13:21:45";
    int         outer_loop_controller_ns = 1;              // number of sections
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad outer_loop_controller[]={   // define the array of floating point biquads
        {1.000000e+00, -1.996103e+00, 9.961071e-01, 1.000000e+00, -1.896659e+00, 8.966592e-01, 0, 0, 0, 0, 0}
        };
