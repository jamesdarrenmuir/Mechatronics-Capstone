//---PIDF position control, wc = 50.2655 r/s
//---13-Apr-2020 12:46:24
    char        headerTime[] = "13-Apr-2020 12:46:24";
    int         PIDF_ns = 1;              // number of sections
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad PIDF[]={   // define the array of floating point biquads
        {4.620281e+01, -9.135472e+01, 4.515400e+01, 1.000000e+00, -1.492420e+00, 4.924205e-01, 0, 0, 0, 0, 0}
        };
