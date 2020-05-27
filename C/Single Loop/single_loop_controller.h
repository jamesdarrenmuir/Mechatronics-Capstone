//---Single loop PDF controller for SEA device
//---27-May-2020 00:01:44
    char        headerTime[] = "27-May-2020 00:01:44";
    int         single_loop_controller_ns = 1;              // number of sections
    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad single_loop_controller[]={   // define the array of floating point biquads
        {1.000000e+00, -9.986727e-01, 0.000000e+00, 1.000000e+00, -9.815128e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
