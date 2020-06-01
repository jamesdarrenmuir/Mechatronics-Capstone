    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    double         BDI_per_rev = 2000.000000;              // (BDI/rev)
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    int         single_loop_controller_ns = 1;              // number of sections
    static	struct	biquad single_loop_controller[]={   // define the array of floating point biquads
        {1.000000e+00, -9.986727e-01, 0.000000e+00, 1.000000e+00, -9.815128e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
