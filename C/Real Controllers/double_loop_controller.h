    double         Krot = 0.462400;              // rotational spring constant (N-m/rad)
    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    double         BDI_per_rev = 2000.000000;              // (BDI/rev)
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    int         inner_loop_controller_ns = 1;              // number of sections
    static	struct	biquad inner_loop_controller[]={   // define the array of floating point biquads
        {1.000000e+00, -9.105210e-01, 0.000000e+00, 1.000000e+00, -3.196563e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
    int         outer_loop_controller_ns = 1;              // number of sections
    static	struct	biquad outer_loop_controller[]={   // define the array of floating point biquads
        {1.000000e+00, -1.995874e+00, 9.958782e-01, 1.000000e+00, -1.896659e+00, 8.966592e-01, 0, 0, 0, 0, 0}
        };
