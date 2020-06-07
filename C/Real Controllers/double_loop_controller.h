    double         Krot = 0.462400;              // rotational spring constant (N-m/rad)
    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    double         BPRM = 2000.000000;              // (BDI/rev)
    double         BPRL = 8000.000000;              // (BDI/rev)
    double         Rg = 16.000000;              // gear ratio
    uint32_t    timeoutValue = 2500;      // time interval - us; f_s = 400 Hz
    int         ilc_ns = 1;              // number of sections
    static	struct	biquad ilc[]={   // define the array of floating point biquads
        {4.185550e+02, -3.764789e+02, 0.000000e+00, 1.000000e+00, -4.223360e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
    int         olc_ns = 1;              // number of sections
    static	struct	biquad olc[]={   // define the array of floating point biquads
        {5.828547e-01, -1.163823e+00, 5.809696e-01, 1.000000e+00, -1.908969e+00, 9.089688e-01, 0, 0, 0, 0, 0}
        };
