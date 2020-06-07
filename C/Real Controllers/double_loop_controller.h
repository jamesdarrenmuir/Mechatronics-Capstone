    double         Krot = 0.462400;              // rotational spring constant (N-m/rad)
    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    double         BPRM = 2000.000000;              // (BDI/rev)
    double         BPRL = 8000.000000;              // (BDI/rev)
    double         Rg = 16.000000;              // gear ratio
    uint32_t    timeoutValue = 2500;      // time interval - us; f_s = 400 Hz
    int         ilc_ns = 1;              // number of sections
    static	struct	biquad ilc[]={   // define the array of floating point biquads
        {2.812221e+02, -2.656923e+02, 0.000000e+00, 1.000000e+00, -5.236223e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
    int         olc_ns = 1;              // number of sections
    static	struct	biquad olc[]={   // define the array of floating point biquads
        {6.374122e-01, -1.272486e+00, 6.350759e-01, 1.000000e+00, -1.908969e+00, 9.089688e-01, 0, 0, 0, 0, 0}
        };
