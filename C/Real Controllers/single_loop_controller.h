    double         Krot = 0.462400;              // rotational spring constant (N-m/rad)
    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    double         BPRM = 2000.000000;              // (BDI/rev)
    double         BPRL = 8000.000000;              // (BDI/rev)
    double         Rg = 16.000000;              // gear ratio
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    int         slc_ns = 1;              // number of sections
    static	struct	biquad slc[]={   // define the array of floating point biquads
        {1.000000e+00, -9.986727e-01, 0.000000e+00, 1.000000e+00, -9.815128e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
