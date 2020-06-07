    double         Krot = 0.462400;              // rotational spring constant (N-m/rad)
    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    double         BPRM = 2000.000000;              // (BDI/rev)
    double         BPRL = 8000.000000;              // (BDI/rev)
    double         Rg = 16.000000;              // gear ratio
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    int         slc_ns = 1;              // number of sections
    static	struct	biquad slc[]={   // define the array of floating point biquads
        {6.833623e-02, -6.824552e-02, 0.000000e+00, 6.833623e-02, -6.707288e-02, 0.000000e+00, 0, 0, 0, 0, 0}
        };
