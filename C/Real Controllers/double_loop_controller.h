    double         Krot = 0.462400;              // rotational spring constant (N-m/rad)
    double         Kt = 0.021400;              // motor torque constant (N-m/A)
    double         Kvi = 0.410000;              // amplifier constant (A/V)
    double         BPRM = 2000.000000;              // (BDI/rev)
    double         BPRL = 8000.000000;              // (BDI/rev)
    double         Rg = 16.000000;              // gear ratio
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    int         ilc_ns = 1;              // number of sections
    static	struct	biquad ilc[]={   // define the array of floating point biquads
        {1.396487e+02, -1.271400e+02, 0.000000e+00, 1.000000e+00, -3.204750e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
    int         olc_ns = 1;              // number of sections
    static	struct	biquad olc[]={   // define the array of floating point biquads
        {2.391356e-01, -4.772851e-01, 2.381505e-01, 1.000000e+00, -1.896659e+00, 8.966592e-01, 0, 0, 0, 0, 0}
        };
