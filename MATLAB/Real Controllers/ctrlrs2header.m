% sos2header, by Prof Garbini, modified by James Muir
function ctrlrs2header(fid, ctrlrs, names, T, Krot, Kvi, Kt, BPRM, BPRL, Rg)
    % Save the single loop controller and other needed values to a (.h) header file.
    % Parameters:
    %    fid          - File indentity
    %    ilc          - Inner Loop Controller
    %    olc          - Outer Loop Controller 
    %    T            - Sample period [s]
    %    Krot         - Rotational Spring Constant [N-m/rad]
    %    Kvi          - Amplifier Constant [A/V]
    %    Kt           - Motor Torque Constant [N-m/A]
    %    BPRM  - Basic Displacement Intervals per Revolution - Motor [BDI/rev]
    %    BPRL  - Basic Displacement Intervals per Revolution - Load [BDI/rev]
    %    Rg           - Gear Ratio
 
    fprintf(fid,'    double         Krot = %f;              // rotational spring constant (N-m/rad)\n',Krot);
    fprintf(fid,'    double         Kt = %f;              // motor torque constant (N-m/A)\n',Kt);
    fprintf(fid,'    double         Kvi = %f;              // amplifier constant (A/V)\n',Kvi);
    fprintf(fid,'    double         BPRM = %f;              // (BDI/rev)\n',BPRM);
    fprintf(fid,'    double         BPRL = %f;              // (BDI/rev)\n',BPRL);
    fprintf(fid,'    double         Rg = %f;              // gear ratio\n',Rg);
    fprintf(fid,'    uint32_t    timeoutValue = %d;      // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
    n = length(ctrlrs);
    for i=1:n
        controller_printer(names{i}, ctrlrs{i}, T, fid)
    end
end

