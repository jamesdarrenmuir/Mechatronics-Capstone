% sos2header, by Prof Garbini, modified by James Muir
function dlc2header(fid, ilc, olc, T, Krot, Kvi, Kt, BDI_per_rev)
    % Save the single loop controller and other needed values to a (.h) header file.
    % Parameters:
    %    fid          - File indentity
    %    ilc          - Inner Loop Controller
    %    olc          - Outer Loop Controller 
    %    T            - Sample period [s]
    %    Krot         - Rotational Spring Constant [N-m/rad]
    %    Kvi          - Amplifier Constant [A/V]
    %    Kt           - Motor Torque Constant [N-m/A]
    %    BDI_per_rev  - Basic Displacement Intervals per Revolution [BDI/rev]
 
    fprintf(fid,'    double         Krot = %f;              // rotational spring constant (N-m/rad)\n',Krot);
    fprintf(fid,'    double         Kt = %f;              // motor torque constant (N-m/A)\n',Kt);
    fprintf(fid,'    double         Kvi = %f;              // amplifier constant (A/V)\n',Kvi);
    fprintf(fid,'    double         BDI_per_rev = %f;              // (BDI/rev)\n',BDI_per_rev);
    fprintf(fid,'    uint32_t    timeoutValue = %d;      // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
    controller_printer("inner_loop_controller", ilc, T, fid)
    controller_printer("outer_loop_controller", olc, T, fid)
end

