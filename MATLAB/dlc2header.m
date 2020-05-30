% sos2header, by Prof Garbini, modified by James Muir
function dlc2header(fid, ilc, olc, T, Krot, Kvi, Kt, BDI_per_rev)
    % Save the single loop controller and other needed values to a (.h) header file.
    % Parameters:
    %    fid          - File indentity
    %    Scaled second order sections, from "tf2sos"
    %     ilc          - Inner Loop Controller
    %     olc          - Outer Loop Controller 
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
    sos_printer("inner_loop_controller", ilc, fid)
    sos_printer("outer_loop_controller", olc, fid)
end

function sos_printer(name, sos, fid)
    [ns,~]=size(sos);
    fprintf(fid,'    int         %s_ns = %d;              // number of sections\n',name,ns);
    fprintf(fid,'    static\tstruct\tbiquad %s[]={   // define the array of floating point biquads\n',name);
    for i=1:ns-1
        fprintf(fid,'        {');
        for j=[1,2,3,4,5,6]
            fprintf(fid,'%e, ',sos(i,j));
        end
        fprintf(fid,'0, 0, 0, 0, 0},\n');
    end
        fprintf(fid,'        {');
        for j=[1,2,3,4,5,6]
            fprintf(fid,'%e, ',sos(ns,j));
        end
        fprintf(fid,'0, 0, 0, 0, 0}\n        };\n');
end

