function controller_printer(name, controller, T, fid)
    % prints controller to file as second order sections
    sos = controller2sos(controller, T);
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

function sos = controller2sos(controller, T)
% discretizes the provided controller into second order sections
    dcontroller = c2d(controller, T);
    [num, den] = tfdata(dcontroller, 'v');
    [sos, g] = tf2sos(num, den);
    sos(1:3) = sos(1:3) * g; % scale numerator by gain
end