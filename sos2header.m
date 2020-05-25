  function sos2header(fid, sos, name, T, comment)
  
  % Print the filter definition to a (.h) header file.
  %
  %   sos2header(fid, sos, name, T, comment)
  %
  %    fid      - File indentity
  %    sos      - Scaled second order sections, from "tf2sos"
  %    name     - Name to be given to the array of biquad structures, and
  %                  associated with the number of sections.
  %    T        - Sample period in seconds
  %    comment  - comment added at top of header
  
%---structure form of cascade

fprintf(fid,'//---%s\n', comment);
fprintf(fid,'//---%s\n', datestr(now,0));
fprintf(fid,'    char        headerTime[] = "%s";\n',datestr(now,0));
[ns,m]=size(sos);
fprintf(fid,'    int         %s_ns = %d;              // number of sections\n',name,ns);
fprintf(fid,'    uint32_t    timeoutValue = %d;      // time interval - us; f_s = %g Hz\n',T*1e6,1/T);
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

