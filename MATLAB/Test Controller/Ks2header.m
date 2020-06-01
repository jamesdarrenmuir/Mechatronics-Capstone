  % sos2header, by Prof Garbini, modified by James Muir
  function Ks2header(fid, Kvi, Kt, comment)
  
  % Print the K values to a (.h) header file.
  %
  %   Ks(fid, sos, name, T, comment)
  %
  %    fid      - File indentity
  %    comment  - comment added at top of header
  %    Kvi      - Amplifier Constant [A/V]
  %    Kt       - Motor Torque Constant [N-m/A]
  
%---structure form of cascade

fprintf(fid,'//---%s\n', comment);
fprintf(fid,'//---%s\n', datestr(now,0));