% discretize controller
T = 0.005; %(s)
dcontroller = c2d(inner_loop_controller, T);
[num, den] = tfdata(dcontroller, 'v');
[sos, ~] = tf2sos(num, den);
% save controller as .h file
fileID = fopen('../../C/Test Controller/inner_loop_controller.h','w');
sos2header(fileID, sos, "inner_loop_controller", T, "Inner loop PDF controller for SEA device")