%% Referencias:
%%% http://playground.arduino.cc/Interfacing/Matlab
%%% http://www.mathworks.com/matlabcentral/fileexchange/32374-matlab-support-for-arduino--aka-arduinoio-package-
%%% http://www.mathworks.com/help/supportpkg/arduinoio/examples/control-servo-motors.html?prodcode=ML


%tarjeta = arduino('COM5');
servoAttach(tarjeta,4);
pause(1.0)
servoWrite(tarjeta,4,100);
pause(2.0)
servoWrite(tarjeta,4,1);
pause(2.0)
servoWrite(tarjeta,4,179);
pause(2.0)
servoWrite(tarjeta,4,15);
val=servoRead(tarjeta,4)
%delete(tarjeta)