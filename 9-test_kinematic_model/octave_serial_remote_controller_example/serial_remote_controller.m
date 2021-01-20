clc
clear all

 pkg load control
 pkg load mapping
 pkg load instrument-control
 pkg load optim
 pkg load signal
 pkg load communications



%%Configurable parameters
%COMPORT = "COM13";
COMPORT = "/dev/ttyACM0";
COMPORT2 = "/dev/ttyUSB0";
BAUD_RATE = 115200;
STOP_DATASTREAM = '0';
START_FAST_DATASTREAM = '2';
START_SLOW_DATASTREAM = '1';
START_BYTE = 'S';
END_BYTE = 'A';


display('Starting execution...')


%%%%For windows systems
%%if sum(COMPORT) > sum("COM9")%%%%"\\\\.\\COM11"
%%    COMPORT = ["\\\\.\\" COMPORT];
%%if sum(COMPORT2) > sum("COM9")%%%%"\\\\.\\COM11"
%%    COMPORT2 = ["\\\\.\\" COMPORT2];
%%end

%%%%Open serial port
try
  shandler = serial(COMPORT, BAUD_RATE);
catch
  try
    shandler = serial(COMPORT2, BAUD_RATE);
  catch
    display('Error, serial port not available')
    return
  end
end


pause(2)
%%%%Stop data stream and flush input and output buffers
srl_fwrite (shandler, STOP_DATASTREAM, "uint8");
srl_flush(shandler);
pause(2)



%%%%Sends comand to serial device
[nbytesent] = srl_fwrite (shandler, START_SLOW_DATASTREAM, "uint8");
Counter = 0;
Byte = 0;


while 1
  
   %Readingdata from serial port
   if get(shandler,'bytesavailable') >= (18 * 4) + 2
       
       Byte = srl_fread (shandler,1, "uint8");
       if Byte == START_BYTE
         [data, count] = srl_fread (shandler,18, "float");
         Byte = srl_fread (shandler,1, "uint8");
         if Byte == END_BYTE
           counter = Counter + 1;
         end
       else
         data = ones(1,18);
       end
       
       data(10:12)
       data(13:15)
       
   end   
   
   %Reading keyboard entries
   x = kbhit(1);
   if ~isempty(x)
       if x == 'W'
            [nbytesent] = srl_fwrite (shandler, START_BYTE, "uint8");
            RobotSpeed = [0.1, 0.0, 0.0]
            nbytesent = srl_fwrite (shandler, RobotSpeed, "float")
       elseif x == 'A'
            [nbytesent] = srl_fwrite (shandler, START_BYTE, "uint8");
            RobotSpeed = [0.0, 0.1, 0.0]
            nbytesent = srl_fwrite (shandler, RobotSpeed, "float")
       elseif x == 'S'
            [nbytesent] = srl_fwrite (shandler, START_BYTE, "uint8");
            RobotSpeed = [-0.1, 0.0, 0.0]
            nbytesent = srl_fwrite (shandler, RobotSpeed, "float")
       elseif x == 'D'
            [nbytesent] = srl_fwrite (shandler, START_BYTE, "uint8");
            RobotSpeed = [0.0, -0.1, 0.0]
            nbytesent = srl_fwrite (shandler, RobotSpeed, "float")
       elseif x == 'Q'
            [nbytesent] = srl_fwrite (shandler, START_BYTE, "uint8");
            RobotSpeed = [0.0, 0.0, 0.0]
            nbytesent = srl_fwrite (shandler, RobotSpeed, "float")
       elseif x == 'X'
            [nbytesent] = srl_fwrite (shandler, START_BYTE, "uint8");
            RobotSpeed = [0.0, 0.0, 0.0]
            nbytesent = srl_fwrite (shandler, RobotSpeed, "float");
            break;
       end
   end
   
end

fclose(shandler);

display('End of execution')