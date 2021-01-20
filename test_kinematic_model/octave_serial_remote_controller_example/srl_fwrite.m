## Copyright (C) 2015 Matthew
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {Function File} {@var{retval} =} srl_fwrite (@var{serial port}, @var{number of bytes to read}, @var{optional variable type})
##
## @seealso{}
## @end deftypefn

## Author: Matthew Powelson - Tennessee Tech University
## Created: 2015-03-02

%So srl_write only accepts strings an uint8. This function converts an int to 
%a uint8 so that we can send it using srl_write. It would be nice to 
% name this fwrite. One step at a time.



function [n] = srl_fwrite (serial, indata, precision = 0)
try
outdata = indata;
serial;
indata;
precision;
%Handle Precision values

switch(precision)
  case "int8"
    outdata8 = int8(indata);
  case "uint8"
    outdata8 = uint8(indata);
  case "int16"
    outdata16 =int16(indata);                            
    outdata8 = typecast(swapbytes(outdata16), 'uint8');  %swapbytes is because typecast returns low byte first
  case "uint16"
    outdata16 = uint16(indata);
    outdata8 = typecast(swapbytes(outdata16), 'uint8');
  case "int32"
    outdata32 = int32(indata);
    outdata8 = typecast(swapbytes(outdata32), 'uint8');
  case "uint32"
    outdata32 = uint32(indata);
    outdata8 = typecast(swapbytes(outdata32), 'uint8');
  case "int64"
    outdata64 = int64(indata);
    outdata8 = typecast(swapbytes(outdata64), 'uint8');
  case "uint64"
    outdata64 = uint64(indata);
    outdata8 = typecast(swapbytes(outdata64), 'uint8');
  case "float"
    %outdata8 = typecast(swapbytes(single(indata)), 'uint8');
    outdata8 = typecast(single(indata), 'uint8');
  case 0
    outdata8 = uint8(indata);
  otherwise
  disp("Precision descriptor not recognized")
endswitch
    


  outdata8;
  n = srl_write(serial, outdata8);
  
  
  
catch
disp('WARNING:  function did not terminate correctly.  Output may be unreliable.')
end
endfunction
