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
## @deftypefn {Function File} {@var{retval} =} srl_fread (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Matthew Powelson - Tennessee Tech University
## Created: 2015-03-06

function [outdata, count] = srl_fread (serial, n, precision = 0)

try

%Get length of packet expected.
%I'm sure there is a more clever method of doing this
switch(precision)
  case "int8"
    m = n;
  case "uint8"
    m = n;
  case "int16"
    m = 2*n;
  case "uint16"
    m = 2*n;
  case "int32"
    m = 4*n;
  case "uint32"
    m = 4*n;
  case "int64"
    m = 8*n;
  case "uint64"
    m = 8*n;
  case "float"
    m = 4*n;
  case 0
    m = n;
  otherwise
  disp("Precision descriptor not recognized")
endswitch



%Get serial data
indata = srl_read(serial , m);
indata = uint8(indata);            %not sure if already uint8




%Convert to correct data type
switch(precision)
  case "int8"
    outdata = int8(indata);
    outdata = swapbytes(typecast(indata , 'int32'));
  case "uint8"
    outdata = uint8(indata);
  case "int16"
    outdata = swapbytes(typecast(indata , 'int16'));   %Swap bytes to get low byte first for typecast
  case "uint16"
    outdata = swapbytes(typecast(indata , 'uint16'));
  case "int32"
  case "uint32"
    outdata = swapbytes(typecast(indata , 'uint32'));
  case "int64"
    outdata = swapbytes(typecast(indata , 'int64'));
  case "uint64"
    outdata = swapbytes(typecast(indata , 'uint64'));
  case "float"
    %outdata = swapbytes(typecast(indata , 'single'));
    outdata = typecast(indata , 'single');
  case 0
    outdata = uint8(indata);
  otherwise
  disp("Precision descriptor not recognized")
endswitch
    
count = length(outdata);
  
  
catch
disp('WARNING:  srl_fread function did not terminate correctly.  Output may be unreliable.')
end
endfunction


