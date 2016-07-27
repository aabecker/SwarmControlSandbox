function relayOn(a,which)
% RELAYON uses an existing arduino connection to turn on the lights
%   RELAYON(a,which)
%
%       which = the relay number you would like to turn on
%       a = an existing arduino connection
%
% By Lillian Lin Summer 2016

% We have 8 Relays.
%west
RELAY1 = 9;
% northwest
RELAY2 = 8;
%north
RELAY3 = 5;
% northeast
RELAY4 = 2;
%east
RELAY5 = 7;
%southeast
RELAY6 = 3;
%south
RELAY7 = 6;
%southwest
RELAY8 = 4;
    switch which
        case 1
            writeDigitalPin(a,RELAY1,0);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,1);
        case 2
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,0);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,1);
        case 3
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,0);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,1);
        case 4
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,0);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,1);
        case 5
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,0);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,1);
        case 6
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,0);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,1);
        case 7
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,0);
            writeDigitalPin(a,RELAY8,1);
        case 8
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,0);
        case 0
            writeDigitalPin(a,RELAY1,0);
            writeDigitalPin(a,RELAY2,0);
            writeDigitalPin(a,RELAY3,0);
            writeDigitalPin(a,RELAY4,0);
            writeDigitalPin(a,RELAY5,0);
            writeDigitalPin(a,RELAY6,0);
            writeDigitalPin(a,RELAY7,0);
            writeDigitalPin(a,RELAY8,0);
        otherwise
            writeDigitalPin(a,RELAY1,1);
            writeDigitalPin(a,RELAY2,1);
            writeDigitalPin(a,RELAY3,1);
            writeDigitalPin(a,RELAY4,1);
            writeDigitalPin(a,RELAY5,1);
            writeDigitalPin(a,RELAY6,1);
            writeDigitalPin(a,RELAY7,1);
            writeDigitalPin(a,RELAY8,1);
    end 
end

