clear all

a = arduino('COM5','uno');

RELAY1 = 7;
RELAY2 = 6;
RELAY3 = 5;
RELAY4 = 4;
RELAY5 = 3;
RELAY6 = 2;
RELAY7 = 8;
RELAY8 = 9;
        
      writeDigitalPin(a, RELAY1, 0);
      writeDigitalPin(a, RELAY2, 0);
      writeDigitalPin(a, RELAY3, 0);
      writeDigitalPin(a, RELAY4, 0);
      writeDigitalPin(a, RELAY5, 0);
      writeDigitalPin(a, RELAY6, 0);
      writeDigitalPin(a, RELAY7, 0);
      writeDigitalPin(a, RELAY8, 0);
      
      pause(2);
