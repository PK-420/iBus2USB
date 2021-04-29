/** semi_iBus2USB v1.2.1 ****************************************************************************

  By: Patrick Kerr
  
  Modified by :
  Dolie 2021/04/28 - Support semi-ibus from fs-ia6 receiver 

  Target: Arduino Pro Micro
  Receiver: Flysky FS-ia6
  Prerequisite: Have the Joystick library included in your "Arduino/libraries" folder
  Wiring receiver (~5V):
     # + on RAW pin (near USB port)
     # - on GND pin
     # Signal on Rx1 pin 
          
  This Sketch takes FlySky semi i-Bus Serial data from the fs-ia6 receiver
  and turns it into an USB Joystick to use with various simulators (Tested in Aerofly RC7).

  This is based on darven discovery, on RCgroups : 
  https://www.rcgroups.com/forums/showthread.php?2711184-Serial-output-from-FS-IA6-%28Semi-I-BUS%29

  Using the ArduinoJoystickLibrary from MHeironimus on GitHub :
  https://github.com/MHeironimus/ArduinoJoystickLibrary
  
  Also inspired by semi_iBus2PPM from povlhp on GitHub for the semi_iBus data reading loop 
  semi_iBus2PPM : https://github.com/povlhp/iBus2PPM adapted for Pro Micro board.

  I am using a FlySky FS-i6 transmitter with custom firmware from benb0jangles on GitHub,
  FlySky-i6-Mod- : https://github.com/benb0jangles/FlySky-i6-Mod- 
  It unlocks all 14 channels when using semi_iBus and 8 in PPM.
  
        Copyright (c) 2017, Patrick Kerr
        
        This source code is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 3 of the License, or (at your option) any later version.
        This source code is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.
        You should have received a copy of the GNU Lesser General Public
        License along with this source code; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

**********************************************************************************************/

#include "Joystick.h" 
// #define LED 9 // Error LED pin
#define RC_CHAN 11  // Maximum 14 due to buffer size limitations. // using 11 here due to Joystick analog axies limitation
#define MIN_COMMAND 1000 // Minimum value that the RC can send (Typically 1000us)
#define MAX_COMMAND 2000 // Maximum value that the RC can send (Typically 2000us)
#define STICK_CENTER (MIN_COMMAND+((MAX_COMMAND-MIN_COMMAND)/2))
#define SEMI_IBUS_BUFFSIZE 31 // semi iBus packet size (1 byte header, space for 14 channels x 2 bytes, 2 byte checksum)

enum {  // enum defines the order of channels
  ROLL, // Channel 1 , index 0
  PITCH,
  THROTTLE,
  YAW,
  AUX1, // First Auxilary, Channel 5, index 4...
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8,
  AUX9,
  AUX10,
};

static Joystick_ Joystick(0x420, JOYSTICK_TYPE_JOYSTICK, 0, 0, true, true, true, true, true, true, true, true, true, true, true); // Maxed out 11 Analog Aux inputs
//     Joystick_ Name( Joystick ID, Joystick Type, Btn,Hat, X,    Y,    Z,   rX,   rY,   rZ, Rud., Thr., Acc., Brk., Str.

static uint8_t semi_ibusIndex = 0; // Index counter, obviously...
static uint8_t semi_ibus[SEMI_IBUS_BUFFSIZE] = {0}; // semi iBus Buffer array
static uint16_t rcValue[RC_CHAN] = {0}; // RC/Joystick Values array

void setup() {
  setupJoystick();
  Serial1.begin(115200); // Serial1.(...) for Leonardo duhh
  #ifdef LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); // LED ON Until Proper semi_iBus RX signal is detected
  #endif
}

void setupJoystick() {
  Joystick.setXAxisRange(MIN_COMMAND, MAX_COMMAND); // Set ranges
  Joystick.setYAxisRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setZAxisRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setRxAxisRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setRyAxisRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setRzAxisRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setThrottleRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setRudderRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setAcceleratorRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setBrakeRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setSteeringRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.begin(false); // Initialize Joystick without autoSend State
}

void loop() {
  if (Serial1.available()) {
    uint8_t val = Serial1.read();
    // Look for 0x55 as start of packet
    if (semi_ibusIndex == 0 && val != 0x55) return; // Not 0x55 at index 0, // Skip all and wait another loop for a new byte
    
    if (semi_ibusIndex < SEMI_IBUS_BUFFSIZE) semi_ibus[semi_ibusIndex] = val; // populate semi_ibus array with current byte
    
    semi_ibusIndex++; 
    
    if (semi_ibusIndex == SEMI_IBUS_BUFFSIZE) { // End of packet, Verify integrity
      semi_ibusIndex = 0;
      uint16_t chksum = 0x0000; // 16 bit Checksum starts at 0x0000 ...
      
      for (uint8_t i = 1; i < SEMI_IBUS_BUFFSIZE - 2; i+=2){
        chksum += semi_ibus[i] + (semi_ibus[i+1] << 8); // ... and adds every received 2 bytes (16 bit chunks) from the stream, excluding the header and the 2 last bytes (checksum).
      }
      
      uint16_t rxsum = semi_ibus[SEMI_IBUS_BUFFSIZE - 2] + (semi_ibus[SEMI_IBUS_BUFFSIZE - 1] << 8);  // Mash the 2 last bytes to form the received 16 bit Checksum, admire the bitshifting trickery to re-order bytes,
                                                                                                      // Least Significant Byte always received first. Example: Receive "0xDC05" means "0x05DC" = 1500
      if (chksum == rxsum) { // Good Packet
        for (uint8_t i = 0; i < RC_CHAN; i++) { // Put each channel value in its place
          uint16_t rcVal = (semi_ibus[(2*i)+2] << 8) + semi_ibus[(2*i)+1];      // Mash the 2 bytes from each channel together to get 16 bit rcValue, First header byte ignored (0x55)
          
          if ((rcVal < MIN_COMMAND) || (rcVal > MAX_COMMAND)) return; // if rcValue is out of bounds (MIN_COMMAND/MAX_COMMAND) the frame is discarded;
          
          rcValue[i] = rcVal;
        }
        
        updateJoystick(); // When RX Frame is done, update Joystick values and send its state to the computer.
        
        #ifdef LED
          digitalWrite(LED, LOW); // OK Packet, Clear LED
        #endif
      }
      #ifdef LED
        else digitalWrite(LED, HIGH); // Checksum Error
      #endif
    }
  }
}

void updateJoystick() {
  Joystick.setXAxis(rcValue[ROLL]);
  Joystick.setYAxis(rcValue[PITCH]);
  Joystick.setThrottle(rcValue[THROTTLE]);
  Joystick.setRzAxis(rcValue[YAW]);
  Joystick.setZAxis(rcValue[AUX1]);
  Joystick.setRxAxis(rcValue[AUX2]);
  Joystick.setRyAxis(rcValue[AUX3]); 
  Joystick.setRudder(rcValue[AUX4]);
  Joystick.setAccelerator(rcValue[AUX5]);
  Joystick.setBrake(rcValue[AUX6]);
  Joystick.setSteering(rcValue[AUX7]);
  Joystick.sendState();
}
