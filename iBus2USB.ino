/** iBus2USB v1.1 ****************************************************************************

  By: Patrick Kerr  
  Target: Arduino Leonardo Pro Micro
  Prerequisite: Have the Joystick library included in your "Arduino/libraries" folder
  Wiring receiver (~5V):
     # + on RAW pin (near USB port)
     # - on GND pin
     # Signal on Rx1 pin 
          
  This Sketch takes FlySky i-Bus Serial data from the receiver 
  and turns it into an USB Joystick to use with various drone simulators 
  (Tested in LiftOff, VelociDrone and DRL Simulator)

  Using the ArduinoJoystickLibrary from MHeironimus on GitHub : https://goo.gl/KoNdqs
  
  Also inspired by iBus2PPM from povlhp on GitHub for the iBus data reading loop 
  iBus2PPM : https://goo.gl/pX7LpG adapted for Leonardo board.

  I am using a FlySky FS-i6 transmitter with custom firmware from benb0jangles on GitHub,
  FlySky-i6-Mod- : https://goo.gl/3rDx3a which unlocks all 10 channels when using iBus and 8 in PPM,
  but who wants to use PPM when almost every receiver can do iBus for the same price.
  
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

#define RC_CHAN 10 // Number of assigned/used RC channels, modify according to your transmitter/receiver capabilities (this code should support up to 14)
#define VARIO // Use this for RC transmitters with 2 AUX pots, comment out if only using AUX switches as buttons
// #define LED 9 // Error LED pin

#ifdef VARIO // In case transmitter is configured with 2 AUX Pots instead of buttons
  #define POT1 RC_CHAN-2 // Default : last 2 Auxiliary channels assigned to Potentiometers
  #define POT2 RC_CHAN-1
  //                                          HID ID,     Joystick Type,           Btn,       Hat, X,    Y,    Z,   rX,   rY,   rZ,   Rud.,  Thr.,  Acc.,  Brk.,  Str.
  static Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, (RC_CHAN - 6), 0, true, true, true, true, true, true, false, false, false, false, false);
#else
  static Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, (RC_CHAN - 4), 0, true, true, true, true, false, false, false, false, false, false, false);
#endif

#define MIN_COMMAND 1000 // Minimum value that the RC can send (Typically 1000us)
#define MAX_COMMAND 2000 // Maximum value that the RC can send (Typically 2000us)
#define STICK_CENTER (MIN_COMMAND+((MAX_COMMAND-MIN_COMMAND)/2))
#define IBUS_BUFFSIZE 32 // iBus packet size (2 byte header, space for 14 channels x 2 bytes, 2 byte checksum)

enum {  // enum defines the order of channels
  ROLL, // Channel 1 , index 0
  PITCH,
  THROTTLE,
  YAW,
  AUX1, // First Auxilary, Channel 5, index 4...
};

static uint8_t ibusIndex = 0; // Index counter, obviously...
static uint8_t ibus[IBUS_BUFFSIZE] = {0}; // iBus Buffer array
static uint16_t rcValue[RC_CHAN] = {0}; // RC/Joystick Values array

void setup() {
//  pinMode(LED, OUTPUT);
  setupJoystick();
  Serial1.begin(115200); // Serial1.(...) for Leonardo duhh
//  digitalWrite(LED, HIGH); // LED ON Until Proper iBUS RX signal is detected
}

void loop() {
  if (Serial1.available()) {
    uint8_t val = Serial1.read();
    // Look for 0x2040 as start of packet
    if (ibusIndex == 0 && val != 0x20) return; // Not 0x20 at index 0, // Skip all and wait another loop for a new byte
    if (ibusIndex == 1 && val != 0x40) {  // Not the expected 0x40 at index 1, 
      ibusIndex = 0;                      // so we have to reset the index.
      return; // Skip all and wait next loop for another byte.
    }
    if (ibusIndex < IBUS_BUFFSIZE) ibus[ibusIndex] = val; // populate ibus array with current byte
    ibusIndex++; 
    if (ibusIndex == IBUS_BUFFSIZE) { // End of packet, Verify integrity
      ibusIndex = 0;
      uint16_t chksum = 0xFFFF; // 16 bit Checksum starts at 0xFFFF ...
      for (uint8_t i = 0; i < IBUS_BUFFSIZE - 2; i++) chksum -= ibus[i]; // ... and substracts every received byte (8 bit chunks) from the stream, including the header but not the 2 last bytes.
      uint16_t rxsum = ibus[IBUS_BUFFSIZE - 2] + (ibus[IBUS_BUFFSIZE - 1] << 8);  // Mash the 2 last bytes to form the received 16 bit Checksum, admire the bitshifting trickery to re-order bytes,
      if (chksum == rxsum) { // Good Packet, Unroll channels                      // Least Significant Byte always received first. Example: Receive "0xDC05" means "0x05DC" = 1500
        for (uint8_t i = 0; i < RC_CHAN; i++) {
          uint16_t rcVal = (ibus[(2*i)+3] << 8) + ibus[(2*i)+2];      // Mash the 2 bytes from each channel together to get 16 bit rcValue, First 2 bytes ignored (0x2040)
          if ((rcVal < MIN_COMMAND) || (rcVal > MAX_COMMAND)) return; // if rcValue is out of bounds (MIN_COMMAND/MAX_COMMAND) the frame is discarded;
          rcValue[i] = rcVal;
        }
        updateJoystick(); // When RX Frame is done, update Joystick values and send its state to the computer.
        //digitalWrite(LED, LOW); // OK Packet, Clear LED
      }
      // else digitalWrite(LED, HIGH); // Checksum Error
      return;
    }
  }
}

void setupJoystick() {
  Joystick.setXAxisRange(MIN_COMMAND, MAX_COMMAND); // Set ranges
  Joystick.setYAxisRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setZAxisRange(MIN_COMMAND, MAX_COMMAND);
  Joystick.setRxAxisRange(MIN_COMMAND, MAX_COMMAND);
  #ifdef VARIO
    Joystick.setRyAxisRange(MIN_COMMAND, MAX_COMMAND);
    Joystick.setRzAxisRange(MIN_COMMAND, MAX_COMMAND);
  #endif
  Joystick.begin(false); // Initialize Joystick without autoSend State
}

void updateJoystick() {
  Joystick.setRxAxis(rcValue[YAW]);
  Joystick.setZAxis(rcValue[THROTTLE]);
  Joystick.setXAxis(rcValue[ROLL]);
  Joystick.setYAxis(rcValue[PITCH]);
  for (uint8_t i = 0; i < RC_CHAN - AUX1; i++) Joystick.setButton(i, rcValue[AUX1 + i] > STICK_CENTER);
  #ifdef VARIO 
    Joystick.setRyAxis(rcValue[POT1]);
    Joystick.setRzAxis(rcValue[POT2]);
  #endif
  Joystick.sendState();
}

