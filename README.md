# iBus2USB v1.0

**Target:** Arduino Leonardo Pro Micro

**Prerequisite:** Have the Joystick library included in your "Arduino/libraries" folder*
  
**Wiring receiver (~5V):**

![Wiring Diagram](https://raw.githubusercontent.com/PK-420/iBus2USB/master/Leonardo.jpg)
     
This Sketch takes FlySky i-Bus Serial data from the receiver and turns it into an USB Joystick to use with various drone simulators (Tested in LiftOff and DRL Simulator)
 
Using the ArduinoJoystickLibrary from MHeironimus on GitHub : https://goo.gl/KoNdqs
 
Also inspired by *iBus2PPM* from *povlhp* on GitHub for the iBus data reading loop that I adapted for the Leonardo board. *iBus2PPM* : https://goo.gl/pX7LpG 
  
I am using a FlySky FS-i6 transmitter with custom firmware from *benb0jangles* on GitHub, *FlySky-i6-Mod-* : https://goo.gl/3rDx3a which unlocks all 10 channels when using iBus and 8 in PPM, but who wants PPM when almost every receiver can do iBus for the same price.

**Suggested receiver** : *Flysky FS82 iBUS AFHDS 2A* Low power, compact, simple to wire up and available on Banggood for cheap : https://goo.gl/7UkU3k
