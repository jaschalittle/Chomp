#ifndef XBEE_H
#define XBEE_H

// Xbee configuration notes:
// Followed tutorial here: https://eewiki.net/display/Wireless/XBee+Wireless+Communication+Setup
// Xbee SN 13A200 40BEFC5C is set to Coordinator AT, and DH/DL programmed to the SN of the Router AT
// Xbee SN 13A200 40B9D1B1 is set to Router AT, and DH/DL programmed to the SN of the Coordinator AT
// They're talking on PAN ID 2001 (A Space Odyssey)
extern HardwareSerial & Xbee;
void xbee_init();

#endif // XBEE_H
