#ifndef _BttnCtl_HEADER_FILE_
#define _BttnCtl_HEADER_FILE_



/**************************************************************
 * BttnCtl.h
 * rev 0.0 - El 2022-04-06
 * ENUMs for communication between Mainboard and Buttonboard
 * 
 * *************************************************************/


// minimal API to openMower ButtonControl board

// via serial command 115200,n,8,2



// The structure mower_com is the command inferface


// The buttonboard is waiting for structure, wich will be transmitted byte by byte serial
// The buttonboard answers with the same protocoll set the acknowledge Byte to 0xff, serial protocoll is COBS


// example set LED


/*
// 

Example 1   set status of a single LED

type    =   Set_LED;
cmd1    =   CHARGING;           // the number of the led you want to control, in this case LED nr. 4 
cmd2    =   LED_blink_slow      // control command to the LED, in this case blink slow
cmd3    =   0;                  // not used
ack     =   0;                  // not used
res     =   0;                  // not used


send bytes  :   00, 03, 02, 00, 00, 00
anwser      :   00, 03, 02, 00, 255, 00


Example 2   set all LEDs off

type    =   Set_LED;
cmd1    =   0;                  // not used     
cmd2    =   LED_All_OFF         // control command to all LEDs, set to OFF
cmd3    =   0;                  // not used
ack     =   0;                  // not used
res     =   0;                  // not used


send bytes  :   00, 00, 05, 00, 00, 00
anwser      :   00, 00, 05, 00, 255, 00


Example 3   use the second LED row as bargraph 

type    =   Set_LED;
cmd1    =   LED_BAR;            // use 7 LEDs as bar  
cmd2    =   80;                 // set the bargraph to 80 percent
cmd3    =   0;                  // not used
ack     =   0;                  // not used
res     =   0;                  // not used


send bytes  :   00, 18, 80, 00, 00, 00
anwser      :   00, 18, 80, 00, 255, 00




// example buzz


Example 1   beep active 100 ms, break 40 ms, repeate 3 times

type    =   Set_Buzzer;
cmd1    =   3;                  // repeat beep 3 times
cmd2    =   100;                // signal active 100 ms
cmd3    =   40;                 // waite afer 100 ms 40 ms
ack     =   0;                  // not used
res     =   0;                  // not used


send bytes  :   01, 03, 100, 40, 00, 00
anwser      :   01, 03, 100, 40, 255, 00




// example get version

type    =   Get_Version;
cmd1    =   0;               // not used
cmd2    =   0;               // not used
cmd3    =   0;               // not used
ack     =   0;               // not used
res     =   0;               // not used


send bytes  :   02, 00, 00, 00, 00, 00
anwser      :   02, 123, 00, 00, 255, 00   // Version number / 100 = V1.23



// see demo testprogram cobstest.cpp





*/



// protocoll header Info
enum TYPE  {Set_LED, Set_Buzzer, Get_Version, Get_Buttonnr };


// allowed LED states on, off, blink slow and fast
enum LED_state { LED_on, LED_off, LED_blink_slow, LED_blink_fast, LED_dimm, LED_All_OFF };

// Function definiton for the 18 LEDS 

enum LED_id { CHARGING, BATTERY_LOW, POOR_GPS, MOWER_LIFTED, LED5, LED6, LED7, LED8, LED9, LED10, LED11, LED12, LED13, LED14, LED15, LED16, LED17, LED18, LED_BAR };

// used for com between moweboad and buttonboard via tty

//** Attention, because of CRC16, the struct has to be always an even lentth 
#pragma pack(push, 1)
struct mower_com 
{
    // Type of this message. Has to be PACKET_ID_LL_IMU.
    uint8_t type;       // command type
    uint8_t cmd1;       // parameter to set 1
    uint8_t cmd2;       // parameter to set 2
    uint8_t cmd3;       // parameter to set 3
    uint8_t ack;        // is set to 0 , buttonboard replys alwas the command with the identical structure but set this byte to 0xff
    uint8_t res;        // not used
    uint16_t crc;       // CRC 16 from element "type" to element "res"
} __attribute__((packed));
#pragma pack(pop)






#endif // _BttnCtl_HEADER_FILE_
