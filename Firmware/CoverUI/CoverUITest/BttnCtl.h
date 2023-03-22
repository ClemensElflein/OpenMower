#ifndef _BttnCtl_HEADER_FILE_
#define _BttnCtl_HEADER_FILE_



/**************************************************************
 * BttnCtl.h
 * rev 0.0 - El 2022-04-06
 * ENUMs for communication between Mainboard and Buttonboard
 * 
 * *************************************************************/

// Protocol Header Info
enum TYPE
{
    Get_Version = 0xB0,
    Set_Buzzer = 0xB1,
    Set_LEDs = 0xB2,
    Get_Button = 0xB3
};


// Function definiton for the 18 LEDS
enum LED_id
{
    MOWER_LIFTED = 0,
    POOR_GPS = 1,
    BATTERY_LOW = 2,
    CHARGING = 3,
    LED5 = 4,
    LED6 = 5,
    LED7 = 6,
    LED8 = 7,
    LED9 = 8,
    LED10 = 9,
    LED11 = 10,
    LED12 = 11,
    LED13 = 12,
    LED14 = 13,
    LED15 = 14,
    LED16 = 15,
    LED17 = 16,
    LED18 = 17,
    LED_BAR = 18,
    LED_BAR1 = 19
};

enum LED_state {
    LED_off = 0b000,
    LED_blink_slow = 0b101,
    LED_blink_fast = 0b110,
    LED_on = 0b111
};
#pragma pack(push, 1)
struct msg_get_version
{
    uint8_t type; // command type
    uint8_t reserved; // padding
    uint16_t version;
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)


#pragma pack(push, 1)
struct msg_set_buzzer
{
    uint8_t type; // command type
    uint8_t repeat; // Repeat X times
    uint8_t on_time; // Signal on time
    uint8_t off_time; // Signal off time
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)


/**
 * @brief Use this to update the LED matrix
 * Each LED gets three bits with the following meaning:
 * 0b000 = Off
 * 0b001 = reserved for future use
 * 0b010 = reserved for future use
 * 0b011 = reserved for future use
 * 0b100 = reserved for future use
 * 0b101 = On slow blink
 * 0b110 = On fast blink
 * 0b111 = On
 */
#pragma pack(push, 1)
struct msg_set_leds
{
    uint8_t type; // command type
    uint8_t reserved; // padding
    uint64_t leds;
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)

#pragma pack(push, 1)
struct msg_event_button
{
    uint8_t type; // command type
    uint16_t button_id;
    uint8_t press_duration;
    uint16_t crc; // CRC 16
} __attribute__((packed));
#pragma pack(pop)




#endif // _BttnCtl_HEADER_FILE_
