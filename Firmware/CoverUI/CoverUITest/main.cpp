/* Version 0.91 v. 27.05.2022  schnelles Blinken von ontime 500 auf 250 ms verkleinert
*/




//#define _serial_debug_
//#define _testCOBS_

#include <stdio.h>
#include <string>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <exception>

#include <chrono>
#include <thread>


// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "COBS.h"
#include "BttnCtl.h"
#include "CRC.h"
#include <cmath>
using namespace std;

COBS cobs;
#define buffersize 256


static string VERSION         = "V 0.91";
char  DEFAULT_SERIAL[]        = "/dev/ttyUSB0";


static uint number_inbuffer = 0;
uint8_t read_buf [buffersize];
uint8_t serial_buf [buffersize];
uint32_t read_ptr = 0;
uint32_t write_ptr = 0;
uint8_t COBS_buf [buffersize];


/****************************************************************************************************
 * 
 * PacketReceived() there is a serial stream in the buffer
 * decode it with cobs, calc crc and decode into the messagestruct
 *****************************************************************************************************/


void PacketReceived()
{
#ifdef _serial_debug_
    printf("\nPacketReceived()");
    printf("\nbefore decode count = %d values are ",number_inbuffer);
    for (int i=0; i<number_inbuffer; i++)
    {
      printf(", 0x%02x",COBS_buf[i]);

    }
#endif

    // deocode COBS
    uint8_t decoded_buffer[number_inbuffer];
    size_t data_size = cobs.decode(COBS_buf, number_inbuffer-1, (uint8_t *)decoded_buffer);

#ifdef _serial_debug_
    printf("\nafter decode count = %d values are ",(int)data_size);
    for (int i=0; i<data_size; i++)
    {
      printf(", 0x%02x",encoded_buffer[i]);

    }
#endif

    // calculate the CRC
    if (data_size < 3) return;
    uint16_t calc_crc = CRC::Calculate(decoded_buffer, data_size - 2, CRC::CRC_16_CCITTFALSE());

    if(decoded_buffer[0] == Get_Version && data_size == sizeof(struct msg_get_version)) {
        struct msg_get_version *message = (struct msg_get_version*)decoded_buffer;
        if(message->crc == calc_crc) {
            printf("button board has version: %d\n", message->version);
        } else {
            printf("GetVersion invalid CRC");
        }
    }else if(decoded_buffer[0] == Get_Button && data_size == sizeof(struct msg_event_button)) {
        struct msg_event_button *message = (struct msg_event_button*)decoded_buffer;
        if(message->crc == calc_crc) {
            printf("button %d was pressed with duration %d\n", message->button_id,  message->press_duration);
        } else {
            printf("GetVersion invalid CRC");
        }
    }
}




/****************************************************************************************************
 * 
 * reply wait for timeout ms if return with result   
 *
 *****************************************************************************************************/

#define timeout 100

void reply(int com_port)
{
    number_inbuffer=0;
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
    int num_bytes=0;
    try
    {
        num_bytes = read(com_port, &read_buf, sizeof(read_buf));
    }
    catch(...)
    {
        std::cout << "Serial error check cable " << '\n';
        return;
    }




    //  printf("*** Error *** no connection to target, check cable");




    if (num_bytes !=0)
    {
        for (int i=0; i<num_bytes; i++)
        {
            serial_buf[write_ptr]=read_buf[i];
            write_ptr++;
            if(write_ptr >= buffersize) write_ptr = 0;
        }




#ifdef _serial_debug_

#endif

        while (write_ptr != read_ptr )
        {
            COBS_buf[number_inbuffer]=serial_buf[read_ptr];
            number_inbuffer++;
            read_ptr++;
            if (read_ptr >= buffersize) read_ptr = 0;

#ifdef _serial_debug_
            printf("\ncount %d , value 0x%2x ",number_inbuffer,COBS_buf[number_inbuffer-1]);
#endif

            if (COBS_buf[number_inbuffer-1] == 0 )
            {
                PacketReceived();
                number_inbuffer=0;

            }

        }

    }


}


/****************************************************************************************************
 * 
 * sendMessage send the command strucutre via COBS and serial Port and wait for reply
 *
 *****************************************************************************************************/


void sendMessage(void *message, size_t size, int serial_port)
{
    // packages need to be at least 1 byte of type, 1 byte of data and 2 bytes of CRC

    if (size < 4)
    {
        return;
    }
    uint8_t *data_pointer = (uint8_t *)message;
    uint16_t *crc_pointer = (uint16_t *)(data_pointer + (size - 2));

    // calculate the CRC
    *crc_pointer = CRC::Calculate(message, size - 2, CRC::CRC_16_CCITTFALSE());
    // structure is filled and CRC calculated, so print out, what should be encoded
    //

#ifdef _serial_debug_
    printf("\nprint struct before encoding %d byte : ",(int)size);
  uint8_t *temp = data_pointer;
  for (int i = 0; i < size; i++)
  {
    printf("0x%02x , ", *temp);
    temp++;
  }
#endif

    // encode message
    uint8_t out_buf[100];
    size_t encoded_size = cobs.encode((uint8_t *)message, size, out_buf);
    out_buf[encoded_size] = 0;
    encoded_size++;

#ifdef _serial_debug_
    printf("\nencoded message               byte : ");
  for (uint i = 0; i < encoded_size; i++)
  {
    printf("0x%02x , ", out_buf[i]);
  }
#endif
    write(serial_port, out_buf, encoded_size);
    //if (!reply(serial_port)) printf("\n*** got no answer ***");

}




/****************************************************************************************************
 * 
 * Init Serial Port 115200 baud, 8,n,n
 *
 *****************************************************************************************************/

int Init_serial_board(char *channel)
{
    // Init serial port
    int ret_val = -1;
    struct termios tty;

    int serial_port = open(channel, O_RDWR);
    if (serial_port < 0)
    {
#ifdef _serial_debug_
        printf("Error %i from open: \n", errno);
#endif
        return (ret_val);
    }

    if (tcgetattr(serial_port, &tty) != 0)
    {
#ifdef _serial_debug_
        printf("Error %i from tcgetattr: \n", errno);
#endif
        return (ret_val);
    }


    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
#ifdef _serial_debug_
        printf("Error %i from tcsetattr: \n", errno);
#endif
        return (ret_val);
    }
    return (ret_val=serial_port);
}


/****************************************************************************************************
 * 
 * fill command structure LED command and send via serial port
 *
 *****************************************************************************************************/


void set_LED_state(int com_port, uint cmd1, uint cmd2, uid_t cmd3)
{




}


/****************************************************************************************************
 * 
 * fill command structure getversion and send via serial port
 *
 *****************************************************************************************************/

/****************************************************************************************************
 * 
 * fill command structure buzzercommand and send via serial port
 *
 *****************************************************************************************************/


void set_buzzer(int com_port, uint ontime, uint offtime, uint count)
{


}


void getversion(int com_port)
{
    printf("Get Version\n");
    // Fill structure
    struct msg_get_version msg;
    msg.type = Get_Version;

    sendMessage(&msg, sizeof(msg), com_port);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    reply(com_port);

}


void setLed(struct msg_set_leds &msg, int led, uint8_t state) {
    // mask the current led state
    uint64_t mask = ~(((uint64_t)0b111) << (led*3));
    msg.leds &= mask;
    msg.leds |= ((uint64_t)(state&0b111)) << (led*3);
}

void setBars7(struct msg_set_leds &msg, double value) {
    int on_leds = round(value * 7.0);
    for(int i = 0; i < 7; i++) {
        setLed(msg, 4+i, i < on_leds ? LED_on : LED_off);
    }
}

void setBars4(struct msg_set_leds &msg, double value) {
    int on_leds = round(value * 4.0);
    for(int i = 0; i < 7; i++) {
        setLed(msg, i, i < on_leds ? LED_on : LED_off);
    }
}

void LEDstatic(int com_port)
{

    struct msg_set_leds msg;
    msg.type = Set_LEDs;
    msg.leds = 0;

    for(int led = 0; led < 18; led++) {
        setLed(msg, led, LED_on);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        sendMessage(&msg, sizeof(msg), com_port);
        setLed(msg, led, LED_off);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        sendMessage(&msg, sizeof(msg), com_port);
    }

}


void LEDfastblink(int com_port)
{
    struct msg_set_leds msg;
    msg.type = Set_LEDs;
    msg.leds = 0;

    for(int led = 0; led < 18; led++) {
        setLed(msg, led, LED_blink_fast);
        sendMessage(&msg, sizeof(msg), com_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }
    for(int led = 0; led < 18; led++) {
        setLed(msg, led, LED_off);
        sendMessage(&msg, sizeof(msg), com_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }


}

void LEDslowblink(int com_port)
{
    struct msg_set_leds msg;
    msg.type = Set_LEDs;
    msg.leds = 0;

    for(int led = 0; led < 18; led++) {
        setLed(msg, led, LED_blink_slow);
        sendMessage(&msg, sizeof(msg), com_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    for(int led = 0; led < 18; led++) {
        setLed(msg, led, LED_off);
        sendMessage(&msg, sizeof(msg), com_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }
}


void Buzzertest(int com_port)
{
    printf("Test Buzzer\n");
    set_buzzer(com_port,100,40,3);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    reply(com_port);
    set_buzzer(com_port,100,40,5);
    reply(com_port);
}


void LEDBar7test(int com_port)
{
    struct msg_set_leds msg;
    msg.type = Set_LEDs;
    msg.leds = 0;

    for(int i = 0; i < 10; i++) {
        for (double d = 0; d < 1.0; d += 0.05) {
            setBars7(msg, d);
            sendMessage(&msg, sizeof(msg), com_port);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        for (double d = 0; d < 1.0; d += 0.05) {
            setBars7(msg, 1.0 - d);
            sendMessage(&msg, sizeof(msg), com_port);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

void LEDBar4test(int com_port)
{
    struct msg_set_leds msg;
    msg.type = Set_LEDs;
    msg.leds = 0;

    for(int i = 0; i < 10; i++) {
        for (double d = 0; d < 1.0; d += 0.05) {
            setBars4(msg, d);
            sendMessage(&msg, sizeof(msg), com_port);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        for (double d = 0; d < 1.0; d += 0.05) {
            setBars4(msg, 1.0 - d);
            sendMessage(&msg, sizeof(msg), com_port);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}



/****************************************************************************************************
 * 
 * main
 *
 *****************************************************************************************************/


int main(int argc, char *argv[])
{

    printf("Testprogramm OpnMower UI-Board Version %s\n",VERSION.c_str() );


    char serialchannel[80];
    if ( argc == 1)
    {
        printf("No command line parameter for serial channel, using default value : %s\n",DEFAULT_SERIAL);
        strcpy(serialchannel,DEFAULT_SERIAL);

    }
    else if (argc > 2)
    {
        printf("To many agruments, try cobstest <serialport>\n");
    }
    else
    {
        strcpy(serialchannel,argv[1]);
        printf("using serial channel: %s \n",serialchannel);


    }


    int com_port = Init_serial_board(serialchannel);
    if (com_port < 0)
    {
        printf("\n\n\r*** ERROR *** can't open serial port. Serial cable connected ?  Program aborted");
        return 1;
    }


    //set_LED_state(com_port , 0 ,LED_on , 0);

    //reply(com_port);


    getversion(com_port);

    LEDstatic(com_port);

    LEDfastblink(com_port);

    LEDslowblink(com_port);

    Buzzertest(com_port);


    LEDBar4test(com_port);

    LEDBar7test(com_port);


    printf("Test Buttons\n");
    while (true)
    {


        reply(com_port);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}