//
// Created by uc0de on 5/24/19.
//
#include "serial.h"



void serial::event(int status){
    std::cout << "muie";
}

serial::serial(char * port_name, __sighandler_t fn){

    int fd;
    struct termios serial_config;                           // for serial config
    struct sigaction saio;                                  // for serial event listener


    fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if(fd == -1){
        std::cout << "[err] serial port not available \n";
        exit(1);
    }else{
        saio.sa_handler = serial::event;
        sigemptyset(&saio.sa_mask);
        saio.sa_flags = 0;
        saio.sa_restorer = NULL;
        sigaction(SIGIO,&saio,NULL);
        std::cout << "[ok] serial handler \n";


        fcntl(fd, F_SETFL, FNDELAY);
        tcgetattr(fd, &serial_config);					    // Gets the current options for the port
        cfsetispeed(&serial_config, B115200);				// Sets the Input Baud Rate
        cfsetospeed(&serial_config, B115200);				// Sets the Output Baud Rate
        serial_config.c_cflag |= (CLOCAL | CREAD);		    // default for for 8N1 serial operations
        serial_config.c_cflag &= ~PARENB;					// keep default pairity bits
        serial_config.c_cflag &= ~CSTOPB;					// keep default stop bits
        serial_config.c_cflag &= ~CSIZE;					// mask the character size bits (why? hell knows)
        serial_config.c_cflag |= CS8;						// keep default 8 data bits
        tcsetattr(fd, TCSANOW, &serial_config);			    // save serial config to control register
        std::cout << "[ok] serial port opened\n";
        }
    }

std::string serial::get(){
    tcflush(sPort, TCIFLUSH);   /* Discards old data in the rx buffer            */
    char read_buffer[32];   /* Buffer to store the data received              */
    read(sPort, &read_buffer, 32);
    //std::cout << read_buffer;
    return read_buffer;
}

// returns length of the message, string is at pointer location
int serial::update(int vibrator, int power, int rate){
    char buffer[20];
    uint8_t sBuffer = sprintf(buffer, "u,%d,%d,%d\n",  vibrator, power, rate);
    write(sPort, buffer, sBuffer);
}

