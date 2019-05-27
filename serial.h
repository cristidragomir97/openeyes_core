//
// Created by uc0de on 5/24/19.
//

#ifndef OPENEYES_CORE_SERIAL_H
#define OPENEYES_CORE_SERIAL_H

#include <stdio.h>      //Standard input/output definitions
#include <string.h>     //String function definitions
#include <unistd.h>     //UNIX standard function definitions
#include <fcntl.h>      //File control definitions
#include <errno.h>      //Error number definitions
#include <sys/signal.h>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>

class serial{
private:
    int sPort;
    char read_buffer[32];

public:
    explicit serial(char * port_name, __sighandler_t fn);
    static void event(int status);

    std::string get();
    int update(int vibrator, int power, int rate);

};

#endif //OPENEYES_CORE_SERIAL_H
