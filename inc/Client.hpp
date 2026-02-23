#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fstream>

#define HOST "127.0.0.1"
#define PORT 4242
#define MAXLINE 1024
#define START_MSG "READY"

class Client
{
    private:
        int _sockfd;
        std::string _buffer[MAXLINE];
        int _index;
        struct sockaddr_in _servaddr;
    public :
        Client();
        ~Client();
        void setIndex(int index);
        std::string getBuffer(void) const;
        int getSockFd(void) const;
        void init(void);
        void receive(void);
        void receiveFirstMessage(void);
        void sendFirstMessage(void);
        void sendEstimation(const std::vector<double>& estimation);
};
