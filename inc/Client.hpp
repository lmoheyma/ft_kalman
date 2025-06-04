#pragma once

#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

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
        std::string getBuffer(void) const;
        int getSockFd(void) const;
        void init(void);
        void receive(void);
};