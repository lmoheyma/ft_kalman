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

class Client
{
    private:
        int _sockfd;
        char _buffer[MAXLINE]; 
        const char *_start_msg;
        struct sockaddr_in _servaddr; 
    public :
        Client();
        ~Client();
        int send();
};