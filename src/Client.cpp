#include "../inc/Client.hpp"

Client::Client()
{
    _start_msg = "READY"; 
}

Client::~Client()
{
}

int Client::send()
{
    if ( (_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
  
    memset(&_servaddr, 0, sizeof(_servaddr)); 

    _servaddr.sin_family = AF_INET; 
    _servaddr.sin_port = htons(PORT); 
    _servaddr.sin_addr.s_addr = INADDR_ANY; 
      
    int n;
    socklen_t len; 
      
    sendto(_sockfd, (const char *)_start_msg, strlen(_start_msg), 
        MSG_CONFIRM, (const struct sockaddr *) &_servaddr,  
            sizeof(_servaddr)); 
    std::cout<<"Hello message sent."<<std::endl; 

    n = recvfrom(_sockfd, (char *)_buffer, MAXLINE,  
                MSG_WAITALL, (struct sockaddr *) &_servaddr, 
                &len);
    std::cout << "Server :" << _buffer << std::endl;

    while(strcmp(_buffer, "MSG_END"))
    {
        bzero(_buffer, MAXLINE);
        n = recvfrom(_sockfd, (char *)_buffer, MAXLINE,  
            MSG_WAITALL, (struct sockaddr *) &_servaddr, 
            &len); 
        std::cout << _buffer << std::endl; 
    }

    close(_sockfd); 
    return 0; 
}
