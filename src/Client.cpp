#include "../inc/Client.hpp"

Client::Client() : _index(0)
{
}

Client::~Client()
{
}

std::string Client::getBuffer(void) const
{
    std::string result;
    for (int i = 0; i < _index; ++i) {
        result += _buffer[i] + "\n";
    }
    return result;
}

int Client::getSockFd(void) const
{
    return (this->_sockfd);
}

void Client::init(void)
{
    if ( (_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    }

    memset(&_servaddr, 0, sizeof(_servaddr)); 
    _servaddr.sin_family = AF_INET; 
    _servaddr.sin_port = htons(PORT); 
    _servaddr.sin_addr.s_addr = INADDR_ANY;

    sendto(_sockfd, (const char *)START_MSG, strlen(START_MSG), 
        MSG_CONFIRM, (const struct sockaddr *) &_servaddr,  
            sizeof(_servaddr));
}

void Client::receive(void)
{
    if (_index >= MAXLINE) {
        std::cerr << "buffer overflow" << std::endl;
        return;
    }
    
    socklen_t len;
    char tmp[MAXLINE] = {0};

    int n = recvfrom(_sockfd, tmp, MAXLINE - 1,  
                MSG_WAITALL, (struct sockaddr *) &_servaddr, 
                &len);
    if (n < 0) {
        perror("recvfrom failed");
        return;
    }
    tmp[n] = '\0';
    _buffer[_index] = std::string(tmp);
    std::cout << "Reçu (" << _index << ") : " << _buffer[_index] << std::endl;
    _index++;
}
