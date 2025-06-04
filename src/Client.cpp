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
        result += this->_buffer[i] + "\n";
    }
    return result;
}

int Client::getSockFd(void) const
{
    return this->_sockfd;
}

void Client::init(void)
{
    if ( (this->_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    }

    memset(&this->_servaddr, 0, sizeof(this->_servaddr)); 
    this->_servaddr.sin_family = AF_INET; 
    this->_servaddr.sin_port = htons(PORT); 
    this->_servaddr.sin_addr.s_addr = INADDR_ANY;

    sendto(this->_sockfd, (const char *)START_MSG, strlen(START_MSG), 
        MSG_CONFIRM, (const struct sockaddr *) &this->_servaddr,  
            sizeof(this->_servaddr));
}

void Client::receive(void)
{
    if (_index >= MAXLINE) {
        std::cerr << "buffer overflow" << std::endl;
        return;
    }
    
    socklen_t len;
    char tmp[MAXLINE] = {0};

    int n = recvfrom(this->_sockfd, tmp, MAXLINE - 1,  
                MSG_WAITALL, (struct sockaddr *) &this->_servaddr, 
                &len);
    if (n < 0) {
        perror("recvfrom failed");
        return;
    }
    tmp[n] = '\0';
    if (_index > 2) { // Skip two first messages
        this->_buffer[_index] = std::string(tmp);
        std::cout << "Reçu (" << _index << ") : " << this->_buffer[_index] << std::endl;
    }
    _index++;
}
