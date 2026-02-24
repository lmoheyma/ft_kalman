#include "../inc/Client.hpp"

Client::Client() : _index(0)
{
}

Client::~Client()
{
}

void Client::setIndex(int index)
{
    this->_index = index;
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
    this->_servaddr.sin_addr.s_addr = inet_addr(HOST);
}

void Client::sendFirstMessage(void) {
    sendto(this->_sockfd, (const char *)START_MSG, strlen(START_MSG), 
        0, (const struct sockaddr *) &this->_servaddr,  
            sizeof(this->_servaddr));
}

void Client::receiveFirstMessage(void)
{
    if (_index >= MAXLINE) {
        std::cerr << "buffer overflow" << std::endl;
        return;
    }
    
    socklen_t len = sizeof(this->_servaddr);
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
    }
    _index++;
}

void Client::receive(void)
{
    if (_index >= MAXLINE) {
        std::cerr << "buffer overflow" << std::endl;
        return;
    }
    
    socklen_t len = sizeof(this->_servaddr);
    char tmp[MAXLINE] = {0};

    int n = recvfrom(this->_sockfd, tmp, MAXLINE - 1,  
                MSG_WAITALL, (struct sockaddr *) &this->_servaddr, 
                &len);
    if (n < 0) {
        perror("recvfrom failed");
        return;
    }
    tmp[n] = '\0';
    if (_index >= 1) { // Skip MSG_START
        this->_buffer[_index] = std::string(tmp);
    }
    _index++;
}

void Client::sendEstimation(const std::vector<double>& estimation) {
    if (estimation.size() < 3) {
        std::cerr << "Estimation invalide (taille < 3)" << std::endl;
        return;
    }
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%.15f %.15f %.15f", estimation[0], estimation[1], estimation[2]);
    sendto(this->_sockfd, (const char *)buffer, strlen(buffer), 
        0, (const struct sockaddr *) &this->_servaddr,  
            sizeof(this->_servaddr));
}
