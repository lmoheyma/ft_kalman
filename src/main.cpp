#include "../inc/Client.hpp"
#include "../inc/Parser.hpp"
#include "../inc/KalmanFilter.hpp"

int main() {
    Client client;
    Parser parser;
    client.init();

    while (true) {
        client.receive();
        if (client.getBuffer().find("MSG_END") != std::string::npos)
            break;
    }
    close(client.getSockFd());
    
    std::map<std::string, std::vector<double> > parsed = parser.parseMessage(client.getBuffer());
    std::map<std::string, std::vector<double> >::iterator it;

    for (it = parsed.begin(); it != parsed.end(); ++it) {
        std::cout << it->first << " : ";
        std::vector<double>::iterator vit;
        for (vit = it->second.begin(); vit != it->second.end(); ++vit) {
            std::cout << *vit << " ";
        }
        std::cout << std::endl;
    }
    KalmanFilter kalmanFilter;
    kalmanFilter.setStateVector(parser.createInitialState(parsed));
    std::cout << "State Vector (0) : " << std::endl;
    printVector(kalmanFilter.getStateVector());
    return 0;
}
