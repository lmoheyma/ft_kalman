#include "../inc/Client.hpp"

int main() {
    Client client;
    client.init();

    while (true) {
        client.receive();
        if (client.getBuffer().find("MSG_END") != std::string::npos)
            break;
    }

    std::cout << "\n--- Messages reçus ---\n";
    std::cout << client.getBuffer() << std::endl;

    close(client.getSockFd());
    return 0;
}