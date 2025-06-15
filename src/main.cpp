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
    kalmanFilter.setAcceleration(parsed.at("ACCELERATION"));
    kalmanFilter.setStateVector(parser.createInitialState(parsed));
    std::cout << "State Vector (0) : " << std::endl;
    printVector(kalmanFilter.getStateVector());
    kalmanFilter.initProcessNoiseMatrix();
    kalmanFilter.initCovarianceMatrix();
    kalmanFilter.initMeasurementMatrix();
    kalmanFilter.initStateTransitionMatrix();

    /*
    Prediction :
    xk-1 = F * xk-1 + B
    Pk-1 = F * Pk-1 * Ft + Q

    Update :
    yk = zk − Hk-1
​    Sk = H * Pk-1 Ht + R
    Kk = Pk-1 * Ht * Sk
    xk = xk-1 + Kk * yk
    Pk = (I - Kk * H) * Pk-1
    */
    return 0;
}
