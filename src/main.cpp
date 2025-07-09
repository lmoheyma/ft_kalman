#include "../inc/Client.hpp"
#include "../inc/Parser.hpp"
#include "../inc/KalmanFilter.hpp"
#include <thread>
#include <chrono>

int main() {
    Client client;
    Parser parser;
    client.init();

    while (true) {
        client.receive_first_message();
        if (client.getBuffer().find("MSG_END") != std::string::npos) {
            break;
        }
    }

    std::map<std::string, std::vector<double> > parsed = parser.parseMessage(client.getBuffer());
    KalmanFilter kalmanFilter;
    kalmanFilter.setAcceleration(parsed.at("ACCELERATION"));
    kalmanFilter.setStateVector(parser.createInitialState(parsed));
    kalmanFilter.initProcessNoiseMatrix();
    kalmanFilter.initCovarianceMatrix();
    kalmanFilter.initMeasurementMatrix();
    kalmanFilter.initStateTransitionMatrix();
    kalmanFilter.initControlMatrix();
    // kalmanFilter.initObservationErrorCov();
    std::cout << "Initial State : " << std::endl;
    printVector(kalmanFilter.getStateVector());

    std::vector<double> state = kalmanFilter.getStateVector();
    std::vector<double> estimation = {state[0], state[1], state[2]};
    std::cout << "Première estimation envoyée : " << std::endl;
    printVector(estimation);
    client.sendEstimation(estimation);

    while (true) {
        client.setIndex(0);
        while (true) {
            client.receive();
            if (client.getBuffer().find("MSG_END") != std::string::npos) {
                break;
            }
        }
        std::string buffer = client.getBuffer();
        std::cout << "Buffer : " << buffer << std::endl;

        std::map<std::string, std::vector<double> > data = parser.parseMessage(buffer);

        if (data.count("ACCELERATION"))
            kalmanFilter.setAcceleration(data["ACCELERATION"]);

        kalmanFilter.predictStateVector();

        if (data.count("GPS")) {
            kalmanFilter.update(data["GPS"]);
        }

        std::vector<double> state = kalmanFilter.getStateVector();
        std::vector<double> estimation = {state[0], state[1], state[2]};
        std::cout << "Estimation : " << std::endl;
        printVector(estimation);
        client.sendEstimation(estimation);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(client.getSockFd());
    return 0;
}
