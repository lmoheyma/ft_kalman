#include "../inc/Client.hpp"
#include "../inc/Parser.hpp"
#include "../inc/KalmanFilter.hpp"
#include <thread>
#include <chrono>
#include <cmath>


double calculateDistance(const std::vector<double>& pos1, const std::vector<double>& pos2) {
    double dx = pos1[0] - pos2[0];
    double dy = pos1[1] - pos2[1];
    double dz = pos1[2] - pos2[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void printData(const std::map<std::string, std::vector<double>>& data) {
    for (const auto& pair : data) {
        std::cout << pair.first << " : ";
        for (double val : pair.second) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}

int main() {
    Client client;
    Parser parser;

    client.init();
    client.sendFirstMessage();

    while (true) {
        client.receiveFirstMessage();
        if (client.getBuffer().find("MSG_END") != std::string::npos) {
            break;
        }
    }

    std::map<std::string, std::vector<double> > parsed = parser.parseMessage(client.getBuffer());
    
    std::cout << "=== DONNÉES INITIALES REÇUES ===" << std::endl;
    printData(parsed);
    std::cout << "=================================" << std::endl;
    
    KalmanFilter kalmanFilter;
    
    // L'accélération est déjà dans le repère XYZ (world frame) selon le sujet
    std::vector<double> initial_acc = parsed.at("ACCELERATION");
    kalmanFilter.setAcceleration(initial_acc);
    kalmanFilter.setStateVector(parser.createInitialState(parsed));
    kalmanFilter.initProcessNoiseMatrix();
    kalmanFilter.initCovarianceMatrix();
    kalmanFilter.initMeasurementMatrix();
    kalmanFilter.initStateTransitionMatrix();
    kalmanFilter.initControlMatrix();
    kalmanFilter.initUncertaintyMatrix();
    // kalmanFilter.initObservationErrorCov();
    std::cout << "Initial State : " << std::endl;
    printVector(kalmanFilter.getStateVector());

    std::vector<double> state = kalmanFilter.getStateVector();
    std::vector<double> estimation = {state[0], state[1], state[2]};
    std::cout << "Première estimation envoyée : " << std::endl;
    printVector(estimation);
    client.sendEstimation(estimation);

    try {
        std::vector<double> direction(3, 0.0);
        if (parsed.count("DIRECTION")) {
            direction = parsed["DIRECTION"];
        }

        while (true) {
            client.setIndex(0);
            while (true) {
                client.receive();
                if (client.getBuffer().find("MSG_END") != std::string::npos) {
                    break;
                }
            }
            std::string buffer = client.getBuffer();
            std::map<std::string, std::vector<double> > data = parser.parseMessage(buffer);

            if (data.count("DIRECTION")) {
                direction = data["DIRECTION"];
            }

            if (data.count("ACCELERATION")) {
                // L'accélération est déjà dans le repère XYZ (world frame)
                kalmanFilter.setAcceleration(data["ACCELERATION"]);
            }

            kalmanFilter.predictStateVector();

            std::vector<double> state = kalmanFilter.getStateVector();
            std::vector<double> estimation = {state[0], state[1], state[2]};

            // Mettre à jour avec POSITION
            if (data.count("POSITION")) {
                kalmanFilter.update(data["POSITION"]);
                // Recalculer l'état après la mise à jour
                state = kalmanFilter.getStateVector();
                estimation = {state[0], state[1], state[2]};
            }
            client.sendEstimation(estimation);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception : " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Exception inconnue" << std::endl;
    }

    close(client.getSockFd());
    return 0;
}
