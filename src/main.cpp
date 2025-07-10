#include "../inc/Client.hpp"
#include "../inc/Parser.hpp"
#include "../inc/KalmanFilter.hpp"
#include <thread>
#include <chrono>


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

            std::cout << "--- Données reçues ---" << std::endl;
            printData(data);

            // b) Mettre à jour l'accélération en tenant compte de la direction si présente
            if (data.count("DIRECTION") && data.count("ACCELERATION")) {
                std::vector<double> direction = data["DIRECTION"];
                Matrix Rx(3, std::vector<double>(3, 0.0));
                Matrix Ry(3, std::vector<double>(3, 0.0));
                Matrix Rz(3, std::vector<double>(3, 0.0));
                setRotationX(Rx, direction[0]);
                setRotationY(Ry, direction[1]);
                setRotationZ(Rz, direction[2]);
                
                Matrix R = multiply(Rz, Ry);
                R = multiply(R, Rx);  // donc : R = Rz * Ry * Rx
                std::vector<double> acc_global = multiplyMatrixVector(R, data["ACCELERATION"]);
                kalmanFilter.setAcceleration(acc_global);
            }

            kalmanFilter.predictStateVector();

            std::cout << data.count("TRUE_POSITION") << std::endl;
            if (data.count("POSITION") || data.count("TRUE_POSITION")) {
                if (data.count("POSITION")) {
                    std::cout << "Update with position" << std::endl;
                    kalmanFilter.update(data["POSITION"]);
                }
                else {
                    std::cout << "Update with true position" << std::endl;
                    kalmanFilter.update(data["TRUE_POSITION"]);
                }
            }

            std::cout << "Accélération utilisée : ";
            printVector(kalmanFilter.getAcceleration());
            std::cout << "Direction utilisée : ";
            printVector(data["DIRECTION"]);
            std::cout << "Etat estimé : ";
            printVector(kalmanFilter.getStateVector());
            std::cout << "Diagonale de P : ";
            for (size_t i = 0; i < kalmanFilter.getP().size(); ++i)
                std::cout << kalmanFilter.getP()[i][i] << " ";
            std::cout << std::endl;

            std::vector<double> state = kalmanFilter.getStateVector();
            std::vector<double> estimation = {state[0], state[1], state[2]};
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
