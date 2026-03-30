#include "../inc/Client.hpp"
#include "../inc/Parser.hpp"
#include "../inc/KalmanFilter.hpp"
#include "../inc/colors.hpp"
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>


void printEstimation(const std::vector<double>& estimation, double t, bool gps_update) {
    std::cout << std::fixed << std::setprecision(4);
    if (gps_update) {
        std::cout << "\033[2K\r" << GREEN << BOLD
                  << "[UPDATE] "
                  << RESET << GREEN
                  << "t=" << std::setw(8) << t << "s"
                  << "  x=" << std::setw(11) << estimation[0]
                  << "  y=" << std::setw(11) << estimation[1]
                  << "  z=" << std::setw(11) << estimation[2]
                  << RESET << std::endl;
    } else {
        std::cout << CYAN
                  << "[PREDICT] "
                  << "t=" << std::setw(8) << t << "s"
                  << "  x=" << std::setw(11) << estimation[0]
                  << "  y=" << std::setw(11) << estimation[1]
                  << "  z=" << std::setw(11) << estimation[2]
                  << RESET << "\r" << std::flush;
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

    std::vector<double> state = kalmanFilter.getStateVector();
    std::vector<double> estimation = {state[0], state[1], state[2]};
    client.sendEstimation(estimation);
    std::cout << BOLD << "ft_kalman started" << RESET << " — " << CYAN << "PREDICT" << RESET << " / " << GREEN << BOLD << "UPDATE (GPS)" << RESET << std::endl;
    std::cout << std::string(72, '-') << std::endl;

    try {
        double current_time = 0.0;
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

            bool gps_update = false;
            // Mettre à jour avec POSITION
            if (data.count("POSITION")) {
                kalmanFilter.update(data["POSITION"]);
                // Recalculer l'état après la mise à jour
                state = kalmanFilter.getStateVector();
                estimation = {state[0], state[1], state[2]};
                gps_update = true;
            }
            client.sendEstimation(estimation);
            printEstimation(estimation, current_time, gps_update);

            current_time += DELTA_T;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception& e) {
        if (std::string(e.what()) != "GOODBYE")
            std::cerr << "Exception : " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Exception inconnue" << std::endl;
    }

    close(client.getSockFd());
    return 0;
}
