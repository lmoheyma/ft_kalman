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

    // Delete les logs
    std::ofstream clearfile("logs.txt", std::ios::trunc);
    clearfile.close();

    while (true) {
        client.receive_first_message();
        if (client.getBuffer().find("MSG_END") != std::string::npos) {
            break;
        }
    }

    std::map<std::string, std::vector<double> > parsed = parser.parseMessage(client.getBuffer());
    
    std::cout << "=== DONNÉES INITIALES REÇUES ===" << std::endl;
    printData(parsed);
    std::cout << "=================================" << std::endl;
    
    KalmanFilter kalmanFilter;
    
    // NE PAS transformer l'accélération initiale
    // Le véhicule se déplace selon son axe longitudinal dans son propre repère
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
        double current_time = 0.0;

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

            // NE PAS transformer l'accélération
            // Le véhicule se déplace selon son axe longitudinal dans son propre repère
            if (data.count("ACCELERATION")) {
                std::cout << "Acceleration (repère véhicule): " << std::endl;
                printVector(data["ACCELERATION"]); 
                kalmanFilter.setAcceleration(data["ACCELERATION"]);
            }

            kalmanFilter.predictStateVector();

            std::vector<double> state = kalmanFilter.getStateVector();
            std::vector<double> estimation = {state[0], state[1], state[2]};
            
            // Vérification de l'erreur AVANT la mise à jour GPS
            if (data.count("TRUE_POSITION")) {
                double error = calculateDistance(estimation, data["TRUE_POSITION"]);
                std::cout << "Erreur vs TRUE_POSITION : " << error << "m" << std::endl;
                if (error > 5.0) {
                    std::cerr << "ERREUR CRITIQUE : Distance > 5m (" << error << "m)" << std::endl;
                    std::cerr << "Position estimée : [" << estimation[0] << ", " << estimation[1] << ", " << estimation[2] << "]" << std::endl;
                    std::cerr << "Position réelle : [" << data["TRUE_POSITION"][0] << ", " << data["TRUE_POSITION"][1] << ", " << data["TRUE_POSITION"][2] << "]" << std::endl;
                    break;
                }
            }

            // Ne mettre à jour qu'avec POSITION (GPS bruité), jamais TRUE_POSITION
            if (data.count("POSITION")) {
                std::cout << "Update with GPS position" << std::endl;
                kalmanFilter.update(data["POSITION"]);
                // Recalculer l'état après la mise à jour
                state = kalmanFilter.getStateVector();
                estimation = {state[0], state[1], state[2]};
            }
            
            client.sendEstimation(estimation);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Logging pour debug
            double pos_est_x = state[0];
            double vel_x = state[3];
            double pos_gps_x = 0.0;
            // double pos_true_x = 0.0;
            bool has_gps = data.count("POSITION") > 0;
            
            if (has_gps)
                pos_gps_x = data["POSITION"][0];
            // if (data.count("TRUE_POSITION"))
            //     pos_true_x = data["TRUE_POSITION"][0];

            std::ofstream logfile("logs.txt", std::ios::app);
            if (logfile.is_open()) {
                logfile << "t=" << current_time
                        << " POS_EST=" << pos_est_x
                        << " POS_EST_Y=" << state[1]
                        << " POS_EST_Z=" << state[2]
                        << " VEL_X=" << vel_x
                        << " VEL_Y=" << state[4]
                        << " VEL_Z=" << state[5]
                        << " POS_GPS=" << pos_gps_x;
                
                if (has_gps) {
                    logfile << " GPS_Y=" << data["POSITION"][1]
                            << " GPS_Z=" << data["POSITION"][2];
                }
                
                logfile << " GPS=" << (has_gps ? "1" : "0")
                        << std::endl;
                logfile.close();
            }

            current_time += DELTA_T;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception : " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Exception inconnue" << std::endl;
    }

    close(client.getSockFd());
    return 0;
}
