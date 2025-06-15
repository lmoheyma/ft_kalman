#include "../inc/Parser.hpp"
#include "../inc/maths.hpp"

Parser::Parser()
{
}

Parser::~Parser()
{
}

std::vector<double> Parser::createInitialState(std::map<std::string, std::vector<double> > data)
{
    try
    {
        std::vector<double> position = data.at("TRUE_POSITION");
        std::vector<double> speed = data.at("SPEED");
        std::vector<double> direction = data.at("DIRECTION");

        Matrix Rx(3, std::vector<double>(3, 0.0));
        Matrix Ry(3, std::vector<double>(3, 0.0));
        Matrix Rz(3, std::vector<double>(3, 0.0));

        speed[0] /= 3.6; // convert km/h to m/s
        for (int i = 0; i < 2; i++) speed.push_back(0.0);

        setRotationX(Rx, direction[0]);
        setRotationY(Ry, direction[1]);
        setRotationZ(Rz, direction[2]);
        Matrix R = multiply(Rx, Ry);
        R = multiply(R, Rz);
        speed = multiplyMatrixVector(R, speed);

        this->_initialState.insert(this->_initialState.begin(), position.begin(), position.end());
        for (auto i : speed)
            this->_initialState.push_back(i);
        return this->_initialState;
    }
    catch(const std::out_of_range& e)
    {
        std::cerr << e.what() << '\n';
    }

    std::map<std::string, std::vector<double> >::iterator it;

    for (it = data.begin(); it != data.end(); ++it) {
        // if (it->first)
        std::cout << it->first << " : ";
        std::vector<double>::iterator vit;
        for (vit = it->second.begin(); vit != it->second.end(); ++vit) {
            std::cout << *vit << " ";
        }
        std::cout << std::endl;
    }
    return this->_initialState;
}

std::map<std::string, std::vector<double> > Parser::parseMessage(const std::string& message)
{
    std::map<std::string, std::vector<double> > data;
    std::istringstream iss(message);
    std::string line;
    std::string currentLabel;

    while (std::getline(iss, line)) {
        if (line.empty() || line == "MSG_END")
            continue;

        if (line[0] == '[') {
            std::size_t pos = line.find(']');
            if (pos != std::string::npos) {
                currentLabel = line.substr(pos + 1);
                for (size_t i = 0; i < currentLabel.size(); ++i) {
                    if (currentLabel[i] == ' ') currentLabel[i] = '_';
                }
                data[currentLabel] = std::vector<double>();
            }
        }
        else {
            char* end;
            double value = std::strtod(line.c_str(), &end);
            if (end != line.c_str()) {
                data[currentLabel].push_back(value);
            } else {
                std::cerr << "Erreur : valeur non numérique -> " << line << std::endl;
            }
        }
    }
    return data;
}
