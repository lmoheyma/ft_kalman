#include "../inc/Parser.hpp"

Parser::Parser() : _hasMsgStarted(false)
{
}

Parser::~Parser()
{
}

std::vector<double> Parser::getInitialState(void) const
{
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
