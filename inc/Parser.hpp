#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cctype>
#include <cstdlib>

enum Mesures {
  ACCELERATION,
  DIRECTION,
  SPEED,
  TRUE_POSITION
};

class Parser
{
    private:
        std::string _message;
        // bool _hasMsgStarted;
        std::vector<double> _initialState;
    public:
        Parser();
        ~Parser();
        std::vector<double> createInitialState(std::map<std::string, std::vector<double> > data);
        std::map<std::string, std::vector<double> > parseMessage(const std::string& message);
};
