#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cctype>
#include <cstdlib>

class Parser
{
    private:
        std::string _message;
        bool _hasMsgStarted;
        std::vector<double> _initialState;
    public:
        Parser();
        ~Parser();
        std::map<std::string, std::vector<double> > parseMessage(const std::string& message);
        std::vector<double> getInitialState(void) const;
};