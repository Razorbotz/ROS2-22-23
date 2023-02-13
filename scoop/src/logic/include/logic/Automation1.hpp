#pragma once

#include "Automation.hpp"

class Automation1 : public Automation{

    enum RobotState{LOCATE,ALIGN,GO_TO_DIG_SITE,DIG,GO_TO_HOME,DOCK,DUMP,RETURN_TO_START};
    RobotState robotState=LOCATE;
    Location destination;

    void automate();

};
