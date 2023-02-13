
#include <cmath>
#include <ctime>

#include "logic/Automation.hpp"
#include "logic/Automation1.hpp"


void Automation1::automate(){
    // Initially start with locating the Aruco marker
    // Turn slowly until it's seen
    if(robotState==LOCATE){
        changeSpeed(0.15,-0.15);
        if(position.arucoVisible==true){
            robotState=ALIGN;
            destination.x=-5;
            destination.z=2;
            changeSpeed(0,0);
        }
    }

    // After finding the Aruco marker, turn the bot to 
    // align with the arena
    if(robotState==ALIGN){

        //robotState = GO_TO_DIG_SITE;
    }

    // After aligning with the arena, navigate to the 
    // excavation area
    if(robotState==GO_TO_DIG_SITE){
        
        //robotState = DIG;
    }

    // After reaching teh excavation area, go through mining
    // sequence
    if(robotState==DIG){
        
        //robotState = GO_TO_HOME;
    }

    // After mining, return to start position
    if(robotState==GO_TO_HOME){

        //robotState = DOCK;
    }

    // After reaching start position, dock at dump bin
    if(robotState==DOCK){

        //robotState = DUMP;
    }

    // Dump the collected rocks in the dump bin
    if(robotState==DUMP){
        
        //robotState = RETURN_TO_START;
    }

    // After dumping the rocks, return to start position and
    // start again
    if(robotState==RETURN_TO_START){

        //robotState = ALIGN;
    }
    else{
        changeSpeed(0,0);
    }
}
    
