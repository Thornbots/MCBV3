#pragma once

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"
#include "time.h"

using namespace subsystems;

class TorqueIndicator : public GraphicsContainer {
public:
    TorqueIndicator(src::Drivers* drivers, tap::motor::DjiMotor* motor, uint8_t index) : 
    drivers(drivers),
    motor(motor),
    index(index) {
        addGraphicsObject(&number);
        if(index == 1){
            number.x = 1070;
            number.y = 360;

            // Only display rpm of one flywheel
            for (int i = 0; i < graphelements; i++)
            {addGraphicsObject(&graph[i]);}
        } else if(index == 2){
            number.x = 660;
            number.y = 360;
        } else if(index == 3){
            number.x = 1070;
            number.y = 670;
        } else if(index == 4){
            number.x = 660;
            number.y = 670;
        }

        
    }

    bool previousL = false;

    void update() {
        int16_t curvalue = motor->getTorque();
        
        if(drivers->remote.keyPressed(Remote::Key::R)) {
            maxvalue = curvalue;
        }
        
        int16_t absmax = maxvalue<0 ? -maxvalue : maxvalue;
        int16_t abscur = curvalue<0 ? -curvalue : curvalue;
        
        if(abscur>absmax) maxvalue = curvalue;
        
        //number._float = maxvalue;
        
        number.color = motor->isMotorOnline() ? UISubsystem::Color::GREEN : UISubsystem::Color::PINK;


        // Left Click Pressed - fired reset average
        bool currentL = drivers->remote.getMouseL();
        if(!currentL) previousL = false;

        if(currentL && !previousL){
            previousL = true;
            averageRPM = 0;
            numberofMeasurements = 0;
            graphindex = 0;
        }
        
        // Display Average RPM value
        // Compare value with a projectile and without to determine if there's a noticable difference in the flywheel's rpm
        else
        {
            averageRPM =+ motor->getShaftRPM();
            numberofMeasurements++;
            number._float = (float)averageRPM/(float)numberofMeasurements;
        }


        // Plot Values
        // view rpm values as the flywheel shoots a projectile
        if (graphindex < graphelements)
        {
            // Reset row to top once it reaches the bottom of the screen
            if (graph_yheight < 250)
            {graph_yheight = 848;}

            // 15 rows, shift it over if it's past 15 rows
            graph[graphindex].x = 500 + 100*(floor(graphindex/15.0));
            graph[graphindex].y = graph_yheight;

            graph[graphindex].height = 40;
            graph[graphindex].thickness = 4;

            graph[graphindex]._float = motor->getShaftRPM();

            graph_yheight =- 40;

            graphindex++;
        }

        
    }

private:
    src::Drivers* drivers;
    tap::motor::DjiMotor* motor;
    uint8_t index;
    int16_t maxvalue = 0;


    int16_t averageRPM = 0;
    int numberofMeasurements = 0;
    //int startTime = 0;


    static constexpr uint16_t X_POSITION = 1680; 
    static constexpr uint16_t Y_POSITION = 610; 

    FloatGraphic number{UISubsystem::Color::GREEN, 0, 0, 0, 60, 3};

    // Graph Flywheel Speed
    int graph_yheight = 0;

    int graphindex = 0;
    static constexpr int graphelements = 60;
    FloatGraphic graph[graphelements];
};