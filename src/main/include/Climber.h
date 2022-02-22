#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "ShooterConsts.h"

class Climber {

    public:
        static Climber& GetInstance() {
            static Climber* instance = new Climber(R_ClimberCANID);
            return *instance;
        }

        void SetClimberSpeed(const double &speedToSet) {
            m_ClimberMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
        }
    
    private:
        Climber(const int R_ClimberCANID) {
            m_ClimberMotor = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClimberCANID);
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClimberMotor;
};
