#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "ShooterConsts.h"

class Climber {

    public:
        static Climber& GetInstance() {
            static Climber* instance = new Climber(R_ClimberCANID, R_ClimberCANID2);
            return *instance;
        }

        void SetClimberSpeed(const double &speedToSet) {
            m_ClimberMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
            m_ClimberMotor2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
        }
    
    private:
        Climber(const int R_ClimberCANID, const int R_ClimberCANID2) {
            m_ClimberMotor = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClimberCANID);
            m_ClimberMotor2 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClimberCANID2);
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClimberMotor;
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClimberMotor2;
};
