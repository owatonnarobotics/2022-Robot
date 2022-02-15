#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "ShooterConsts.h"

class Intake {
   
    public:
        static Intake& GetInstance() {
            static Intake* instance = new Intake(R_IntakeCANID);
            return *instance;
        }

        void SetIntakeSpeed(const double &speedToSet) {
            m_intakeMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
        }
    
    private:
        Intake(const int R_IntakeCANID) {
            m_intakeMotor = new ctre::phoenix::motorcontrol::can::VictorSPX(R_IntakeCANID);
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_intakeMotor;
};
