#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "ShooterConsts.h"

class Climber {

    public:
        static Climber& GetInstance() {
            static Climber* instance = new Climber(R_ClimberCANID, R_ClimberCANID2, R_LimitSwitch);
            return *instance;
        }

        void SetClimberSpeed(const double &speedToSet) {

            frc::SmartDashboard::PutBoolean("ls", m_limitSwitch->Get());

            if (m_limitSwitch->Get() && speedToSet > 0) {

                m_ClimberMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
                m_ClimberMotor2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            }
            else {
                
                m_ClimberMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
                m_ClimberMotor2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
            }
        }
    
    private:
        Climber(const int R_ClimberCANID, const int R_ClimberCANID2, const int R_LimitSwitch) {
            m_ClimberMotor = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClimberCANID);
            m_ClimberMotor2 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClimberCANID2);
            m_limitSwitch = new frc::DigitalInput (R_LimitSwitch);
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClimberMotor;
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClimberMotor2;
        frc::DigitalInput *m_limitSwitch;
};
