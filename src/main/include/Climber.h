#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "ShooterConsts.h"

class Climber {

    public:
        static Climber& GetInstance() {
            static Climber* instance = new Climber(R_ClimberCANID, R_ClimberCANID2, R_LimitSwitch, R_ClimberSolenoid, R_LimitSwitch2);
            return *instance;
        }

        void SetClimberSpeed(const double &speedToSet, const bool& override = false) {

            frc::SmartDashboard::PutBoolean("Climber Bottomn limit switch state", m_limitSwitch->Get());
            frc::SmartDashboard::PutBoolean("Climber Top limit switch state", m_limitSwitch2->Get());

            if (!override && (m_limitSwitch->Get() && speedToSet > 0)) {

                m_ClimberMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
                m_ClimberMotor2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            }
            else if (!override && (!m_limitSwitch2->Get() && speedToSet < 0)) {

                m_ClimberMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
                m_ClimberMotor2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
            }
            else {
                
                m_ClimberMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
                m_ClimberMotor2->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
            }
        }
        void SwitchPneumatics(){
            m_ClimberSolenoid->Toggle();
        }
    
    private:
        Climber(const int R_ClimberCANID, const int R_ClimberCANID2, const int R_LimitSwitch, const int R_ClimberSolenoid, const int R_LimitSwitch2) {
            m_ClimberMotor = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClimberCANID);
            m_ClimberMotor2 = new ctre::phoenix::motorcontrol::can::VictorSPX(R_ClimberCANID2);
            m_limitSwitch = new frc::DigitalInput (R_LimitSwitch);
            m_ClimberSolenoid = new frc::Solenoid {frc::PneumaticsModuleType::CTREPCM, R_ClimberSolenoid};
            m_limitSwitch2 = new frc::DigitalInput (R_LimitSwitch2);
        }
        
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClimberMotor;
        ctre::phoenix::motorcontrol::can::VictorSPX *m_ClimberMotor2;
        frc::DigitalInput *m_limitSwitch;
        frc::Solenoid *m_ClimberSolenoid;
        frc::DigitalInput *m_limitSwitch2;
};


