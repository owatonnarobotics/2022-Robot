#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/motorcontrol/VictorSP.h>
#include "ShooterConsts.h"

class Shooter {
    
    public:
        static Shooter& GetInstance() {
            static Shooter* instance = new Shooter(R_ShooterCANID);
            return *instance;
        }

        void SetShooterSpeed(const double &speedToSet) {
            m_shooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -speedToSet);
        }

        double GetVelocity() {

            return m_shooterMotor->GetSelectedSensorVelocity();
        }

    private:
        Shooter(const int R_ShooterCANID) {
            m_shooterMotor = new ctre::phoenix::motorcontrol::can::TalonFX(R_ShooterCANID);
        
        }
        ctre::phoenix::motorcontrol::can::TalonFX *m_shooterMotor; //joe's contribution
        

};