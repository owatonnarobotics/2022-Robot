#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/motorcontrol/VictorSP.h>
#include "ShooterConsts.h"

class Shooter {
    
    public:
        static Shooter& GetInstance() {
            static Shooter* instance = new Shooter(R_ShooterCANID, R_SpinnerCANID);
            return *instance;
        }

        void SetShooterSpeed(const double &velocityToSet) {
            m_shooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, -velocityToSet);
        }
        void SetSpinSpeed(const double &velocityToSet){
            m_spinnerMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, velocityToSet);
        }

        double GetVelocity() {

            return m_shooterMotor->GetSelectedSensorVelocity();
        }
        double GetVelocitySpinner() {

            return m_spinnerMotor->GetSelectedSensorVelocity();
        }

    private:
        Shooter(const int R_ShooterCANID, const int R_SpinnerCANID) {
            m_shooterMotor = new ctre::phoenix::motorcontrol::can::TalonFX(R_ShooterCANID);
            m_spinnerMotor = new ctre::phoenix::motorcontrol::can::TalonFX(R_SpinnerCANID);

            m_shooterMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
            m_spinnerMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
        }
        ctre::phoenix::motorcontrol::can::TalonFX *m_shooterMotor; //joe's contribution
        ctre::phoenix::motorcontrol::can::TalonFX *m_spinnerMotor;
};
