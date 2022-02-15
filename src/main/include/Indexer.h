#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include "ShooterConsts.h"

class Indexer{

    public:
        static Indexer& GetInstance() {
            static Indexer* instance = new Indexer(R_IndexerCANID);
            return *instance;
        }
    
        void SetIndexerSpeed(const double &speedToSet) {
            m_indexerMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
        }

    private:
        Indexer(const int R_IndexerCANID) {
            m_indexerMotor = new ctre::phoenix::motorcontrol::can::VictorSPX(R_IndexerCANID);
        }
    
    ctre::phoenix::motorcontrol::can::VictorSPX *m_indexerMotor;

};