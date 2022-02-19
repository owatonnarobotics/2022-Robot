#pragma once

#include "commonauto/AutoStep.h"

#include "Shooter.h"

class SetShooter : public AutoStep {

    public:
        SetShooter(const double speed) : AutoStep("SetShooter") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Shooter::GetInstance().SetShooterSpeed(m_speed);
            return true;
        }

    private:
        double m_speed;
};