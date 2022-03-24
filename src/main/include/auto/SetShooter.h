#pragma once

#include "commonauto/AutoStep.h"

#include "Shooter.h"

class SetShooter : public AutoStep {

    public:
        SetShooter(const double shooter, const double spinner) : AutoStep("SetShooter") {

            m_shooter = shooter;
            m_spinner = spinner;
        }

        void Init() {}

        bool Execute() {

            Shooter::GetInstance().SetShooterSpeed(m_shooter);
            Shooter::GetInstance().SetSpinSpeed(m_spinner);
            return true;
        }

    private:
        double m_shooter;
        double m_spinner;
};