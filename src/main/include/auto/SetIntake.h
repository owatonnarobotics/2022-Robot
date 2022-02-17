#pragma once

#include "commonauto/AutoStep.h"

#include "Intake.h"

class SetIntake : public AutoStep {

    public:
        SetIntake(const double speed) : AutoStep("SetIntake") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Intake::GetInstance().SetIntakeSpeed(m_speed);
            return true;
        }

    private:
        double m_speed;
};