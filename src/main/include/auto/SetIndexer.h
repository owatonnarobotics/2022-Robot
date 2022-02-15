#pragma once

#include "commonauto/AutoStep.h"

#include "Indexer.h"

class SetIndexer : public AutoStep {

    public:
        SetIndexer(const double speed) : AutoStep("SetIndexer") {

            m_speed = speed;
        }

        void Init() {}

        bool Execute() {

            Indexer::GetInstance().SetIndexerSpeed(m_speed);
            return true;
        }

    private:
        double m_speed;
};