// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <math.h>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

#include "swerve/src/include/SwerveTrain.h"
#include "controller/Controller.h"
#include "limelight/Limelight.h"

frc::Joystick* playerOne;

void Robot::RobotInit() {

    playerOne = new frc::Joystick(R_controllerPortPlayerOne);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

    if (playerOne->GetRawButton(3)) {

        SwerveTrain::GetInstance().SetZeroPosition();
    }
    if (playerOne->GetRawButton(4)) {

        NavX::GetInstance().resetYaw();
    }
    if (playerOne->GetRawButton(12)) {
        
        SwerveTrain::GetInstance().AssumeZeroPosition();
    }
    else {

        double x = playerOne->GetX();
        double y = playerOne->GetY();
        double z = playerOne->GetZ();
        Controller::forceControllerXYZToZeroInDeadzone(x, y, z);

        frc::SmartDashboard::PutNumber("x", x);
        frc::SmartDashboard::PutNumber("y", y);
        frc::SmartDashboard::PutNumber("z", z);

        SwerveTrain::GetInstance().Drive(
            -x,
            -y,
            playerOne->GetRawButton(6) ? Limelight::GetInstance().CalculateLimelightLockSpeed() : z,
            playerOne->GetRawButton(5),
            playerOne->GetRawButton(7),
            playerOne->GetRawButton(2),
            -(((playerOne->GetThrottle() + 1.0) / 2.0) - 1.0)
        );
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
