// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <math.h>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include "swerve/src/include/SwerveTrain.h"
#include "controller/Controller.h"
#include "limelight/Limelight.h"
#include "Shooter.h"
#include "ShooterConsts.h"
#include "Indexer.h"
#include "Intake.h"

#include "commonauto/AutoSequence.h"
#include "commonauto/steps/WaitSeconds.h"
#include "commonauto/steps/TimeDriveForwardHold.h"
#include "commonauto/steps/TurnToAbsoluteAngle.h"
#include "commonauto/steps/LimelightLock.h"
#include "auto/SetIndexer.h"
#include "auto/SetShooter.h"
#include "auto/SetIntake.h"

frc::Joystick* playerOne;
frc::XboxController* playerTwo;

frc::SendableChooser<std::string>* autoChooser;
AutoSequence* bigSequence;

void Robot::RobotInit() {

    playerOne = new frc::Joystick(R_controllerPortPlayerOne);
    playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);

    autoChooser = new frc::SendableChooser<std::string>;
    autoChooser->AddOption("Red tri-ball", "rtb");
    autoChooser->SetDefaultOption("Blue tri-ball", "btb");
    frc::SmartDashboard::PutData(autoChooser);

    bigSequence = new AutoSequence(false);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {

    bigSequence->Reset();

    std::string selectedAuto = autoChooser->GetSelected();

    if (selectedAuto == "rtb") {
    
        bigSequence->AddStep(new SetShooter(R_shooterSpeed));
        bigSequence->AddStep(new SetIntake(1));
        bigSequence->AddStep(new TimeDriveForwardHold(1.5));
        bigSequence->AddStep(new TurnToAbsoluteAngle(180));
        bigSequence->AddStep(new TimeDriveForwardHold(1.0));
        bigSequence->AddStep(new SetIntake(0));
        
        AsyncLoop* aimLoop = new AsyncLoop;
        aimLoop->AddStep(new LimelightLock);
        aimLoop->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(aimLoop);
        
        bigSequence->AddStep(new SetIntake(1));
        bigSequence->AddStep(new SetIndexer(1));
        bigSequence->AddStep(new WaitSeconds(3));
        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));

        bigSequence->AddStep(new TurnToAbsoluteAngle(100));
        bigSequence->AddStep(new SetIntake(1));
        bigSequence->AddStep(new TimeDriveForwardHold(2));
        bigSequence->AddStep(new TurnToAbsoluteAngle(270));

        AsyncLoop* secondAimLoop = new AsyncLoop;
        secondAimLoop->AddStep(new LimelightLock);
        secondAimLoop->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(aimLoop);

        bigSequence->AddStep(new SetIntake(1));
        bigSequence->AddStep(new SetIndexer(1));
        bigSequence->AddStep(new WaitSeconds(2));
        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new SetShooter(0));
    }

    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);
    NavX::GetInstance().resetYaw();
    SwerveTrain::GetInstance().SetZeroPosition();

    bigSequence->Init();
}

void Robot::AutonomousPeriodic() {

    if (bigSequence->Execute()) {

        SwerveTrain::GetInstance().AssumeZeroPosition();
    }
}

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

    if (playerTwo->GetRightTriggerAxis() < 0.2 ){
        Shooter::GetInstance().SetShooterSpeed(0);
    }


    else if(( 0.2 <= playerTwo->GetRightTriggerAxis()) && (playerTwo->GetRightTriggerAxis() < 0.4 )){
       Shooter::GetInstance().SetShooterSpeed(-0.85);
    }

    else if(( 0.4 <= playerTwo->GetRightTriggerAxis()) && (playerTwo->GetRightTriggerAxis() < 0.9 )){
       Shooter::GetInstance().SetShooterSpeed(-0.92);
    }

    else if( 0.9 <= playerTwo->GetRightTriggerAxis()){
       Shooter::GetInstance().SetShooterSpeed(-1);
    }


    if (playerTwo->GetRightBumper()){
        Indexer::GetInstance().SetIndexerSpeed(-.5);
    }
    else if (playerTwo->GetLeftBumper()){
       Indexer::GetInstance().SetIndexerSpeed(.5);
    
       
    }
    else{
        Indexer::GetInstance().SetIndexerSpeed(0);
    }


    if(playerTwo->GetLeftTriggerAxis() > .5){
        Intake::GetInstance().SetIntakeSpeed(-.5);
    }
    else if(playerTwo->GetBButton()){
        Intake::GetInstance().SetIntakeSpeed(.5);
    }
    else {
        Intake::GetInstance().SetIntakeSpeed(0);
    }
    
    

    /* Baseline code for a 4-stepped launcher instead a linear or parabolic launcher*/
    /* "Just something I wanted to try" - Aids*/

    
    
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {

    SwerveTrain::GetInstance().SetSwerveBrake(false);
    SwerveTrain::GetInstance().SetDriveBrake(false);
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { 
    return frc::StartRobot<Robot>();
}
#endif
