// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <math.h>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <cameraserver/CameraServer.h>

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
#include "commonauto/steps/Stop.h"
#include "commonauto/steps/ResetNavXYaw.h"
#include "auto/SetIndexer.h"
#include "auto/SetShooter.h"
#include "auto/SetIntake.h"

frc::Joystick* playerOne;
frc::XboxController* playerTwo;

frc::SendableChooser<std::string>* autoChooser;
AutoSequence* bigSequence;

void Robot::RobotInit() {

    frc::CameraServer::StartAutomaticCapture();

    playerOne = new frc::Joystick(R_controllerPortPlayerOne);
    playerTwo = new frc::XboxController(R_controllerPortPlayerTwo);

    autoChooser = new frc::SendableChooser<std::string>;
    autoChooser->SetDefaultOption("tri-ball", "tb");
    frc::SmartDashboard::PutData(autoChooser);

    bigSequence = new AutoSequence(false);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {

    bigSequence->Reset();

    std::string selectedAuto = autoChooser->GetSelected();

    if (selectedAuto == "tb") {
        
        // TODO: figure out why the NavX needs to be reset twice after each
        // deploy. From what I have observed, the NavX recalibrates at the
        // beginning of the FIRST ENABLE after every deploy. I am not sure what
        // triggers the recalibration on the NavX, but from what I have read
        // online, the NavX needs to be calibrated before calling its ZeroYaw
        // function. It is possible that the reason it has to be called twice
        // is that on the first call, the NavX sees that it needs to be
        // initialized, does so, and moves on. Then, the second call to ZeroYaw
        // actually registers because the NavX has been calibrated. It seems
        // calling the ZeroYaw function, waiting a second, then calling
        // the ZeroYaw function again results in a successful reset of the yaw.
        // P.S. it is particularily interesting that the NavX ZeroYaw function
        // works on the first try whenever the number of enables since the last
        // deploy is greater than one.
        bigSequence->AddStep(new ResetNavXYaw);
        bigSequence->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(new ResetNavXYaw);

        // Spool up intake while driving to first cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new TimeDriveForwardHold(1.5));

        // After picking up the cargo, turn 180 degrees and drive back to 
        // optimal shooting range. Turn off the intake, as the cargo should be
        // full intaked by now.
        bigSequence->AddStep(new TurnToAbsoluteAngle(180));
        bigSequence->AddStep(new TimeDriveForwardHold(1.5));
        bigSequence->AddStep(new SetIntake(0));

        // Spool up launcher as we turn towards the goal
        bigSequence->AddStep(new SetShooter(R_shooterSpeed));
        bigSequence->AddStep(new TurnToAbsoluteAngle(190));

        // Shoot all the cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(3));

        // Don't waste voltage on the intake, indexer, and launcher while turning to the third
        // cargo.
        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new SetShooter(0));
        bigSequence->AddStep(new TurnToAbsoluteAngle(95));

        // Spool up the intake as we travel to pick up the third cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new TimeDriveForwardHold(2));

        // Spool up launcher as we turn towards the goal
        bigSequence->AddStep(new SetShooter(R_shooterSpeed));
        bigSequence->AddStep(new TurnToAbsoluteAngle(225));

        // Drive back to optimal shooting distance
        bigSequence->AddStep(new TimeDriveForwardHold(1.0));

        // Take one second to Limelight lock with the goal
        AsyncLoop* secondAimLoop = new AsyncLoop;
        secondAimLoop->AddStep(new LimelightLock);
        secondAimLoop->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(secondAimLoop);

        // Shoot the thrid cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(2));

        // Turn off intake, indexer, and launcher
        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new SetShooter(0));
    }

    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);

    // Make sure we stop moving when we're done with any sequence
    bigSequence->AddStep(new Stop);
    bigSequence->Init();
}

void Robot::AutonomousPeriodic() {

    bigSequence->Execute();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

    if (playerOne->GetRawButton(3)) {

        SwerveTrain::GetInstance().SetSoftwareZero();
    }
    if (playerOne->GetRawButton(9) && playerOne->GetRawButton(10)) {

        SwerveTrain::GetInstance().HardwareZero();
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
            playerOne->GetRawButton(2) ? Limelight::GetInstance().CalculateLimelightLockSpeed() : z,
            playerOne->GetRawButton(5),
            playerOne->GetRawButton(6),
            playerOne->GetRawButton(7),
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
