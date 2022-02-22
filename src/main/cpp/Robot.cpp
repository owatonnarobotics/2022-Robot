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
#include "Shooter.h"
#include "ShooterConsts.h"
#include "Indexer.h"
#include "Intake.h"
#include "Climber.h"

#include "commonauto/AutoSequence.h"
#include "commonauto/steps/WaitSeconds.h"
#include "commonauto/steps/TimeDriveHold.h"
#include "commonauto/steps/TurnToAbsoluteAngle.h"
#include "commonauto/steps/Stop.h"
#include "commonauto/steps/ResetNavXYaw.h"
#include "commonauto/steps/CalibrateNavXThenReset.h"
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
    autoChooser->AddOption("one ball", "1b");
    autoChooser->AddOption("two ball top", "2bt");
    autoChooser->AddOption("two ball middle", "2bm");
    autoChooser->AddOption("two ball rungs", "2br");
    autoChooser->SetDefaultOption("tri-ball", "tb");
    frc::SmartDashboard::PutData(autoChooser);

    bigSequence = new AutoSequence(false);
    bigSequence->EnableLogging();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {

    SwerveTrain::GetInstance().ResetHold();
    // TODO: NEEDS TO CHANGE
    SwerveTrain::GetInstance().HardwareZero();

    bigSequence->Reset();

    std::string selectedAuto = autoChooser->GetSelected();

    if (selectedAuto == "1b") {

        bigSequence->AddStep(new CalibrateNavXThenReset);
        bigSequence->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(new ResetNavXYaw);

        bigSequence->AddStep(new SetShooter(R_shooterSpeed));
        bigSequence->AddStep(new WaitSeconds(3));

        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(2));

        bigSequence->AddStep(new SetShooter(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new TimeDriveHold(0, -1, 2.0));
    }
    else if (selectedAuto == "2bt") {

        bigSequence->AddStep(new CalibrateNavXThenReset);
        bigSequence->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(new ResetNavXYaw);

        // Spool up intake while driving to first cargo and wait a second
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new TimeDriveHold(0, 1, 1.9));
        bigSequence->AddStep(new WaitSeconds(1));

        // After picking up the cargo, turn 180 degrees and drive back to 
        // optimal shooting range. Turn off the intake, as the cargo should be
        // full intaked by now.
        bigSequence->AddStep(new TurnToAbsoluteAngle(180));
        bigSequence->AddStep(new TimeDriveHold(0, -1, 1.8));
        bigSequence->AddStep(new SetIntake(0));

        // Spool up launcher as we turn towards the goal
        bigSequence->AddStep(new SetShooter(R_shooterSpeed));
        bigSequence->AddStep(new TurnToAbsoluteAngle(195));

        // Shoot all the cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(0.3));

        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new WaitSeconds(1));

        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(3));

        // Spool down
        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new SetShooter(0));
    }
    else if (selectedAuto == "2bm") {

        bigSequence->AddStep(new CalibrateNavXThenReset);
        bigSequence->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(new ResetNavXYaw);

        // Spool up intake while driving to first cargo and wait a second
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new TimeDriveHold(0, 1, 1.9));
        bigSequence->AddStep(new WaitSeconds(1));

        // After picking up the cargo, turn 180 degrees and drive back to 
        // optimal shooting range. Turn off the intake, as the cargo should be
        // full intaked by now.
        bigSequence->AddStep(new TurnToAbsoluteAngle(180));
        bigSequence->AddStep(new TimeDriveHold(0, -1, 1.8));
        bigSequence->AddStep(new SetIntake(0));

        // Spool up launcher as we turn towards the goal
        bigSequence->AddStep(new SetShooter(R_shooterSpeed));
        bigSequence->AddStep(new TurnToAbsoluteAngle(160));

        // Shoot all the cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(0.3));

        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new WaitSeconds(1));

        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(3));

        // Spool down
        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new SetShooter(0));
    }
    else if (selectedAuto == "2br") {

        bigSequence->AddStep(new CalibrateNavXThenReset);
        bigSequence->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(new ResetNavXYaw);

        // Spool up intake while driving to first cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new TimeDriveHold(0, 1, 1.8));

        // After picking up the cargo, turn 180 degrees and drive back to 
        // optimal shooting range. Turn off the intake, as the cargo should be
        // full intaked by now.
        bigSequence->AddStep(new TurnToAbsoluteAngle(180));
        bigSequence->AddStep(new TimeDriveHold(0, -1, 1.8));
        bigSequence->AddStep(new SetIntake(0));

        // Spool up launcher as we turn towards the goal
        bigSequence->AddStep(new SetShooter(R_shooterSpeed * 0.95));
        bigSequence->AddStep(new TurnToAbsoluteAngle(170));

        // Shoot all the cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(3));

        // Spool down
        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new SetShooter(0));
    }
    else if (selectedAuto == "tb") {
        
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
        bigSequence->AddStep(new CalibrateNavXThenReset);
        bigSequence->AddStep(new WaitSeconds(1));
        bigSequence->AddStep(new ResetNavXYaw);

        bigSequence->AddStep(new SetShooter(R_shooterSpeed * 0.9));
        bigSequence->AddStep(new TimeDriveHold(0, -1, 1));
        bigSequence->AddStep(new WaitSeconds(0.125));
        bigSequence->AddStep(new TurnToAbsoluteAngle(15));

        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(0.25));
        bigSequence->AddStep(new SetShooter(0));
        bigSequence->AddStep(new SetIndexer(0));

        bigSequence->AddStep(new TurnToAbsoluteAngle(180));
        bigSequence->AddStep(new WaitSeconds(0.125));

        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new TimeDriveHold(0, -1, 1.7));
        bigSequence->AddStep(new WaitSeconds(0.125));
        

        bigSequence->AddStep(new TurnToAbsoluteAngle(290));
        bigSequence->AddStep(new WaitSeconds(0.125));
        bigSequence->AddStep(new TimeDriveHold(0.940, 0.342, 2.0));
        bigSequence->AddStep(new WaitSeconds(0.25));
        bigSequence->AddStep(new TurnToAbsoluteAngle(75));
        bigSequence->AddStep(new WaitSeconds(0.125));
        bigSequence->AddStep(new TimeDriveHold(-0.966, 0.259, 1.9));
        bigSequence->AddStep(new SetIntake(0));

        bigSequence->AddStep(new SetShooter(R_shooterSpeed * 0.9));
        bigSequence->AddStep(new TurnToAbsoluteAngle(15));
        bigSequence->AddStep(new WaitSeconds(1));
        // Shoot all the cargo
        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(0.3));

        bigSequence->AddStep(new SetIntake(0));
        bigSequence->AddStep(new SetIndexer(0));
        bigSequence->AddStep(new WaitSeconds(0.25));

        bigSequence->AddStep(new SetIntake(0.5));
        bigSequence->AddStep(new SetIndexer(0.5));
        bigSequence->AddStep(new WaitSeconds(3));

        // Spool down
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

void Robot::TeleopInit() {

    SwerveTrain::GetInstance().SetSwerveBrake(true);
    SwerveTrain::GetInstance().SetDriveBrake(true);
}

void Robot::TeleopPeriodic() {

    if (playerOne->GetRawButton(3)) {

        SwerveTrain::GetInstance().HardwareZero();
    }
    if (playerOne->GetRawButton(9) && playerOne->GetRawButton(10)) {

        SwerveTrain::GetInstance().HardwareZero();
    }
    if (playerOne->GetRawButton(4)) {

        NavX::GetInstance().Calibrate();
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

        SwerveTrain::GetInstance().Drive(
            -x,
            -y,
            z * R_controllerZMultiplier,
            playerOne->GetRawButton(1),
            playerOne->GetRawButton(2),
            -(((playerOne->GetThrottle() + 1.0) / 2.0) - 1.0)
        );
    }

    if(playerTwo->GetRightTriggerAxis() >= 0.5){

       Shooter::GetInstance().SetShooterSpeed(R_shooterSpeed);
    }
    else {

        Shooter::GetInstance().SetShooterSpeed(0);
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

    if(playerTwo->GetLeftY() > 0.25){
        Climber::GetInstance().SetClimberSpeed(0.75);
    }
    else if(playerTwo->GetLeftY() < -0.25){
        Climber::GetInstance().SetClimberSpeed(-0.75);
    }
    else{
        Climber::GetInstance().SetClimberSpeed(0);
    }
    
    
    

    /* Baseline code for a 4-stepped launcher instead a linear or parabolic launcher*/
    /* "Just something I wanted to try" - Aids*/

    
    frc::SmartDashboard::PutNumber("angle", NavX::GetInstance().getYawFull());
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
