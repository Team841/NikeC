// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/DriverStation.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "units/velocity.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("autos");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            if (auto ally = frc::DriverStation::GetAlliance()){
                if (ally == frc::DriverStation::Alliance::kRed){
                    return drive.WithVelocityX(joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .WithVelocityY(joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                } else {
                    return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
                }
            } else {
                return drive.WithVelocityX(units::meters_per_second_t{0})
                .WithVelocityY(units::meters_per_second_t{0})
            .WithRotationalRate(units::radians_per_second_t{0});
            }
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));


    joystick.X().WhileTrue(
        drivetrain.latestCommand.get()
).WhileFalse(
        this->drivetrain.ApplyRequest(
            [this]() -> auto&& {
                return drive.WithVelocityX(0_mps).WithVelocityY(0_mps).WithRotationalRate(0_tps);
            }
        )
    );

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // // Simple drive forward auton
    // return frc2::cmd::Sequence(
    //     // Reset our field centric heading to match the robot
    //     // facing away from our alliance station wall (0 deg).
    //     drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
    //     // Then slowly drive forward (away from us) for 5 seconds.
    //     drivetrain.ApplyRequest([this]() -> auto&& {
    //         return drive.WithVelocityX(0.5_mps)
    //             .WithVelocityY(0_mps)
    //             .WithRotationalRate(0_tps);
    //     })
    //     .WithTimeout(5_s),
    //     // Finally idle for the rest of auton
    //     drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
    // );
    return autoChooser.GetSelected();

}
