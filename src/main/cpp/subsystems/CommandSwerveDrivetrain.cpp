#include "subsystems/CommandSwerveDrivetrain.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation2d.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <frc/RobotController.h>
#include <memory>
#include <vector>
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/Requirements.h"
#include "limelights/LimelightHelpers.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "pathplanner/lib/path/GoalEndState.h"
#include "pathplanner/lib/path/IdealStartingState.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/path/Waypoint.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/time.h"
#include <iostream>
#include "pathplanner/lib/commands/FollowPathCommand.h"



using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
    //  */
    // if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
    //     auto const allianceColor = frc::DriverStation::GetAlliance();
    //     if (allianceColor) {
    //         SetOperatorPerspectiveForward(
    //             *allianceColor == frc::DriverStation::Alliance::kRed
    //                 ? kRedAlliancePerspectiveRotation
    //                 : kBlueAlliancePerspectiveRotation
    //         );
    //         m_hasAppliedOperatorPerspective = true;
    //     }
    // }

    this->visionPeriodic();
    // this->latestCommand = this->buildPickupAuto();
    this->processIphone();
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void CommandSwerveDrivetrain::visionPeriodic(){
    this->setLLSettings();

    double base1 = 0.3;
    double base2 = 0.3;
    double base3 = 0.3;

    bool seesTarget = this->appleTable->GetEntry("tv").GetDouble(0) == 1.0;
    if (seesTarget){
        auto megaTag = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight-apple");
        std::array<double, 3> std{base1 * megaTag.avgTagArea,
                                base2 * megaTag.avgTagArea,
                                base3 * megaTag.avgTagArea
                            };
        
        this->AddVisionMeasurement(megaTag.pose, megaTag.timestampSeconds, std);
    }
}

void CommandSwerveDrivetrain::setLLSettings(){
    frc::Rotation2d gyroAngle = this->GetState().Pose.Rotation();
    units::radians_per_second_t gyroAngularVel = this->GetState().Speeds.omega; 
    try {
        LimelightHelpers::SetIMUMode("limelight-apple", 1);

        frc::Translation2d limelightToCenterOfTurret{
            units::meter_t{0.0381},
            units::meter_t{0}
        };
        
        LimelightHelpers::setCameraPose_RobotSpace("limelight-apple", limelightToCenterOfTurret.Y().value(), limelightToCenterOfTurret.X().value(),  0.7747, 0, 0, 180);

    } catch (...){
        return;
    }
}

void CommandSwerveDrivetrain::ConfigurePathPlanner(){
    using namespace pathplanner;

    RobotConfig config = RobotConfig::fromGUISettings();

    AutoBuilder::configure(
         [this]{ return GetState().Pose; }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]{ return GetState().Speeds; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ 
            return SetControl(
                rSpeeds.WithSpeeds(frc::ChassisSpeeds::Discretize(speeds, 20_ms))
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
            );
        }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(7.0, 0.0, 0.0) // Rotation PID constants
        ),
        std::move(config), // The robot configuration
        [] {
            auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
            return alliance == frc::DriverStation::Alliance::kRed;
        },
        this // Reference to this subsystem to set requirements
    );
}

void CommandSwerveDrivetrain::processIphone(){
    std::vector<frc::Pose3d> listOfPoses = this->targPoses.Get();

    std::vector<frc::Pose2d> poses2d;
    poses2d.reserve(listOfPoses.size());
    std::vector<frc::Pose3d> poseAdjusted;
    poseAdjusted.reserve(listOfPoses.size());
    
    frc::Pose2d currPose = this->GetState().Pose;

    for (auto &p : listOfPoses){
        poses2d.emplace_back(frc::Pose2d{
            p.ToPose2d().Translation(),
            frc::Rotation2d{
                0_deg
            }
        });

        frc::Pose3d toPose3d = frc::Pose3d{
            currPose.X(),
            currPose.Y(),
            p.Z(),
            frc::Rotation3d{}
        };
        
        auto pRot = p.RotateAround(
            frc::Translation3d{},
            frc::Rotation3d{
                0_deg,
                0_deg,
                currPose.Rotation().Degrees()
            }
        );

        poseAdjusted.push_back(
            frc::Pose3d{
                toPose3d.Translation() - pRot.Translation(),
                frc::Rotation3d{}      
            }
        );
    }

    this->adjustedPosesPublisher.Set(poseAdjusted);
    this->adjustedPoseHold = poseAdjusted;
}

frc2::CommandPtr CommandSwerveDrivetrain::buildPickupAuto(){

    using namespace pathplanner;
    
    std::cout << ((adjustedPoseHold.size() > 2 && !(adjustedPoseHold[0] == frc::Pose3d{})) ? "Eval: True" : "Eval: False") << std::endl;

    if (adjustedPoseHold.size() > 2 && !(adjustedPoseHold[0] == frc::Pose3d{})){
        // std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses();

        std::vector<frc::Pose2d> poses2d;
        poses2d.reserve(adjustedPoseHold.size());

        // poses2d.push_back(this->GetState().Pose);
        
        for (auto &p : adjustedPoseHold){
            if (p.Translation() != frc::Translation3d{}) poses2d.push_back(p.ToPose2d());
            std::cout<< "x val: " << p.Translation().ToTranslation2d().X().value() << "; y val: " << p.Translation().ToTranslation2d().Y().value() << std::endl;
        }

        PathPlannerPath::clearPathCache();
        std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses2d);

        PathConstraints constraints(1.0_mps, 0.5_mps_sq, 360_deg_per_s, 720_deg_per_s_sq); // The constraints for this path.
        
        auto path = std::make_shared<PathPlannerPath>(
            waypoints,
            constraints,
            std::nullopt,
            GoalEndState(0.0_mps, frc::Rotation2d{0_deg})
        );

        path->preventFlipping = true;

        std::cout << "At the point" << std::endl;

        // RobotConfig config = RobotConfig::fromGUISettings();

        // auto controller = std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
        //     PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
        //     PIDConstants(7.0, 0.0, 0.0));
        
        // controller->reset(this->GetState().Pose, this->GetState().Speeds);

        // auto followcommand =  FollowPathCommand(
        //     path,
        //     [this]{return this->GetState().Pose;},
        //     [this]{return this->GetState().Speeds;},
        //     [this](auto speeds, auto feedforwards){ 
        //         return SetControl(
        //             rSpeeds.WithSpeeds(frc::ChassisSpeeds::Discretize(speeds, 20_ms))
        //                 // .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
        //                 // .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
        //         );
        //     },
        //     std::move(controller), // Rotation PID constants 
        //     std::move(config),
        //     []{return false;},
        //     frc2::Requirements({this})
        // ).ToPtr();

        // return followcommand;
        return AutoBuilder::followPath(path);
    }

    return frc2::InstantCommand().ToPtr();
}