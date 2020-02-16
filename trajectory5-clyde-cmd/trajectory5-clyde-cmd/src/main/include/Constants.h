



/*

Okay look I don't really know what I'm doing here but I'm following the Constants.h file
from the WPI github that does the right thing supposedly. Most ports here are educated guesses at best
and should be fixed as I find errors. Some things include descriptions of where I got them from, i.e. 
what step from the Trajectory Tutorial they are from, and I am just glad I have no red squigglies. That
is all, I am trying my best so thanks

possible error in future may come from fact that I renamed ExampleSubsystem.h file to DriveSubsystem

*/



#pragma once
#include <units/units.h>
/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

//following GitHub for Ramsete controller example because I don't know what the heck I'm doing

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <wpi/math>

namespace DriveConstants {
constexpr int kLeftMotor1Port = 2;
constexpr int kLeftMotor2Port = 6;
constexpr int kRightMotor1Port = 3; //switched with 9
constexpr int kRightMotor2Port = 9;

constexpr int kLeftEncoderPorts[]{0, 1};
constexpr int kRightEncoderPorts[]{2, 3};
constexpr bool kLeftEncoderReversed = false;
constexpr bool kRightEncoderReversed = false; //switched from true

//data from robot measurements

constexpr auto kTrackwidth = 0.5334_m; //21 in track width
extern const frc::DifferentialDriveKinematics kDriveKinematics; //instance of class DifferentialDriveKinematics that lets us use trackwidth (dist. b/w wheels) to convert from chassis speeds to wheel speeds

constexpr int kEncoderCPR = 1024; //CPR is 1/4 of EPR I think
constexpr double kWheelDiameterInches = 6; //6 in wheels
constexpr double kEncoderDistancePerPulse = 
    //assumes encoders = directly mounted on wheel shafts
    (kWheelDiameterInches * wpi::math::pi) / static_cast<double>(kEncoderCPR);

constexpr bool kGyroReversed = false; //is gyro reversed ? who knows !


/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

//the following until the star line are all from Trajectory Tutorial Step 2: Entering the Calculated Constants

//following constants are from frc-characterization tool
  constexpr auto ks = 0.146_V; //volts needed to overcome static friction from bo
  constexpr auto kv = 1.35 * 1_V * 1_s / 1_m; //velocity
  constexpr auto ka = 0.252 * 1_V * 1_s * 1_s / 1_m; //acceleration

//WHAT IS kPDriveVel??!!!!! NEED D:

  constexpr double kPDriveVel = 8.5; //no idea where to get this from but I feel like this is somewhat important
} //namespace DriveConstants

namespace AutoConstants{
  constexpr auto kMaxSpeed = 3.913632_mps; //from Hector, 12.84 ft/s adjusted speed
  constexpr auto kMaxAcceleration = 3_mps_sq; //I actually don't know this, got 3 from the trajectory tutorial but they said it's "not extremely crucial"

//following are constants needed for RAMSETE controller, see "Constructing the Ramsete Controller Object" for adjusting/tuning if necessary (step 2)

  constexpr double kRamseteB = 2; 
  constexpr double kRamseteZeta = 0.7;
} //namespace AutoConstants
/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

namespace OIConstants {

constexpr int kDriverControllerPort = 1;
} //namespace OIConstants
