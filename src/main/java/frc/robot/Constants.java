/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.quixlib.devices.CANDeviceID;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Constants {
    // CANID's:
    //
    // Drivetrain 1-19
    // Elevator / Arm 20-29 
    // Intake 30-39
    // Climber 40-49



    // "rio" for rio bus
    public static final String kDriveTrainCanivoreName = "driveTrain"; // need to update after new tuner
    public static final String kSuperStructureCanivoreName = "superStructure";

    public static final boolean PublishState = true; // Turns on publishing through smartdashboard network tables for all device data

    public static final double g = 9.81; // m/s/s
    public static final double defaultPeriodSecs = 0.02; // s



    public static class Vision {
        public static final String kCameraName = "FrontCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center, up 15 degs.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.3, 0.0, 0.15), new Rotation3d(0, Math.toRadians(-15), 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class Drive { //Drive Constants that are not in TunerConstants / Gnenerated

        // PID for Rotation and Translation for Auto and Teleop Snap
        public static final double PTranslation = 5;
        public static final double ITranslation = 0;
        public static final double DTranslation = 0.1;

        public static final double PRotation = 3;
        public static final double IRotation = 0;
        public static final double DRotation = 0.03;
        
        // 0.0-1.0 of the max speed
        public static final double MaxSpeedPercentage = 1.0; // Default 1.0
        // Rotation per second max angular velocity
        public static final double MaxAngularRatePercentage = 1; // Default 0.75 

        // Deadbands for the drive and rotation
        public static final double DriveDeadband = 0.15; // Drive Deadband
        public static final double RotationDeadband = 0.15; // Rotation Deadband
        public static final double SnapRotationDeadband = 0.001; // Snap Rotation Deadband

    }

    public static final class Example {
    public static final CANDeviceID motorID = new CANDeviceID(99, kSuperStructureCanivoreName);
    public static final MechanismRatio motorRatio = new MechanismRatio(1, 1);
    public static final boolean motorInvert = false;
  }

  public static final class Elevator {
    public static final CANDeviceID motorID = new CANDeviceID(20, kSuperStructureCanivoreName);
    public static final CANDeviceID followerID = new CANDeviceID(21, kSuperStructureCanivoreName);
    public static final double StatorLimit = 80.0;
    public static final double SupplyLimit = 40.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.273); // 16T #25
    public static final MechanismRatio motorRatio =
        new MechanismRatio(
            1, (28.0 / 10.0) * (1.0 / 4.0) * (42.0 / 18.0), Math.PI * sprocketPitchDiameter);
    public static final boolean motorInvert = false;
    public static final boolean followerInvert = true;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = new PIDConfig(3, 0, 0.1, 0, 0.12, 0, 0.4);
    public static final double maxVelocity = 1.0; // m/s
    public static final double maxAcceleration = 30.0; // m/s^2
    public static final double maxJerk = 0.0; // m/s^3 (0 disables jerk limit)

    // TODO: use real numbers
    public static final double minHeight = 0.0; // m
    //public static final double powerCutoffHeight = Units.inchesToMeters(0.1); // m
    public static final double maxHeight = Units.inchesToMeters(75.0); // m
    public static final double stowHeight = Units.inchesToMeters(1); // m
    public static final double wristStowHeight = Units.inchesToMeters(6); // m
    public static final double armStowHeight = Units.inchesToMeters(16); // m
    public static final double stowTolerance = Units.inchesToMeters(0.25); // m
    public static final double intakeHeight = Units.inchesToMeters(10); // m

    public static final double L1 = Units.inchesToMeters(12); // m
    public static final double L2 = Units.inchesToMeters(34); // m
    public static final double L3 = Units.inchesToMeters(44); // m
    public static final double L4 = Units.inchesToMeters(68); // m

    public static final double L2Algae = Units.inchesToMeters(33); // m
    public static final double L3Algae = Units.inchesToMeters(46); // m

    // For simulation.
    public static final double simCarriageMass = 7.0; // kg

    // TODO: find real values
    public static final ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(0.0, 0.0, 0.0); // new ElevatorFeedforward(0.35, 0.15, 15.8);
  }

  public static final class Intake {
    public static final int beamBreakPort = 1;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(33, kSuperStructureCanivoreName);

    public static final MechanismRatio rollerMotorRatio =
        new MechanismRatio(1, (36.0 / 12.0) * (36.0 / 28.0));
    public static final boolean rollerMotorInvert = false;
    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.3, 0.12, 0);
    public static final PIDConfig rollerPIDConfig = new PIDConfig(0.1, 0, 0);
    public static final int rollerVelocitySlot = 0;

    public static final CANDeviceID deployMotorID = new CANDeviceID(31, kSuperStructureCanivoreName);
    public static final CANDeviceID deployFollowerID = new CANDeviceID(32, kSuperStructureCanivoreName);

    public static final boolean followerInvert = true;
    public static final MechanismRatio deployMotorRatio =
        new MechanismRatio(1, (42.0 / 10.0) * (22.0 / 22.0) * (42.0 / 16.0) * (36.0 / 16.0));
    public static final boolean deployMotorInvert = true;
    public static final PIDConfig deployPIDConfig = new PIDConfig(2.0, 0, 0.3, 0, 0.12, 0.007, 0);
    public static final int deployPositionSlot = 0;
    public static final double deployMaxVelocity = 12.0; // rad/s
    public static final double deployMaxAcceleration = 140.0; // rad/s^2
    public static final double deployMaxJerk = 800.0; // rad/s^3

    public static final double bootAbsPositionOffset = Units.degreesToRadians(1.8);
    public static final double minAngle = Units.degreesToRadians(-50.0); // rads
    public static final double maxAngle = Units.degreesToRadians(110.0); // rads
    public static final double startingAngle = maxAngle + bootAbsPositionOffset;
    public static final double intakeDeployAngle = Math.toRadians(-50); // rad
    public static final double intakeScoreAngle = Math.toRadians(85); // rad
    public static final double intakeStowAngle = Math.toRadians(105); // rad
    public static final double intakeClimbAngle = Math.toRadians(100); // rad
    public static final double intakeRollerVelocity = 100; // rad/s
    public static final double outtakeRollerVelocity = -100; // rad/s
    public static final double holdRollerVelocity = 10; // rad/s

    // For simulation.
    public static final double simArmMOI = 0.2; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(7.0); // m
    public static final double simRollerMOI = 0.01; // kgMetersSquared
  }

  public static final class Arm {
    public static final int beamBreakPort = 0;

    public static final CANDeviceID armMotorID = new CANDeviceID(25, kSuperStructureCanivoreName);
    public static final CANDeviceID armCoderID = new CANDeviceID(26, kSuperStructureCanivoreName);
    public static final MechanismRatio armMotorRatio =
        new MechanismRatio(1, (90.0 / 1.0) * (80.0 / 38.0));
    public static final MechanismRatio armSensorRatio =
        new MechanismRatio(1, (1.0));
    public static final boolean armMotorInvert = true;

    public static final CANDeviceID wristMotorID = new CANDeviceID(27, kSuperStructureCanivoreName);
    public static final MechanismRatio wristMotorRatio =
        new MechanismRatio(1, (5.0 / 1.0) * (32.0 / 14.0));
    public static final boolean wristMotorInvert = true;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(28, kSuperStructureCanivoreName);
    public static final MechanismRatio rollerMotorRatio = new MechanismRatio(12, 18);
    public static final boolean rollerMotorInvert = true;


    //public static final ArmFeedforward armFeedForward = new ArmFeedforward(3.0, 0.3, 0.6);
    public static final Constraints ArmConstraints =
        new Constraints(60.0, 80.0); // rad/s and rad/s^2
    public static final int armPositionPIDSlot = 0;
    public static final PIDConfig armPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);
    public static final int armCoralPositionPIDSlot = 1;
    public static final PIDConfig armCoralPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);

    //public static final ArmFeedforward wristFeedForward = new ArmFeedforward(0.0, 0.3, 0.6);
    public static final Constraints WristConstraints =
        new Constraints(60.0, 80.0); // rad/s and rad/s^2
    public static final int wristPositionPIDSlot = 0;
    public static final PIDConfig wristPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);
    public static final int wristCoralPositionPIDSlot = 1;
    public static final PIDConfig wristCoralPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);

    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.028);
    public static final int rollerVelocityPIDSlot = 1;
    public static final PIDConfig rollerVelocityPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final int rollerPositionPIDSlot = 0;
    public static final PIDConfig rollerPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    // TODO: Use real values
    public static final double armBootAbsPositionOffset = Units.degreesToRadians(0);
    public static final double armMinAngle = Units.degreesToRadians(-10.0); // rads (trap position)
    public static final double armMaxAngle = Units.degreesToRadians(160.0); // rads (stow position)
    public static final double armStartingAngle = Units.degreesToRadians(90);
    public static final double armCgOffset = Units.degreesToRadians(0);

    public static final double wristBootAbsPositionOffset = Units.degreesToRadians(0);
    public static final double wristMinAngle = Units.degreesToRadians(-16.0); // rads (trap position)
    public static final double wristMaxAngle = Units.degreesToRadians(180.0); // rads (stow position)
    public static final double wristStartingAngle = Units.degreesToRadians(180) + armStartingAngle;
    public static final double wristCgOffset = Units.degreesToRadians(0);

    public static final double AngleTolerance = Units.degreesToRadians(2);

    public static final double intakeVelocity = 350.0; // rads/s
    public static final double outtakeVelocity = 310.0; // rads/s


    public static final double armIntakeAngle = Units.degreesToRadians(145);
    public static final double wristIntakeAngle = Units.degreesToRadians(90);
    public static final double armGroundIntakeAngle = Units.degreesToRadians(-5);
    public static final double wristGroundIntakeAngle = Units.degreesToRadians(25);
    public static final double armStowAngle = Units.degreesToRadians(89);
    public static final double armStowIntakeAngle = Units.degreesToRadians(95);
    public static final double wristStowAngle = Units.degreesToRadians(90);
    public static final double armScoreAngle = Units.degreesToRadians(89);
    public static final double wristScoreAngle = Units.degreesToRadians(-15);
    public static final double armWackAngle = Units.degreesToRadians(0);
    public static final double wristWackAngle = Units.degreesToRadians(80);

    public static final Transform2d robotToArm =
        new Transform2d(Units.inchesToMeters(12.0), 0.0, new Rotation2d());
    public static final double ArmHeight = Units.inchesToMeters(12);

    public static final double rollerBeamBreakOffset = 1.5 * Math.PI; // rads

    // For simulation.
    public static final double simArmMOI = 0.406; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(9.0); // m
    public static final double simRollerMOI = 0.003; // kgMetersSquared

    public static final double wristArmMOI = 0.045; // kgMetersSquared
    public static final double simwristCGLength = Units.inchesToMeters(5.0); // m
    
  }


  public static final class Climber {
    public static final CANDeviceID motorID = new CANDeviceID(40, kSuperStructureCanivoreName);
    public static final CANDeviceID followerID = new CANDeviceID(41, kSuperStructureCanivoreName);
    public static final double StatorLimit = 80.0;
    public static final double SupplyLimit = 40.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(0.7); 
    public static final MechanismRatio motorRatio =
        new MechanismRatio(
            1, (68.0 / 10.0) * (2.0 / 4.0) * (42.0 / 18.0), Math.PI * sprocketPitchDiameter);
    public static final boolean motorInvert = false;
    public static final boolean followerInvert = true;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = new PIDConfig(3, 0, 0.1, 0, 0.12, 0, 0.4);
    public static final double maxVelocity = 1.0; // m/s
    public static final double maxAcceleration = 30.0; // m/s^2
    public static final double maxJerk = 0.0; // m/s^3 (0 disables jerk limit)

    // TODO: use real numbers
    public static final double minHeight = 0.0; // m
    //public static final double powerCutoffHeight = Units.inchesToMeters(0.1); // m
    public static final double maxHeight = Units.inchesToMeters(75.0); // m
    public static final double stowHeight = Units.inchesToMeters(0); // m
    public static final double upperStowHeight = Units.inchesToMeters(8); // m
    public static final double climbHeight = Units.inchesToMeters(5); // m
    public static final double stowTolerance = Units.inchesToMeters(0.25); // m

    // For simulation.
    public static final double simCarriageMass = 25.0; // kg

    // TODO: find real values
    public static final ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(0.35, 0.15, 15.8);
  }


  public static final class Pose {

    // public static final PathConstraints constraints = new PathConstraints(
    //         3, 2.5,
    //         Units.degreesToRadians(400), Units.degreesToRadians(600));

    // public static final double XvelocityFactor = 0.15;
    // public static final double YvelocityFactor = 0.15;

    public static final double PTranslationSlow = 3;
    public static final double ITranslationSlow = 0;
    public static final double DTranslationSlow = 0.1;

    public static final double PIDmultValue = 2;

    public static final Pose2d Error = new Pose2d(6, 6, Rotation2d.fromDegrees(0));

    public static final Pose2d Ablue = new Pose2d(3.2, 4.3, Rotation2d.fromDegrees(0));
    public static final Pose2d Bblue = new Pose2d(3.2, 3.8, Rotation2d.fromDegrees(0));
    public static final Pose2d Cblue = new Pose2d(3.7, 3.0, Rotation2d.fromDegrees(60));
    public static final Pose2d Dblue = new Pose2d(4.1, 2.8, Rotation2d.fromDegrees(60));
    public static final Pose2d Eblue = new Pose2d(4.9, 2.8, Rotation2d.fromDegrees(120));
    public static final Pose2d Fblue = new Pose2d(5.4, 3.1, Rotation2d.fromDegrees(120));
    public static final Pose2d Gblue = new Pose2d(5.8, 3.7, Rotation2d.fromDegrees(180));
    public static final Pose2d Hblue = new Pose2d(5.8, 4.3, Rotation2d.fromDegrees(180));
    public static final Pose2d Iblue = new Pose2d(5.4, 5.0, Rotation2d.fromDegrees(240));
    public static final Pose2d Jblue = new Pose2d(5.0, 5.3, Rotation2d.fromDegrees(240));
    public static final Pose2d Kblue = new Pose2d(4.1, 5.3, Rotation2d.fromDegrees(300));
    public static final Pose2d Lblue = new Pose2d(3.7, 5.0, Rotation2d.fromDegrees(300));

    public static final double feildFlip = 117.5;
    public static final Pose2d Ared = new Pose2d(feildFlip - 3.2, 4.3, Rotation2d.fromDegrees(180));
    public static final Pose2d Bred = new Pose2d(feildFlip - 3.2, 3.8, Rotation2d.fromDegrees(180));
    public static final Pose2d Cred = new Pose2d(feildFlip - 3.7, 3.0, Rotation2d.fromDegrees(120));
    public static final Pose2d Dred = new Pose2d(feildFlip - 4.1, 2.8, Rotation2d.fromDegrees(120));
    public static final Pose2d Ered = new Pose2d(feildFlip - 4.9, 2.8, Rotation2d.fromDegrees(60));
    public static final Pose2d Fred = new Pose2d(feildFlip - 5.4, 3.1, Rotation2d.fromDegrees(60));
    public static final Pose2d Gred = new Pose2d(feildFlip - 5.8, 3.7, Rotation2d.fromDegrees(0));
    public static final Pose2d Hred = new Pose2d(feildFlip - 5.8, 4.3, Rotation2d.fromDegrees(0));
    public static final Pose2d Ired = new Pose2d(feildFlip - 5.4, 5.0, Rotation2d.fromDegrees(300));
    public static final Pose2d Jred = new Pose2d(feildFlip - 5.0, 5.3, Rotation2d.fromDegrees(300));
    public static final Pose2d Kred = new Pose2d(feildFlip - 4.1, 5.3, Rotation2d.fromDegrees(240));
    public static final Pose2d Lred = new Pose2d(feildFlip - 3.7, 5.0, Rotation2d.fromDegrees(240));



  }


  


  public static final class Viz {
    public static final double xOffset = Units.inchesToMeters(30.0);

    public static final double intakePivotX = xOffset + Units.inchesToMeters(27.25);
    public static final double intakePivotY = Units.inchesToMeters(11.25);
    public static final double intakeArmLength = Units.inchesToMeters(14.0);

    public static final double elevatorBaseX = xOffset + Units.inchesToMeters(18.0);
    public static final double elevatorBaseY = Units.inchesToMeters(3.0);
    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(90.0);
    public static final double elevatorBaseLength = Units.inchesToMeters(35.0);
    public static final double elevatorCarriageLength = Units.inchesToMeters(6.0);

    public static final double ArmArmPivotX = Units.inchesToMeters(4.0);
    public static final double ArmArmLength = Units.inchesToMeters(12.0);
    public static final double ArmWristLength = Units.inchesToMeters(6.0);
    public static final double ArmRollerX = Units.inchesToMeters(8.0);
    public static final double ArmRollerY = Units.inchesToMeters(0);

    public static final double climberBaseX = xOffset + Units.inchesToMeters(10.0);
    public static final double climberBaseY = Units.inchesToMeters(3.0);
    public static final Rotation2d climberAngle = Rotation2d.fromDegrees(130.0);
    public static final double climberBaseLength = Units.inchesToMeters(15.0);
    public static final double climberCarriageLength = Units.inchesToMeters(6.0);
    

    public static final double angularVelocityScalar = 0.01;
  }




}