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
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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


    public static final double g = 9.81; // m/s/s
    public static final double defaultPeriodSecs = 0.02; // s
    public static boolean isSim =  edu.wpi.first.wpilibj.RobotBase.isSimulation(); // 
    public static boolean PoseSoring = !false; // Only move Scoring Stuff if close to reef
    public static boolean ElevatorOff = false; // Shut off Elevator
    public static boolean Lineup = false; // Auto Lineup to Reef to Scrore.
    public static boolean ExtraInfo = true; // Turn on Extra network info


    public static class Vision {
        public static final String kCameraName = "Front";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center, up 15 degs.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(13.311564), 0.0, Units.inchesToMeters(7.332072)), new Rotation3d(0, Math.toRadians(-20), 0));

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
        public static final double ITranslation = 0.001;
        public static final double DTranslation = 0.1;

        public static final double PRotation = 3;
        public static final double IRotation = 0.001;
        public static final double DRotation = 0.03;
        
        // 0.0-1.0 of the max speed
        public static final double MaxSpeedPercentage = 0.95; // Default 1.0
        public static final double SlowSpeedPercentage = 0.10; // Default 0.15
        
        // Rotation per second max angular velocity
        public static final double MaxAngularRatePercentage = 0.82; // Default 0.75 
        public static final double SlowRotPercentage = 0.15; // Default 0.15

        // Deadbands for the drive and rotation
        public static final double DriveDeadband = isSim ? 0.15 : 0.02; // Drive Deadband
        public static final double RotationDeadband = isSim ? 0.15 : 0.02; // Rotation Deadband
        public static final double SnapDriveDeadband = 0.001; // Snap Rotation Deadband
        public static final double SnapRotationDeadband = 0.001; // Snap Rotation Deadband

    }


  public static final class Elevator {
    public static final CANDeviceID motorID = new CANDeviceID(20, kSuperStructureCanivoreName);
    public static final CANDeviceID followerID = new CANDeviceID(21, kSuperStructureCanivoreName);
    public static final double StatorLimit = 80.0;
    public static final double SupplyLimit = 40.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(2.273); // 16T #25
    public static final MechanismRatio motorRatio =
        isSim ? 
        new MechanismRatio(
            1, (28.0 / 10.0) * (1.0 / 4.0) * (42.0 / 18.0), Math.PI * sprocketPitchDiameter) : // Sim
        new MechanismRatio(
            1, (9.0 / 1.0), Math.PI * sprocketPitchDiameter); // Real
    public static final boolean motorInvert = true;
    public static final boolean followerInvert = true;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = isSim ? 
        new PIDConfig(5, 0.001, 0.1, 0.04, 0.02, 0.008, 0.13, GravityTypeValue.Elevator_Static) : 
        new PIDConfig(20, 0.01, 0.04, 0.04, 0.13, 0.008, 0.13, GravityTypeValue.Elevator_Static);
    public static final double maxVelocity = 1.8; // m/s // 1.2
    public static final double maxAcceleration = 27.0; // m/s^2
    public static final double maxJerk = 2.0; // m/s^3 (0 disables jerk limit)
    public static final double Expo_kV = 0.1;    
    public static final double Expo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    // TODO: use real numbers
    public static final double minHeight = 0.0; // m
    //public static final double powerCutoffHeight = Units.inchesToMeters(0.1); // m
    public static final double maxHeight = Units.inchesToMeters(60.0); // m
    public static final double stowHeight = Units.inchesToMeters(0.5); // m
    public static final double wristStowHeight = Units.inchesToMeters(4); // m
    public static final double armStowHeight = Units.inchesToMeters(8); // m
    public static final double stowTolerance = Units.inchesToMeters(0.1); // m
    public static final double intakeHeight = Units.inchesToMeters(11.75); // m // 

    public static final double L1 = Units.inchesToMeters(1); // m
    public static final double L2 = Units.inchesToMeters(13.75); // m
    public static final double L3 = Units.inchesToMeters(29.5); // m
    public static final double L4 = Units.inchesToMeters(56); // m 56.3 max, 55.8 was a lil too low

    public static final double L2Algae = Units.inchesToMeters(15); // m
    public static final double L3Algae = Units.inchesToMeters(31.5); // m

    public static final double PreStow = Units.inchesToMeters(10); // m
    public static final double SlowmodeHeight = Units.inchesToMeters(25); // m

    // For simulation.
    public static final double simCarriageMass = 7.0; // kg

  }

  

  public static final class Arm {
    public static final int beamBreakPort = 0;

    public static final CANDeviceID armMotorID = new CANDeviceID(25, kSuperStructureCanivoreName);
    public static final CANDeviceID armCoderID = new CANDeviceID(26, kSuperStructureCanivoreName);
    public static final MechanismRatio armMotorRatio =
        isSim ? 
        new MechanismRatio(
            1, (90.0 / 1.0) * (80.0 / 38.0)) : // Sim
        new MechanismRatio(
            1, (60.0 / 1.0) * (80.0 / 38.0)); // Real
    public static final MechanismRatio armSensorRatio =
        new MechanismRatio(1, (1.0));
    public static final boolean armMotorInvert = true;

    public static final CANDeviceID wristMotorID = new CANDeviceID(27, kSuperStructureCanivoreName);
    public static final MechanismRatio wristMotorRatio =
        isSim ? 
        new MechanismRatio(
            1, (4.0 / 1.0) * (32.0 / 14.0)) : // Sim
        new MechanismRatio(
            1, (27.0 / 1.0) * (32.0 / 14.0)); // Real
    public static final boolean wristMotorInvert = true;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(28, kSuperStructureCanivoreName);
    public static final MechanismRatio rollerMotorRatio = new MechanismRatio(12, 18);
    public static final boolean rollerMotorInvert = false;


    //public static final ArmFeedforward armFeedForward = new ArmFeedforward(3.0, 0.3, 0.6);
    public static final Constraints ArmConstraints =
        new Constraints(3.5, 10.0); // rad/s and rad/s^2  8, 20.0
    public static final double ArmMaxJerk = 1.0; // rad/s^3
    public static final int armPositionPIDSlot = 0;
    public static final PIDConfig armPositionPIDConfig = new PIDConfig(8, 0.0001, 0.03, 0, 0.25, 0.0008, 0.09, GravityTypeValue.Arm_Cosine);
    public static final double armExpo_kV = 0.25;    
    public static final double armExpo_kA = 0.01; // Use a slower kA of 0.1 V/(rps/s)
  //  public static final int armCoralPositionPIDSlot = 1;
  //  public static final PIDConfig armCoralPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);

    //public static final ArmFeedforward wristFeedForward = new ArmFeedforward(0.0, 0.3, 0.6);
    public static final Constraints WristConstraints =
        new Constraints(6.0, 20.0); // rad/s and rad/s^2   40.0, 80.0
        public static final double WristMaxJerk = 2.0; // rad/s^3
    public static final int wristPositionPIDSlot = 0;
    public static final PIDConfig wristPositionPIDConfig = new PIDConfig(3.0, 0.0001, 0.1, 0, 0.1, 0.0008, 0.02,GravityTypeValue.Arm_Cosine);
    public static final double wristExpo_kV = 0.1;   //                                                      ^ 1.22   
    public static final double wristExpo_kA = 0.005; // Use a slower kA of 0.1 V/(rps/s)
    // public static final int wristCoralPositionPIDSlot = 1;
    //public static final PIDConfig wristCoralPositionPIDConfig = new PIDConfig(2.0, 0, 0.1, 0, 0.12, 0.007, 0);

    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.028);
    public static final int rollerVelocityPIDSlot = 1;
    public static final PIDConfig rollerVelocityPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final int rollerPositionPIDSlot = 0;
    public static final PIDConfig rollerPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    // TODO: Use real values
    public static final double armBootAbsPositionOffset = Units.degreesToRadians(0);
    public static final double armMinAngle = Units.degreesToRadians(-1.0); // rads 
    public static final double armMaxAngle = Units.degreesToRadians(150.0); // rads 
    public static final double armStartingAngle = Units.degreesToRadians(90);
    public static final double armCgOffset = Units.degreesToRadians(0);

    public static final double wristBootAbsPositionOffset = Units.degreesToRadians(0);
    public static final double wristMinAngle = Units.degreesToRadians(-16.0); // rads 
    public static final double wristMaxAngle = Units.degreesToRadians(181.0); // rads 
    public static final double wristStartingAngle = Units.degreesToRadians(181) ; //+ armStartingAngle;
    public static final double wristCgOffset = Units.degreesToRadians(0);

    public static final double AngleTolerance = Units.degreesToRadians(1);

    public static final double intakeVelocity = -150.0; // rads/s
    public static final double outtakeVelocity = 1300.0; // rads/s

    public static final double rollerStallVelocity = 40; // rads/s
    public static final double rollerStallCurrent = 30; // Amps


    public static final double armIntakeAngle = Units.degreesToRadians(130);
    public static final double wristIntakeAngle = Units.degreesToRadians(90);
    public static final double armGroundIntakeAngle = Units.degreesToRadians(-5);
    public static final double wristGroundIntakeAngle = Units.degreesToRadians(25);
    public static final double armStowAngle = Units.degreesToRadians(89);
    public static final double armStowIntakeAngle = Units.degreesToRadians(110);
    public static final double wristStowAngle = Units.degreesToRadians(70);
    public static final double armScoreAngle = Units.degreesToRadians(89);
    public static final double wristScoreAngle = Units.degreesToRadians(-10);
    public static final double armWackAngle = Units.degreesToRadians(0);
    public static final double wristWackAngle = Units.degreesToRadians(80);
    public static final double armWackAfterAngle = Units.degreesToRadians(35);
    public static final double wristWackAfterAngle = Units.degreesToRadians(95);
    public static final double wristTestAngle = Units.degreesToRadians(160);
    public static final double wristL1Score = Units.degreesToRadians(80);
    public static final double armL1Score = Units.degreesToRadians(65);
    public static final double rollerL1Score = 400;
    public static final double wristL4Score = Units.degreesToRadians(-14);
    
    public static final Transform2d robotToArm =
        new Transform2d(Units.inchesToMeters(12.0), 0.0, new Rotation2d());
    public static final double ArmHeight = Units.inchesToMeters(12);

    // For simulation.
    public static final double simArmMOI = 0.379; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(8.5); // m
    public static final double simRollerMOI = 0.003; // kgMetersSquared

    public static final double wristArmMOI = 0.0374; // kgMetersSquared
    public static final double simwristCGLength = Units.inchesToMeters(3.5); // m
    
  }




  public static final class Pose {

    // public static final PathConstraints constraints = new PathConstraints(
    //         3, 2.5,
    //         Units.degreesToRadians(400), Units.degreesToRadians(600));

     public static final double XvelocityFactor = 0.1;
     public static final double YvelocityFactor = 0.1;

    public static final double SpeedReductionFactor = 0.15;

    public static final double PTranslationSlow = 3;
    public static final double ITranslationSlow = 10;
    public static final double DTranslationSlow = 0.03;

    public static final double PRotationSlow = 3;
    public static final double IRotationSlow = 10;
    public static final double DRotationSlow = 0.03;

    public static final double Tolerance = 0.01;

    public static final Pose2d Error = new Pose2d(6, 6, Rotation2d.fromDegrees(0));

    public static final Pose2d Ablue = new Pose2d(3.180, 4.175, Rotation2d.fromDegrees(0));
    public static final Pose2d Bblue = new Pose2d(3.180, 3.850, Rotation2d.fromDegrees(0));
    public static final Pose2d Cblue = new Pose2d(3.685, 2.975, Rotation2d.fromDegrees(60));
    public static final Pose2d Dblue = new Pose2d(3.975, 2.825, Rotation2d.fromDegrees(60));
    public static final Pose2d Eblue = new Pose2d(5.000, 2.825, Rotation2d.fromDegrees(120));
    public static final Pose2d Fblue = new Pose2d(5.285, 2.975, Rotation2d.fromDegrees(120));
    public static final Pose2d Gblue = new Pose2d(5.8, 3.850, Rotation2d.fromDegrees(180));
    public static final Pose2d Hblue = new Pose2d(5.8, 4.175, Rotation2d.fromDegrees(180));
    public static final Pose2d Iblue = new Pose2d(5.285, 5.075, Rotation2d.fromDegrees(240));
    public static final Pose2d Jblue = new Pose2d(5.000, 5.230, Rotation2d.fromDegrees(240));
    public static final Pose2d Kblue = new Pose2d(3.975, 5.230, Rotation2d.fromDegrees(300));
    public static final Pose2d Lblue = new Pose2d(3.685, 5.075, Rotation2d.fromDegrees(300));

    public static final double feildFlip = 17.5;
    public static final double feildFlipy = 8;

    public static final Pose2d Ared = new Pose2d(feildFlip - 3.180, feildFlipy - 4.175, Rotation2d.fromDegrees(180));
    public static final Pose2d Bred = new Pose2d(feildFlip - 3.180, feildFlipy - 3.850, Rotation2d.fromDegrees(180));
    public static final Pose2d Cred = new Pose2d(feildFlip - 3.685, feildFlipy - 2.975, Rotation2d.fromDegrees(-120));
    public static final Pose2d Dred = new Pose2d(feildFlip - 3.975, feildFlipy - 2.825, Rotation2d.fromDegrees(-120));
    public static final Pose2d Ered = new Pose2d(feildFlip - 5.000, feildFlipy - 2.825, Rotation2d.fromDegrees(-60));
    public static final Pose2d Fred = new Pose2d(feildFlip - 5.285, feildFlipy - 2.975, Rotation2d.fromDegrees(-60));
    public static final Pose2d Gred = new Pose2d(feildFlip - 5.8, feildFlipy - 3.850, Rotation2d.fromDegrees(0));
    public static final Pose2d Hred = new Pose2d(feildFlip - 5.8, feildFlipy - 4.175, Rotation2d.fromDegrees(0));
    public static final Pose2d Ired = new Pose2d(feildFlip - 5.285, feildFlipy - 5.075, Rotation2d.fromDegrees(-300));
    public static final Pose2d Jred = new Pose2d(feildFlip - 5.000, feildFlipy - 5.230, Rotation2d.fromDegrees(-300));
    public static final Pose2d Kred = new Pose2d(feildFlip - 3.975, feildFlipy - 5.230, Rotation2d.fromDegrees(-240));
    public static final Pose2d Lred = new Pose2d(feildFlip - 3.685, feildFlipy - 5.075, Rotation2d.fromDegrees(-240));



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

  public static final class Viz3d {
    public static double stage1Height = Units.inchesToMeters(26.0);
    public static final Pose3d intakePivotBase =
        new Pose3d(Units.inchesToMeters(-12.5), 0.0, Units.inchesToMeters(11.0), new Rotation3d());
    public static final Pose3d elevatorBase =
        new Pose3d(
            Units.inchesToMeters(3.5),
            0,
            Units.inchesToMeters(4.0),
            new Rotation3d(0, 0, 0));
    public static final Transform3d elevatorCarriageToLauncherArmPivot =
        new Transform3d(0, 0, Units.inchesToMeters(16.0), new Rotation3d());
  }







  //----------------------------------------------------------OLD DO NOT REMOVE----------------------------------------------------------
  public static final class Intake {
    public static final int beamBreakPort = 1;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(33, kSuperStructureCanivoreName);

    public static final MechanismRatio rollerMotorRatio =
        new MechanismRatio(1, (1.0 / 3.0));
    public static final boolean rollerMotorInvert = false;
    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.3, 0.12, 0);
    public static final PIDConfig rollerPIDConfig = new PIDConfig(0.1, 0, 0);
    public static final int rollerVelocitySlot = 0;

    public static final CANDeviceID deployMotorID = new CANDeviceID(31, kSuperStructureCanivoreName);
    public static final CANDeviceID deployFollowerID = new CANDeviceID(32, kSuperStructureCanivoreName);

    public static final boolean followerInvert = true;
    public static final MechanismRatio deployMotorRatio =
        isSim ? 
        new MechanismRatio(
            1, (42.0 / 10.0) * (22.0 / 22.0) * (42.0 / 16.0) * (36.0 / 16.0)) : // Sim
        new MechanismRatio(
            1, (27.0 / 1.0) * (36.0 / 16.0)); // Real
    public static final boolean deployMotorInvert = false;
    public static final PIDConfig deployPIDConfig = new PIDConfig(2.0, 0, 0.3, 0, 1.5, 0.000, 0.08, GravityTypeValue.Arm_Cosine);
    public static final int deployPositionSlot = 0;
    public static final double deployMaxVelocity = 0.2; // rad/s
    public static final double deployMaxAcceleration = 140.0; // rad/s^2
    public static final double deployMaxJerk = 800.0; // rad/s^3

    public static final double bootAbsPositionOffset = Units.degreesToRadians(1.8);
    public static final double minAngle = Units.degreesToRadians(-20.0); // rads
    public static final double maxAngle = Units.degreesToRadians(110.0); // rads
    public static final double startingAngle = maxAngle + bootAbsPositionOffset;
    public static final double intakeDeployAngle = Math.toRadians(40); // rad
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

  public static final class Climber {
    public static final CANDeviceID motorID = new CANDeviceID(40, kSuperStructureCanivoreName);
    public static final CANDeviceID followerID = new CANDeviceID(41, kSuperStructureCanivoreName);
    public static final double StatorLimit = 80.0;
    public static final double SupplyLimit = 40.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(2); 
    public static final MechanismRatio motorRatio =
        new MechanismRatio(
            1, (200.0 / 1.0), Math.PI * sprocketPitchDiameter); // Sim
        // new MechanismRatio(
            // 1, (135.0 / 1.0), Math.PI * sprocketPitchDiameter); // Real
    public static final boolean motorInvert = false;
    public static final boolean followerInvert = true;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = new PIDConfig(3, 0, 0.1, 0, 0.12, 0, 0.4,GravityTypeValue.Elevator_Static);
    public static final double maxVelocity = 0.04; // m/s
    public static final double maxAcceleration = 30.0; // m/s^2
    public static final double maxJerk = 0.0; // m/s^3 (0 disables jerk limit)

    // TODO: use real numbers
    public static final double minHeight = Units.inchesToMeters(-25.0); // m
    //public static final double powerCutoffHeight = Units.inchesToMeters(0.1); // m
    public static final double maxHeight = Units.inchesToMeters(25.0); // m
    public static final double stowHeight = Units.inchesToMeters(0); // m
    public static final double upperStowHeight = Units.inchesToMeters(-8.1); // m //-1.3141   -8.2623
    public static final double climbHeight = Units.inchesToMeters(1.1); // m
    public static final double stowTolerance = Units.inchesToMeters(0.01); // m

    // For simulation.
    public static final double simCarriageMass = 25.0; // kg

    // TODO: find real values
    public static final ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(0.35, 0.15, 15.8);
  }

}