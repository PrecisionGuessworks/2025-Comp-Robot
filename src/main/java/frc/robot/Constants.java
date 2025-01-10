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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    public static final String kCanivoreName = "canivore";
    public static final double g = 9.81; // m/s/s
    public static final double defaultPeriodSecs = 0.02; // s



    public static class Vision {
        public static final String kCameraName = "YOUR CAMERA NAME";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center, up 15 degs.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.2), new Rotation3d(0, Math.toRadians(-15), 0));

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
    public static final CANDeviceID motorID = new CANDeviceID(99, kCanivoreName);
    public static final MechanismRatio motorRatio = new MechanismRatio(1, 1);
    public static final boolean motorInvert = false;
  }

  public static final class Elevator {
    public static final CANDeviceID motorID = new CANDeviceID(10, kCanivoreName);
    public static final double StatorLimit = 80.0;
    public static final double SupplyLimit = 40.0;
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.273); // 16T #25
    public static final MechanismRatio motorRatio =
        new MechanismRatio(
            1, (28.0 / 10.0) * (1.0 / 4.0) * (42.0 / 18.0), Math.PI * sprocketPitchDiameter);
    public static final boolean motorInvert = false;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = new PIDConfig(3, 0, 0.1, 0, 0.12, 0, 0.4);
    public static final double maxVelocity = 1.0; // m/s
    public static final double maxAcceleration = 30.0; // m/s^2
    public static final double maxJerk = 0.0; // m/s^3 (0 disables jerk limit)

    // TODO: use real numbers
    public static final double minHeight = 0.0; // m
    public static final double powerCutoffHeight = Units.inchesToMeters(0.5); // m
    public static final double maxHeight = Units.inchesToMeters(36.0); // m
    public static final double stowHeight = Units.inchesToMeters(1); // m
    public static final double stowTolerance = Units.inchesToMeters(0.25); // m
    public static final double scoreAmpHeight = Units.inchesToMeters(34.0); // m
    public static final double scoreAmpTolerance = Units.inchesToMeters(0.25); // m
    public static final double climbRetractHeight = Units.inchesToMeters(0.0); // m
    public static final double climbExtendHeight = Units.inchesToMeters(0.25); // m

    // For simulation.
    public static final double simCarriageMass = 8.0; // kg

    // TODO: find real values
    public static final Constraints elevatorTrapConstraints =
        new Constraints(1, 3); // m/s and m/s^2
    public static final ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(0.0, 0.0, 0.0); // new ElevatorFeedforward(0.35, 0.15, 15.8);
  }

  public static final class Intake {
    public static final int beamBreakPort = 1;

    public static final CANDeviceID rollerMotorID = new CANDeviceID(9, kCanivoreName);
    public static final MechanismRatio rollerMotorRatio =
        new MechanismRatio(1, (36.0 / 12.0) * (36.0 / 28.0));
    public static final boolean rollerMotorInvert = false;
    public static final SimpleMotorFeedforward rollerFeedforward =
        new SimpleMotorFeedforward(0.3, 0.12, 0);
    public static final PIDConfig rollerPIDConfig = new PIDConfig(0.1, 0, 0);
    public static final int rollerVelocitySlot = 0;

    public static final CANDeviceID deployMotorID = new CANDeviceID(8, kCanivoreName);
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
    public static final double intakeStowAngle = Math.toRadians(105); // rad
    public static final double intakeRollerVelocity = 100; // rad/s

    // For simulation.
    public static final double simArmMOI = 0.2; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(7.0); // m
    public static final double simRollerMOI = 0.01; // kgMetersSquared
  }

  public static final class EleArm {
    public static final int beamBreakPort = 0;

    public static final CANDeviceID armMotorID = new CANDeviceID(12, kCanivoreName);
    public static final MechanismRatio armMotorRatio =
        new MechanismRatio(1, (28.0 / 10.0) * (60.0 / 14.0) * (60.0 / 16.0));
    public static final boolean armMotorInvert = true;

    public static final CANDeviceID redirectMotorID = new CANDeviceID(13, kCanivoreName);
    public static final MechanismRatio redirectMotorRatio = new MechanismRatio(12, 18);
    public static final boolean redirectMotorInvert = true;

    public static final CANDeviceID feedMotorID = new CANDeviceID(14, kCanivoreName);
    public static final MechanismRatio feedMotorRatio = new MechanismRatio(16, 24);
    public static final boolean feedMotorInvert = true;

    public static final CANDeviceID lowerMotorID = new CANDeviceID(15, kCanivoreName);
    public static final MechanismRatio lowerMotorRatio = new MechanismRatio(1, 1);
    public static final boolean lowerMotorInvert = false;

    public static final CANDeviceID upperMotorID = new CANDeviceID(16, kCanivoreName);
    public static final MechanismRatio upperMotorRatio = new MechanismRatio(1, 1);
    public static final boolean upperMotorInvert = false;

    public static final ArmFeedforward armFeedForward = new ArmFeedforward(0.0, 0.3, 0.6);
    public static final Constraints armTrapConstraints =
        new Constraints(12.5, 80.0); // rad/s and rad/s^2
    public static final Constraints armSlowTrapConstraints =
        new Constraints(6.0, 80.0); // rad/s and rad/s^2
    public static final int armPositionPIDSlot = 0;
    public static final PIDConfig armPositionPIDConfig = new PIDConfig(3.0, 0.0, 0.01);

    public static final SimpleMotorFeedforward redirectRollerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.03);
    public static final int redirectVelocityPIDSlot = 0;
    public static final PIDConfig redirectVelocityPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final int redirectPositionPIDSlot = 1;
    public static final PIDConfig redirectPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    public static final SimpleMotorFeedforward feedRollerFeedforward =
        new SimpleMotorFeedforward(0.1, 0.028);
    public static final int feedVelocityPIDSlot = 0;
    public static final PIDConfig feedVelocityPIDConfig = new PIDConfig(0.1, 0.0, 0.0);
    public static final int feedPositionPIDSlot = 1;
    public static final PIDConfig feedPositionPIDConfig = new PIDConfig(30.0, 0.0, 0.0);

    public static final SimpleMotorFeedforward EleArmFeedforward =
        new SimpleMotorFeedforward(0.0, 0.019);
    public static final int EleArmVelocityPIDSlot = 0;
    public static final PIDConfig EleArmVelocityPIDConfig = new PIDConfig(0.2, 0.0, 0.0);

    // TODO: Use real values
    public static final double bootAbsPositionOffset = Units.degreesToRadians(-1.6);
    public static final double minAngle = Units.degreesToRadians(-128.0); // rads (trap position)
    public static final double maxAngle = Units.degreesToRadians(55.0); // rads (stow position)
    public static final double startingAngle = maxAngle + bootAbsPositionOffset;
    public static final double cgOffset = Units.degreesToRadians(30.0);

    public static final double climbAngle = Units.degreesToRadians(-45.0); // rads (trap position)
    public static final double trapAngle = Units.degreesToRadians(-128.0); // rads (trap position)

    public static final double intakeAngle = Units.degreesToRadians(45);
    public static final double intakeAngleTolerance = Units.degreesToRadians(5);

    public static final double subwooferLaunchAngle = Units.degreesToRadians(55);
    public static final double podiumLaunchAngle = Units.degreesToRadians(30);
    public static final double feedLaunchAngle = Units.degreesToRadians(45);
    public static final double launchAngleTolerance = Units.degreesToRadians(2);
    public static final double launchVelocity = 550.0; // rads/s
    public static final double launchFeedShotVelocity = 310.0; // rads/s
    public static final double launchVelocityTolerance = 50.0; // rads/s
    public static final double autoLaunchStartVelocity = 300.0; // rads/s

    public static final double scoreAmpArmAngle = Units.degreesToRadians(-105.0); // rads
    public static final double scoreAmpArmAngleTolerance = Units.degreesToRadians(2); // rads

    public static final double intakeFromSourceAngle = Units.degreesToRadians(55); // rads
    public static final double intakeFromSourceLaunchVelocity = -100; // rads/s
    public static final double intakeFromSourceFeedPower = -0.25;

    public static final double intakeFeedVelocity = 70; // rad/s
    public static final double scoreAmpFeedVelocity = 300; // rad/s
    public static final double scoreSpeakerFeedVelocity = 300; // rad/s
    public static final double scoreTrapFeedVelocity = 300; // rad/s
    public static final double scoreTrapReverseVelocity = 50; // rad/s

    public static final double shotVelocity = 20.0; // m/s
    public static final Transform2d robotToEleArm =
        new Transform2d(Units.inchesToMeters(12.0), 0.0, new Rotation2d());
    public static final double EleArmHeight = Units.inchesToMeters(21);

    public static final double rollerBeamBreakOffset = 1.5 * Math.PI; // rads

    // For simulation.
    public static final double simArmMOI = 0.3; // kgMetersSquared
    public static final double simArmCGLength = Units.inchesToMeters(4.0); // m
    public static final double simWheelMOI = 0.01; // kgMetersSquared
    public static final double simRollerMOI = 0.003; // kgMetersSquared
  }


  


  public static final class Viz {
    public static final double xOffset = Units.inchesToMeters(12.0);
    public static final double intakePivotX = xOffset + Units.inchesToMeters(27.25);
    public static final double intakePivotY = Units.inchesToMeters(11.25);
    public static final double intakeArmLength = Units.inchesToMeters(14.0);
    public static final double elevatorBaseX = xOffset + Units.inchesToMeters(12.0);
    public static final double elevatorBaseY = Units.inchesToMeters(3.0);
    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(90.0);
    public static final double elevatorBaseLength = Units.inchesToMeters(24.0);
    public static final double elevatorCarriageLength = Units.inchesToMeters(24.0);
    public static final double EleArmArmPivotX = Units.inchesToMeters(17.0);
    public static final double EleArmArmLength = Units.inchesToMeters(15.0);
    public static final double EleArmWheelX = Units.inchesToMeters(-3.0);
    public static final double EleArmTopWheelY = Units.inchesToMeters(4.625);
    public static final double EleArmBottomWheelY = Units.inchesToMeters(-0.625);
    public static final double EleArmFeedRollerX = Units.inchesToMeters(12.0);
    public static final double EleArmFeedRollerY = Units.inchesToMeters(1.875);
    public static final double EleArmRedirectRollerX = Units.inchesToMeters(12.0);
    public static final double EleArmRedirectRollerY = Units.inchesToMeters(5.125);

    public static final double angularVelocityScalar = 0.01;
  }




}