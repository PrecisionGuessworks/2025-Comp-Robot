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
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.273); // 16T #25
    public static final MechanismRatio motorRatio =
        new MechanismRatio(
            1, (28.0 / 10.0) * (1.0 / 4.0) * (42.0 / 18.0), Math.PI * sprocketPitchDiameter);
    public static final boolean motorInvert = false;
    public static final int motorPositionSlot = 0;
    public static final PIDConfig motorPIDConfig = new PIDConfig(3, 0, 0.1, 0, 0.12, 0, 0.4);
    public static final double maxVelocity = 2.0; // m/s
    public static final double maxAcceleration = 30.0; // m/s^2
    public static final double maxJerk = 0.0; // m/s^3 (0 disables jerk limit)

    // TODO: use real numbers
    public static final double minHeight = 0.0; // m
    public static final double powerCutoffHeight = Units.inchesToMeters(0.5); // m
    public static final double maxHeight = Units.inchesToMeters(16.0); // m
    public static final double stowHeight = Units.inchesToMeters(1); // m
    public static final double stowTolerance = Units.inchesToMeters(0.25); // m
    public static final double scoreAmpHeight = Units.inchesToMeters(16.0); // m
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



  


  public static final class Viz {
    public static final double xOffset = Units.inchesToMeters(12.0);
    
    public static final double elevatorBaseX = xOffset + Units.inchesToMeters(12.0);
    public static final double elevatorBaseY = Units.inchesToMeters(3.0);
    public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(90.0);
    public static final double elevatorBaseLength = Units.inchesToMeters(24.0);
    public static final double elevatorCarriageLength = Units.inchesToMeters(24.0);


    public static final double angularVelocityScalar = 0.01;
  }




}