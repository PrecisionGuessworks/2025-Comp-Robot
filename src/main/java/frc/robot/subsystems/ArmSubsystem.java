// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  //public final DigitalInput m_beamBreak = new DigitalInput(Constants.Arm.beamBreakPort);

  private final QuixTalonFX m_rollerMotor =
      new QuixTalonFX(
          Constants.Arm.rollerMotorID,
          Constants.Arm.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.rollerMotorInvert)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setPIDConfig(Constants.Arm.rollerVelocityPIDSlot, Constants.Arm.rollerPositionPIDConfig));

  private final QuixTalonFX m_armMotor =
      new QuixTalonFX(
          Constants.Arm.armMotorID,
          Constants.Arm.armMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.armMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(40.0)
              .setMotionMagicConfig(
                  Constants.Arm.ArmConstraints.maxVelocity,
                  Constants.Arm.ArmConstraints.maxAcceleration,
                  1)
              .setPIDConfig(Constants.Arm.armPositionPIDSlot, Constants.Arm.armPositionPIDConfig)
              .setBootPositionOffset(Constants.Arm.armStartingAngle)
              .setReverseSoftLimit(Constants.Arm.armMinAngle)
              .setForwardSoftLimit(Constants.Arm.armMaxAngle));

  private double m_targetAngle = Constants.Arm.armStartingAngle;
  private Timer m_lastPieceTimer = new Timer();

  public ArmSubsystem(Link2d ArmArmViz, Link2d ArmWristViz, Link2d ArmRollerViz) {
    m_lastPieceTimer.start();
    m_lastPieceTimer.reset();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_ArmArmViz = ArmArmViz;
    m_ArmRollerViz = ArmRollerViz;
    m_ArmWristViz = ArmWristViz;
  }

  // public boolean hasPiece() {
  //   return m_beamBreak.get();
  // }

  public boolean recentlyHadPiece() {
    return m_lastPieceTimer.get() < 1.0;
  }

  public double getAngle() {
    return m_armMotor.getSensorPosition();
  }

  public void setArmAngle(double targetAngle) {
    m_targetAngle = targetAngle;
  }

  public boolean isAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_armMotor.getSensorPosition()) <= tolerance;
  }

  public void setRollerVelocity(double velocity) {
    if (velocity == 0.0) {
      m_rollerMotor.setPercentOutput(0.0);
    } else {
      m_rollerMotor.setVelocitySetpoint(
          Constants.Arm.rollerVelocityPIDSlot,
          velocity,
          Constants.Arm.rollerFeedforward.calculate(velocity));
    }
  }

  // public void disabledInit() {
  //   m_armMotor.setBrakeMode(true);
  // }

  // public void disabledExit() {
  //   m_armMotor.setBrakeMode(false);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (hasPiece()) {
    //   m_lastPieceTimer.reset();
    // }

    //SmartDashboard.putBoolean("Arm: Beam Break", m_beamBreak.get());

    m_armMotor.setMotionMagicPositionSetpoint(
        Constants.Arm.armPositionPIDSlot, m_targetAngle);

    SmartDashboard.putNumber(
        "Arm: Current Angle (deg)", Units.radiansToDegrees(m_armMotor.getSensorPosition()));
    SmartDashboard.putNumber(
        "Arm: Target Angle (deg)",
        Units.radiansToDegrees(m_armMotor.getClosedLoopReference()));
    SmartDashboard.putNumber(
        "Arm: Current Velocity (deg per sec)",
        Units.radiansToDegrees(m_armMotor.getSensorVelocity()));
    SmartDashboard.putNumber(
        "Arm: Target Velocity (deg per sec)",
        Units.radiansToDegrees(m_armMotor.getClosedLoopReferenceSlope()));
    SmartDashboard.putNumber(
        "Arm: Current Roller Velocity (rad per sec)", m_rollerMotor.getSensorVelocity());

    m_rollerMotor.logMotorState();
    m_armMotor.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Arm.armMotorRatio.reduction(),
          Constants.Arm.simArmMOI,
          Constants.Arm.simArmCGLength,
          Constants.Arm.armMinAngle,
          Constants.Arm.armMaxAngle,
          true, // Simulate gravity
          Constants.Arm.armStartingAngle);
  static final DCMotor m_simMotor = DCMotor.getKrakenX60Foc(1);
  private static final FlywheelSim m_rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_simMotor,
              Constants.Arm.simRollerMOI,
              Constants.Arm.rollerMotorRatio.reduction()),
          m_simMotor);

          
  // Visualization
  private final Link2d m_ArmArmViz;
  private final Link2d m_ArmRollerViz;
  private final Link2d m_ArmWristViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_armSim.setInput(m_armMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_armMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads() - Constants.Arm.armStartingAngle,
        // m_armSim.getVelocityRadPerSec(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);

    m_rollerSim.setInput(m_rollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_rollerMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);

    // Update arm viz.
    m_ArmArmViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmArmPivotX,
            0,
            Rotation2d.fromRadians(m_armSim.getAngleRads() + Units.degreesToRadians(- Constants.Viz.elevatorAngle.getDegrees()))));
    m_ArmRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmArmLength,
            0.0,
            Rotation2d.fromRadians(
                m_ArmRollerViz.getRelativeTransform().getRotation().getRadians()
                    + m_rollerSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
  }
  // --- END STUFF FOR SIMULATION ---
}
