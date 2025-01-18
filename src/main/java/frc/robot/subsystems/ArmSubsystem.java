// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.planning.QuixTrapezoidProfile;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  public final DigitalInput m_beamBreak = new DigitalInput(Constants.Arm.beamBreakPort);

  // This motor controls the upper launch wheels
  private final QuixTalonFX m_armRollerMotor =
      new QuixTalonFX(
          Constants.Arm.rollerMotorID,
          Constants.Arm.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.rollerMotorInvert)
              .setPIDConfig(
                  Constants.Arm.ArmVelocityPIDSlot,
                  Constants.Arm.ArmVelocityPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(120.0));



  // This motor controls the angle of the arm (Arm)
  private final QuixTalonFX m_armAngleMotor =
      new QuixTalonFX(
          Constants.Arm.armMotorID,
          Constants.Arm.armMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.armMotorInvert)
              .setBrakeMode()
              .setPIDConfig(
                  Constants.Arm.armPositionPIDSlot, Constants.Arm.armPositionPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBootPositionOffset(Constants.Arm.startingAngle)
              .setReverseSoftLimit(Constants.Arm.minAngle)
              .setForwardSoftLimit(Constants.Arm.maxAngle));



  private final QuixTalonFX m_armWristMotor =
      new QuixTalonFX(
          Constants.Arm.armMotorID,
          Constants.Arm.armMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.armMotorInvert)
              .setBrakeMode()
              .setPIDConfig(
                  Constants.Arm.armPositionPIDSlot, Constants.Arm.armPositionPIDConfig)
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0)
              .setBootPositionOffset(Constants.Arm.startingAngle)
              .setReverseSoftLimit(Constants.Arm.minAngle)
              .setForwardSoftLimit(Constants.Arm.maxAngle));

  private QuixTrapezoidProfile m_armProfile;
  private final Timer m_armTimer = new Timer();
  private QuixTrapezoidProfile m_WristProfile;
  private final Timer m_WristTimer = new Timer();
  private State m_armState = new State(m_armAngleMotor.getSensorPosition(), 0.0);
  private State m_WristState = new State(m_armWristMotor.getSensorPosition(), 0.0);

  private boolean m_beamBreakLastState = false;
  private Double m_beamBreakFeedPosition = null;
  private Double m_beamBreakRedirectPosition = null;

  public ArmSubsystem(
      Link2d ArmArmViz,
      Link2d ArmWristViz,
      Link2d ArmWheelViz) {

    m_armProfile =
        new QuixTrapezoidProfile(
            Constants.Arm.armTrapConstraints,
            new State(Constants.Arm.startingAngle, 0.0),
            m_armState);
    m_armTimer.start();

    m_WristProfile =
        new QuixTrapezoidProfile(
            Constants.Arm.armTrapConstraints,
            new State(Constants.Arm.startingAngle, 0.0),
            m_WristState);
    m_WristTimer.start();
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_ArmArmViz = ArmArmViz;
    m_ArmWristViz = ArmWristViz;
    m_ArmRollerViz = ArmWheelViz;

  }

  public boolean hasPiece() {
    return m_beamBreak.get();
  }

  public boolean readyForIntake() {
    return !hasPiece()
        && isAtAngle(Constants.Arm.intakeAngle, Constants.Arm.intakeAngleTolerance);
  }

  public double getArmAngle() {
    return m_armAngleMotor.getSensorPosition();
  }

  public boolean isAtLaunchVelocity(double launchVelocity, double tolerance) {
    return Math.abs(launchVelocity - m_armRollerMotor.getSensorVelocity()) <= tolerance
        && Math.abs(launchVelocity - m_armRollerMotor.getSensorVelocity()) <= tolerance;
  }


  public boolean isAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_armAngleMotor.getSensorPosition()) <= tolerance;
  }

  public void setArmAngle(double targetArmAngle) {
    m_armProfile =
        new QuixTrapezoidProfile(
            Constants.Arm.armTrapConstraints,
            new State(
                MathUtils.clamp(
                    targetArmAngle, Constants.Arm.minAngle, Constants.Arm.maxAngle),
                0.0),
            m_armState);
    m_armTimer.reset();
  }

  public void setWristAngle(double targetArmAngle) {
    m_armProfile =
        new QuixTrapezoidProfile(
            Constants.Arm.armTrapConstraints,
            new State(
                MathUtils.clamp(
                    targetArmAngle, Constants.Arm.minAngle, Constants.Arm.maxAngle),
                0.0),
                m_WristState);
    m_WristTimer.reset();
  }

  public void setArmAngleSlow(double targetArmAngle) {
    m_armProfile =
        new QuixTrapezoidProfile(
            Constants.Arm.armSlowTrapConstraints,
            new State(
                MathUtils.clamp(
                    targetArmAngle, Constants.Arm.minAngle, Constants.Arm.maxAngle),
                0.0),
            m_armState);
    m_armTimer.reset();
  }

  public void setRollerVelocity(double velocity) {
    final double feedffVolts = Constants.Arm.feedRollerFeedforward.calculate(velocity);
    if (velocity == 0.0) {
      m_armRollerMotor.setPercentOutput(0.0);
    } else {
      // m_feedRollerMotor.setVelocitySetpoint(
      //     Constants.Arm.feedVelocityPIDSlot, velocity, feedffVolts);
    }
  }

  public void setRollerPower(double power) {
    m_armRollerMotor.setPercentOutput(power);
  }

  public void setRedirectPower(double power) {
    m_armRollerMotor.setPercentOutput(power);
  }

  public void stopRoller() {
    m_armRollerMotor.setPercentOutput(0.0);
  }

  public void setRedirectVelocity(double velocity) {
    final double redirectffVolts = Constants.Arm.redirectRollerFeedforward.calculate(velocity);
    if (velocity == 0.0) {
      m_armRollerMotor.setPercentOutput(0.0);
    } else {
      m_armRollerMotor.setVelocitySetpoint(
          Constants.Arm.redirectVelocityPIDSlot, velocity, redirectffVolts);
    }
  }

  public void moveFeedAndRedirectToPositionOffset(double rads) {
    if (m_beamBreakFeedPosition != null && m_beamBreakRedirectPosition != null) {
      m_armRollerMotor.setPositionSetpoint(
          Constants.Arm.feedPositionPIDSlot, m_beamBreakFeedPosition + rads);
          m_armRollerMotor.setPositionSetpoint(
          Constants.Arm.redirectPositionPIDSlot, m_beamBreakRedirectPosition + rads);
    }
  }

  /** Velocity in rad/s */
  public void setLaunchVelocity(double velocity) {
    final double ffVolts = Constants.Arm.ArmFeedforward.calculate(velocity);

    if (velocity == 0.0) {
      m_armRollerMotor.setPercentOutput(0.0);
      m_armRollerMotor.setPercentOutput(0.0);
    } else {
      m_armRollerMotor.setVelocitySetpoint(
          Constants.Arm.ArmVelocityPIDSlot, velocity, ffVolts);
          m_armRollerMotor.setVelocitySetpoint(
          Constants.Arm.ArmVelocityPIDSlot, velocity, ffVolts);
    }
  }

  public double setLinearLaunchVelocity(double metersPerSecond) {
    // Linear approximation of launch velocity.
    final double radsPerSec =
        (metersPerSecond / Constants.Arm.shotVelocity) * Constants.Arm.launchVelocity;
    setLaunchVelocity(radsPerSec);
    return radsPerSec;
  }

  public void disabledInit() {
    m_armAngleMotor.setBrakeMode(true);
  }

  public void disabledExit() {
    m_armAngleMotor.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      // Update state to sensor state when disabled to prevent jumps on enable.
      m_armState = new State(m_armAngleMotor.getSensorPosition(), 0.0);
      setArmAngle(m_armAngleMotor.getSensorPosition());
    }

    // Track the position where the beam break is broken.
    if (m_beamBreak.get()) {
      if (!m_beamBreakLastState) {
        m_beamBreakFeedPosition = m_armRollerMotor.getSensorPosition();
       // m_beamBreakRedirectPosition = m_armRollerMotor.getSensorPosition();
      }
      m_beamBreakLastState = true;
    } else {
      m_beamBreakFeedPosition = null;
      m_beamBreakRedirectPosition = null;
      m_beamBreakLastState = false;
    }

    SmartDashboard.putBoolean("Arm: Beam Break", m_beamBreak.get());

    m_armState = m_armProfile.calculate(m_armTimer.get());
    m_armAngleMotor.setPositionSetpoint(
        Constants.Arm.armPositionPIDSlot,
        m_armState.position,
        // Arm angle is defined as positive when the Arm is pointed up, but the CG is on the
        // other side with some offset, so we need to negate the angle and voltage for FF.
        -Constants.Arm.armFeedForward.calculate(
            -m_armState.position + Constants.Arm.cgOffset, -m_armState.velocity));

    SmartDashboard.putNumber(
        "Arm: Current Arm Angle (deg)",
        Units.radiansToDegrees(m_armAngleMotor.getSensorPosition()));
    SmartDashboard.putNumber(
        "Arm: Target Arm Angle (deg)", Units.radiansToDegrees(m_armState.position));
    SmartDashboard.putNumber(
        "Arm: Arm Angle Error (deg)",
        Units.radiansToDegrees(m_armState.position - m_armAngleMotor.getSensorPosition()));

    SmartDashboard.putNumber(
        "Arm: Current Roller Velocity (rad per sec)",
        m_armRollerMotor.getSensorVelocity());
    

    SmartDashboard.putNumber(
        "Wrist: Current Roller Current (A)", m_armRollerMotor.getStatorCurrent());



        SmartDashboard.putNumber(
        "Wrist: Current Wrist Angle (deg)",
        Units.radiansToDegrees(m_armWristMotor.getSensorPosition()));
    SmartDashboard.putNumber(
        "Wrist: Target Wrist Angle (deg)", Units.radiansToDegrees(m_WristState.position));
    SmartDashboard.putNumber(
        "Wrist: Wrist Angle Error (deg)",
        Units.radiansToDegrees(m_WristState.position - m_armWristMotor.getSensorPosition()));

    SmartDashboard.putNumber(
        "Wrist: Current Roller Velocity (rad per sec)",
        m_armRollerMotor.getSensorVelocity());
    

    SmartDashboard.putNumber(
        "Wrist: Current Roller Current (A)", m_armWristMotor.getStatorCurrent());
    m_armRollerMotor.logMotorState();
    m_armWristMotor.logMotorState();
    m_armAngleMotor.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  // Note that the arm simulated backwards because the sim requires zero angle to be gravity acting
  // down on the arm, but gravity acts "up" on the arm from the perspective of the launch angle.
  private static final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Arm.armMotorRatio.reduction(),
          Constants.Arm.simArmMOI,
          Constants.Arm.simArmCGLength,
          -Constants.Arm.maxAngle, // Arm is simulated backwards
          -Constants.Arm.minAngle, // Arm is simulated backwards
          true, // Simulate gravity
          Constants.Arm.startingAngle);

    private static final SingleJointedArmSim m_wristSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60Foc(1),
              Constants.Arm.wristMotorRatio.reduction(),
              Constants.Arm.wristArmMOI,
              Constants.Arm.simwristCGLength,
              -Constants.Arm.maxAngle, // Arm is simulated backwards
              -Constants.Arm.minAngle, // Arm is simulated backwards
              true, // Simulate gravity
              Constants.Arm.startingAngle);

  

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
    m_armSim.setInput(-m_armAngleMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(TimedRobot.kDefaultPeriod);
    m_wristSim.setInput(-m_armWristMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_wristSim.update(TimedRobot.kDefaultPeriod);
    // Arm is simulated backwards because gravity acting on a horizontal arm needs to be at
    // zero degrees
    m_armAngleMotor.setSimSensorPositionAndVelocity(
        -m_armSim.getAngleRads() - Constants.Arm.startingAngle,
        m_armSim.getVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Arm.armMotorRatio);

        m_armWristMotor.setSimSensorPositionAndVelocity(
          -m_wristSim.getAngleRads() - Constants.Arm.startingAngle,
          m_wristSim.getVelocityRadPerSec(),
          TimedRobot.kDefaultPeriod,
          Constants.Arm.wristMotorRatio);

        m_rollerSim.setInput(
          m_armRollerMotor.getPercentOutput() * RobotController.getBatteryVoltage());
        m_rollerSim.update(TimedRobot.kDefaultPeriod);
        m_armRollerMotor.setSimSensorVelocity(
      m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Arm.rollerMotorRatio);
    

    m_ArmArmViz.setRelativeTransform(
      
        new Transform2d(
            Constants.Viz.ArmArmPivotX,
            0.0,
            // TODO: Figure out how to do this without hardcoding
            Rotation2d.fromRadians(
                m_armSim.getAngleRads() - Constants.Viz.elevatorAngle.getRadians())));

    m_ArmWristViz.setRelativeTransform(
      
                new Transform2d(
                    Constants.Viz.ArmArmPivotX,
                    0.0,
                    // TODO: Figure out how to do this without hardcoding
                    Rotation2d.fromRadians(
                      m_ArmRollerViz.getRelativeTransform().getRotation().getRadians()
                      + m_rollerSim.getAngularVelocityRadPerSec()
                          * Constants.Viz.angularVelocityScalar)));

    m_ArmRollerViz.setRelativeTransform(
        new Transform2d(
            Constants.Viz.ArmRollerX,
            Constants.Viz.ArmRollerY,
            Rotation2d.fromRadians(
              m_ArmRollerViz.getRelativeTransform().getRotation().getRadians()
                    + m_rollerSim.getAngularVelocityRadPerSec()
                        * Constants.Viz.angularVelocityScalar)));
    
   
  }
  // --- END STUFF FOR SIMULATION ---
}
