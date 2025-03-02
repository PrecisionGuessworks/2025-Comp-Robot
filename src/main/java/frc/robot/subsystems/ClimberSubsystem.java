// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
  private final QuixTalonFX m_motor =
      new QuixTalonFX(
          Constants.Climber.motorID,
          Constants.Climber.motorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(60.0)
              .setInverted(Constants.Climber.motorInvert)
              .setPIDConfig(Constants.Climber.motorPositionSlot, Constants.Climber.motorPIDConfig)
              .setMotionMagicConfig(
                  Constants.Climber.maxVelocity,
                  Constants.Climber.maxAcceleration,
                  Constants.Climber.maxJerk)
              .setReverseSoftLimit(Constants.Climber.minHeight)
              .setForwardSoftLimit(Constants.Climber.maxHeight));

  // private final QuixTalonFX m_follower = new QuixTalonFX(
  //     Constants.Climber.followerID,
  //     m_motor,
  //     Constants.Climber.followerInvert,
  //     QuixTalonFX.makeDefaultConfig().setBrakeMode()
  //     .setSupplyCurrentLimit(40.0)
  //     .setStatorCurrentLimit(60.0)
  //     .setInverted(Constants.Climber.motorInvert)
  //     .setPIDConfig(Constants.Climber.motorPositionSlot, Constants.Climber.motorPIDConfig)
  //     .setMotionMagicConfig(
  //         Constants.Climber.maxVelocity,
  //         Constants.Climber.maxAcceleration,
  //         Constants.Climber.maxJerk)
  //     .setReverseSoftLimit(Constants.Climber.minHeight)
  //     .setForwardSoftLimit(Constants.Climber.maxHeight));

  private double m_setTargetHeight = Constants.Climber.stowHeight;
  private double m_targetHeight = Constants.Climber.stowHeight;

  public ClimberSubsystem(Link2d climberCarriageViz) {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_climberCarriageViz = climberCarriageViz;
  }


  public double getHeight() {
    return  Constants.Climber.motorRatio.sensorRadiansToMechanismPosition(m_motor.getSensorPosition());
  }

  public void setHeight(double targetHeight) {
    m_setTargetHeight = targetHeight;
  }

  public void setTargetAdjust(double adjust) {
    m_setTargetHeight += adjust * Constants.defaultPeriodSecs *0.0254 *0.8; 
    // Constants.defaultPeriodSecs converts seconds to cycles, 0.0254 is inches to meters, 0.1 is a scaling factor
    
  }

  public void setZero() {
    m_motor.setSensorPosition(0.0);
  }

  public void setZero(double height) {
    m_motor.setSensorPosition(height);
  }
 

  public boolean isAtHeight(double height, double tolerance) {
    return Math.abs(height - getHeight()) <= tolerance;
  }
  private double armAngle = 0;
  private double wristAngle = 0;

  @Override
  public void periodic() {
    armAngle = RobotContainer.arm.getArmAngle();
    wristAngle = RobotContainer.arm.getWristAngle();
  //  if (armAngle < 92 && wristAngle < 92){
  //     m_targetHeight = m_setTargetHeight;
  //   } else {
  //    // m_targetHeight = Constants.Climber.stowHeight;
  //   }
    // This method will be called once per scheduler run
    m_targetHeight = m_setTargetHeight;
    m_motor.setDynamicMotionMagicPositionSetpoint(
        Constants.Climber.motorPositionSlot,
        m_targetHeight,
        Constants.Climber.maxVelocity,
        Constants.Climber.maxAcceleration,
        Constants.Climber.maxJerk);



    SmartDashboard.putNumber(
        "Climber: Current Height (in)", Units.metersToInches(getHeight()));
    SmartDashboard.putNumber(
        "Climber: Target Height (in)", Units.metersToInches(Constants.Climber.motorRatio.sensorRadiansToMechanismPosition(m_motor.getClosedLoopReference())));

  

    m_motor.logMotorState();
    //m_follower.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_climberSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Climber.motorRatio.reduction(),
          Constants.Climber.simCarriageMass,
          Constants.Climber.sprocketPitchDiameter * 0.5,
          Constants.Climber.minHeight,
          Constants.Climber.maxHeight,
          true,
          0);

  // Visualization
  private final Link2d m_climberCarriageViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_climberSim.setInput(m_motor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_climberSim.update(TimedRobot.kDefaultPeriod);
    m_motor.setSimSensorPositionAndVelocity(
        m_climberSim.getPositionMeters(),
        // m_climberSim.getVelocityMetersPerSecond(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Climber.motorRatio);

    // Update carriage viz.
    m_climberCarriageViz.setRelativeTransform(
        new Transform2d(m_climberSim.getPositionMeters(), 0.0, new Rotation2d()));
  }
  // --- END STUFF FOR SIMULATION ---
}
