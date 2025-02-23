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

public class ElevatorSubsystem extends SubsystemBase {
  private final QuixTalonFX m_motor =
      new QuixTalonFX(
          Constants.Elevator.motorID,
          Constants.Elevator.motorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(10.0)
              .setStatorCurrentLimit(20.0)
              .setInverted(Constants.Elevator.motorInvert)
              .setPIDConfig(Constants.Elevator.motorPositionSlot, Constants.Elevator.motorPIDConfig)
              .setMotionMagicConfig(
                  Constants.Elevator.maxVelocity,
                  Constants.Elevator.maxAcceleration,
                  Constants.Elevator.maxJerk)
              .setReverseSoftLimit(Constants.Elevator.minHeight)
              .setForwardSoftLimit(Constants.Elevator.maxHeight));

  private final QuixTalonFX m_follower = new QuixTalonFX(
      Constants.Elevator.followerID,
      m_motor,
      Constants.Elevator.followerInvert,
      QuixTalonFX.makeDefaultConfig().setBrakeMode()
      .setSupplyCurrentLimit(10.0)
      .setStatorCurrentLimit(20.0)
      .setInverted(Constants.Elevator.motorInvert)
      .setPIDConfig(Constants.Elevator.motorPositionSlot, Constants.Elevator.motorPIDConfig)
      .setMotionMagicConfig(
          Constants.Elevator.maxVelocity,
          Constants.Elevator.maxAcceleration,
          Constants.Elevator.maxJerk)
      .setReverseSoftLimit(Constants.Elevator.minHeight)
      .setForwardSoftLimit(Constants.Elevator.maxHeight));

  private double m_setTargetHeight = Constants.Elevator.minHeight;
  private double m_targetHeight = Constants.Elevator.minHeight;
  public int m_HeightLocation = 4;
  private boolean Loc1 = false;
  private boolean Loc2 = false;
  private boolean Loc3 = false;
  private boolean Loc4 = false;

  public ElevatorSubsystem(Link2d elevatorCarriageViz) {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_elevatorCarriageViz = elevatorCarriageViz;
  }

  public boolean isAtScore(){
    if (m_HeightLocation == 4){
      return isAtHeight(Constants.Elevator.L4, 0.5);
    } else if (m_HeightLocation == 3){
      return isAtHeight(Constants.Elevator.L3, 0.5);
    } else if (m_HeightLocation == 2){
      return isAtHeight(Constants.Elevator.L2, 0.5);
    } else if (m_HeightLocation == 1){
      return isAtHeight(Constants.Elevator.L1, 0.5);
    } else {
      return false;
    }
  }

  public double getHeight() {
    return Constants.Elevator.motorRatio.sensorRadiansToMechanismPosition(m_motor.getSensorPosition());
  }

  public void setHeight(double targetHeight) {
    m_setTargetHeight = targetHeight;
  }
  public void setHeightLocation(int targetHeight) {
    m_HeightLocation = targetHeight;
  }
  public int getHeightLocation() {
    return m_HeightLocation;
  }

  public boolean isAtHeight(double height, double tolerance) {
    return Math.abs(height - m_motor.getSensorPosition()) <= tolerance;
  }
  private double armAngle = 0;
  private double wristAngle = 0;

  @Override
  public void periodic() {
    armAngle = RobotContainer.arm.getArmAngle();
    wristAngle = RobotContainer.arm.getWristAngle();
    // if (armAngle < 90 && wristAngle < 90 && m_setTargetHeight > Constants.Elevator.armStowHeight){
    //   m_targetHeight = m_setTargetHeight;
    // } else if (armAngle < 85 && m_setTargetHeight < Constants.Elevator.armStowHeight && !isAtHeight(Constants.Elevator.stowHeight, 3)){ 
    //   m_targetHeight = Constants.Elevator.armStowHeight;
    // } else if (wristAngle < 88 && m_setTargetHeight < Constants.Elevator.wristStowHeight && !isAtHeight(Constants.Elevator.stowHeight, 3)){ 
    //   m_targetHeight = Constants.Elevator.wristStowHeight; 
    // } else if (armAngle < 100 && armAngle > 91){ // intake
    //   m_targetHeight = Constants.Elevator.stowHeight;
    // } else if (armAngle > 100 && m_setTargetHeight <= Constants.Elevator.intakeHeight){
    //   m_targetHeight = m_setTargetHeight;
    // } else {
    //   m_targetHeight = Constants.Elevator.stowHeight;
    // }

    m_targetHeight = m_setTargetHeight;

    // This method will be called once per scheduler run
    m_motor.setDynamicMotionMagicPositionSetpoint(
        Constants.Elevator.motorPositionSlot,
        m_targetHeight,
        Constants.Elevator.maxVelocity,
        Constants.Elevator.maxAcceleration,
        Constants.Elevator.maxJerk);

    if (m_HeightLocation==1){
      Loc1 = true;
      Loc2 = false;
      Loc3 = false;
      Loc4 = false;
    } else if (m_HeightLocation==2){
      Loc1 = false;
      Loc2 = true;
      Loc3 = false;
      Loc4 = false;
    } else if (m_HeightLocation==3){
      Loc1 = false;
      Loc2 = false;
      Loc3 = true;
      Loc4 = false;
    } else if (m_HeightLocation==4){
      Loc1 = false;
      Loc2 = false;
      Loc3 = false;
      Loc4 = true;
    }


    SmartDashboard.putNumber(
        "Elevator: Current Height (in)", Units.metersToInches(getHeight()));
    SmartDashboard.putNumber(
        "Elevator: Target Height (in)", Units.metersToInches(Constants.Elevator.motorRatio.sensorRadiansToMechanismPosition(m_motor.getClosedLoopReference())));

    SmartDashboard.putBoolean(
          "L1", Loc1);
    SmartDashboard.putBoolean(
          "L2", Loc2);
    SmartDashboard.putBoolean(  
          "L3", Loc3);    
    SmartDashboard.putBoolean(
          "L4", Loc4);

    m_motor.logMotorState();
    m_follower.logMotorState();
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          Constants.Elevator.motorRatio.reduction(),
          Constants.Elevator.simCarriageMass,
          Constants.Elevator.sprocketPitchDiameter * 0.5,
          Constants.Elevator.minHeight,
          Constants.Elevator.maxHeight,
          true,
          0);

  // Visualization
  private final Link2d m_elevatorCarriageViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_elevatorSim.setInput(m_motor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(TimedRobot.kDefaultPeriod);
    m_motor.setSimSensorPositionAndVelocity(
        m_elevatorSim.getPositionMeters(),
        // m_elevatorSim.getVelocityMetersPerSecond(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Elevator.motorRatio);

    // Update carriage viz.
    m_elevatorCarriageViz.setRelativeTransform(
        new Transform2d(m_elevatorSim.getPositionMeters(), 0.0, new Rotation2d()));
  }
  // --- END STUFF FOR SIMULATION ---
}
