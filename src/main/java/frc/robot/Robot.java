// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.io.IOException;
import java.io.Serial;
import java.text.ParseException;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  private String autoName, newAutoName;

  private final Field2d m_field = new Field2d();
  
  Optional<Alliance> ally = DriverStation.getAlliance();
  Optional<Alliance> newAlly;
  private Vision vision;

  public Robot() {
    m_robotContainer = new RobotContainer();
    vision = new Vision();
  }

  // @Override
  // public void robotInit(){
  // PathfindingCommand.warmupCommand().schedule();
  // }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    // if (kUseLimelight) {
    //   var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    //   if (llMeasurement != null) {
    //     RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
    //   }
    // }
    try{
      var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(
            est -> {
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = vision.getEstimationStdDevs();

                RobotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
    } catch (Exception e) {
      e.printStackTrace();
    }

  if (m_robotContainer.driver.pov(0).getAsBoolean() == true){
      RobotContainer.elevator.setHeightLocation(4);
  } else if (m_robotContainer.driver.pov(90).getAsBoolean() == true){
    RobotContainer.elevator.setHeightLocation(3);

  } else if (m_robotContainer.driver.pov(180).getAsBoolean() == true){
    RobotContainer.elevator.setHeightLocation(2);

  } else if (m_robotContainer.driver.pov(270).getAsBoolean() == true){
    RobotContainer.elevator.setHeightLocation(1);

  }
  
  double leftY = m_robotContainer.operator.getLeftY();
  if (Math.abs(leftY) > 0.1) { // Deadband of 0.1
    if (m_robotContainer.operator.leftBumper().getAsBoolean() == true) {
      RobotContainer.climber.setTargetAdjust(leftY);
    }
  }

}

  @Override
  public void disabledInit() {
    autoName = "";

    Command resetGryo = new Command()
    {
        public boolean runsWhenDisabled()
        {
            return true;
        }

        public void initialize()
        {
            RobotContainer.drivetrain.getPigeon2().reset();
        }
        public boolean isFinished()
        {
            return true;
        }
    };
 
    SmartDashboard.putData("Reset Gyro", resetGryo);
  }

  @Override
  public void disabledPeriodic() { 
    ally = DriverStation.getAlliance();
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName | ally != newAlly) {
        newAlly = ally;
        autoName = newAutoName;
        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
            System.out.println("Displaying " + autoName);
            try {
                List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                        if (ally.isPresent()) {
                          if (ally.get() == Alliance.Red) {
                            poses.addAll(path.getAllPathPoints().stream()
                            .map(point -> new Pose2d(Constants.Pose.feildFlip - point.position.getX(),Constants.Pose.feildFlipy - point.position.getY(), new Rotation2d()))
                          .collect(Collectors.toList()));
                          Elastic.selectTab("RED");
                          }
                          if (ally.get() == Alliance.Blue) {
                            poses.addAll(path.getAllPathPoints().stream()
                            .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                          .collect(Collectors.toList()));
                          Elastic.selectTab("BLUE");
                          }
                        }
                        else {
                            System.out.println("No alliance found");
                            poses.addAll(path.getAllPathPoints().stream()
                            .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                          .collect(Collectors.toList()));
                        }
                }
              
                m_field.getObject("path").setPoses(poses);
            } catch (IOException e) {
                e.printStackTrace();
            } catch (Exception e) {
                if (e instanceof ParseException) {
                    e.printStackTrace();
                } else {
                  e.printStackTrace();
                }
            }
        }
    }
    SwerveDriveState state = RobotContainer.drivetrain.getState();
    Pose2d pose = state.Pose;
    m_field.setRobotPose(pose);
    SmartDashboard.putData(m_field);
    
}
  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoName = "";
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }

  @Override
  public void autonomousPeriodic() {
    ally = DriverStation.getAlliance();
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName | ally != newAlly) {
      newAlly = ally;
        autoName = newAutoName;
        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
            System.out.println("Displaying " + autoName);
            try {
                List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                List<Pose2d> poses = new ArrayList<>();
                for (PathPlannerPath path : pathPlannerPaths) {
                  if (ally.isPresent()) {
                    if (ally.get() == Alliance.Red) {
                      poses.addAll(path.getAllPathPoints().stream()
                      .map(point -> new Pose2d(Constants.Pose.feildFlip - point.position.getX(),Constants.Pose.feildFlipy - point.position.getY(), new Rotation2d()))
                    .collect(Collectors.toList()));
                    Elastic.selectTab("RED");
                    }
                    if (ally.get() == Alliance.Blue) {
                      poses.addAll(path.getAllPathPoints().stream()
                      .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                    .collect(Collectors.toList()));
                    Elastic.selectTab("BLUE");
                    }
                  }
                  else {
                      System.out.println("No alliance found");
                      poses.addAll(path.getAllPathPoints().stream()
                      .map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                    .collect(Collectors.toList()));
                  }
                }
                
                m_field.getObject("path").setPoses(poses);
            } catch (IOException e) {
                e.printStackTrace();
            } catch (Exception e) {
                if (e instanceof ParseException) {
                    e.printStackTrace();
                } else {
                  e.printStackTrace();
                }
            }
        }
    }
    SwerveDriveState state = RobotContainer.drivetrain.getState();
    Pose2d pose = state.Pose;
    m_field.setRobotPose(pose);
    SmartDashboard.putData(m_field);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    SwerveDriveState state = RobotContainer.drivetrain.getState();
    Pose2d pose = state.Pose;
    m_field.getObject("path").setPoses();
    m_field.setRobotPose(pose);
    SmartDashboard.putData(m_field);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
// Update drivetrain simulation

SwerveDriveState state = RobotContainer.drivetrain.getState();
Pose2d pose = state.Pose;
// Update camera simulation
vision.simulationPeriodic(pose);

var debugField = vision.getSimDebugField();
debugField.getObject("EstimatedRobot").setPose(pose);


  }
}
