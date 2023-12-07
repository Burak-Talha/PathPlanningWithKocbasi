// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  CANSparkMax frontLeft = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rearRight = new CANSparkMax(3, MotorType.kBrushless);

  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(frontRight, rearRight);
  MotorControllerGroup LeftControllerGroup = new MotorControllerGroup(frontLeft, rearLeft);

  DifferentialDrive diffDrive = new DifferentialDrive(LeftControllerGroup, rightControllerGroup);

  AHRS navx;
  RelativeEncoder rightEncoder;
  RelativeEncoder leftEncoder;
  Joystick joystick;

  DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(0.27);
  DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(differentialDriveKinematics, new Rotation2d(), 0, 0, new Pose2d());

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Sensors
    navx = new AHRS(Port.kMXP);
    rightEncoder = frontRight.getEncoder();
    leftEncoder = frontLeft.getEncoder();
    joystick = new Joystick(0);
    // Path Planning Ramsete
    AutoBuilder.configureRamsete(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::setSpeeds,
      new ReplanningConfig(),
      this
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    differentialDrivePoseEstimator.update(getRotation2d(), getLeftEncoderMeter(), getRightEncoderMeter());
  }

  public void drive(){
    diffDrive.arcadeDrive(joystick.getRawAxis(1)*0.6, joystick.getRawAxis(4)*0.6);
  }

  public void setSpeeds(ChassisSpeeds chassisSpeeds){
    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;

    double leftSpeed = vx + vy * 0.27 / 2;
    double rightSpeed = vx - vy * 0.27 / 2;

    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public double getYaw(){
    return navx.getYaw();
  }

  public Pose2d getPose(){
    return differentialDrivePoseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation2d(){
    return navx.getRotation2d();
  }

  public ChassisSpeeds getChassisSpeeds(){
    return new ChassisSpeeds(leftEncoder.getVelocity()/60, rightEncoder.getVelocity()/60, navx.getQuaternionW());
  }

  public double getLeftEncoderMeter(){
    return (leftEncoder.getPosition() / Constants.DriveTrainConstants.GEAR_RATIO) * Constants.DriveTrainConstants.CM_PERIMETER_OF_WHEEL;
  }
  public double getRightEncoderMeter(){
    return (rightEncoder.getPosition() / Constants.DriveTrainConstants.GEAR_RATIO) * Constants.DriveTrainConstants.CM_PERIMETER_OF_WHEEL;
  }

  public void resetSensors(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    navx.reset();
  }

  public void resetPose(Pose2d pose2d){
    resetSensors();
    differentialDrivePoseEstimator.resetPosition(pose2d.getRotation(), getLeftEncoderMeter(), getRightEncoderMeter(), pose2d);
  }

  public Command BasedSigmAutonomous(){
    PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("Straight Drive Path");
    return new FollowPathWithEvents(
      new FollowPathRamsete(
          pathPlannerPath,
          this::getPose, // Robot pose supplier
          this::getChassisSpeeds, // Current ChassisSpeeds supplier
          this::setSpeeds, // Method that will drive the robot given ChassisSpeeds
          new ReplanningConfig(), // Default path replanning config. See the API for the options here
          this // Reference to this subsystem to set requirements
      ),
      pathPlannerPath, // FollowPathWithEvents also requires the path
      this::getPose // FollowPathWithEvents also requires the robot pose supplier
  );
  }

}
