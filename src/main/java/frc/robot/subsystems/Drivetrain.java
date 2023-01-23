// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Represents a differential drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static  final class Odometry 
  {
    public static Pose2d construct_p2(double x, double y, double theta)
    {
        return new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    }

    private Pose2d p2;
    private final Drivetrain drive;
    private final DifferentialDriveKinematics KINE;
    private final DifferentialDriveOdometry odom;

    public Odometry(Drivetrain drive)
    {
        KINE = new DifferentialDriveKinematics(Drivetrain.kTrackWidth);
        this.drive = drive;
        odom = new DifferentialDriveOdometry(drive.m_gyro.getRotation2d(), 0D, 0D);
    }
    
    public DifferentialDriveKinematics expose_kinematics()
    {
        return KINE;
    }

    public DifferentialDriveOdometry expose_odometry()
    {
        return odom;
    }

    public Rotation2d expose_gyro_r2d()
    {
        return drive.m_gyro.getRotation2d();
    }

    public double left_encoder()
    {
        return drive.m_leftEncoder.getDistance();
    }

    public double right_encoder()
    {
        return drive.m_rightEncoder.getDistance();
    }

    public void update()
    {
        p2 = odom.update(drive.m_gyro.getRotation2d(), drive.m_leftEncoder.getDistance(), drive.m_RightEncoder.getDistance());
    }
  }

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  public static final double kTrackWidth = 0.381 * 2; // meters
  public static final double kWheelRadius = 0.0508; // meters
  public static final int kEncoderResolution = 4096;

  private final MotorController m_leftLeader = new WPI_TalonSRX(1);
  private final MotorController m_leftFollower = new WPI_TalonSRX(2); 
  private final MotorController m_rightLeader = new WPI_TalonSRX(3);
  private final MotorController m_rightFollower = new WPI_TalonSRX(4);

  public final Encoder m_leftEncoder = new Encoder(0, 1);
  public final Encoder m_rightEncoder = new Encoder(2, 3);

  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightLeader, m_rightFollower);

  public final AnalogGyro m_gyro = new AnalogGyro(0);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final Odometry m_odometry;
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    m_gyro.reset();
 
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true); // changed back to true

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_odometry = new Odometry(this);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    setSpeeds(m_odometry.expose_kinematics().toWheelSpeeds(new ChassisSpeeds(xSpeed, 0D, rot)));
  }

  public void resetOdom()
  {
    m_odometry.expose_odometry().resetPosition(m_gyro.getRotation2d(), m_odometry.left_encoder(), m_odometry.right_encoder())
  }

  @Override public void periodic()
  {
    m_odometry.update();
  }
}
