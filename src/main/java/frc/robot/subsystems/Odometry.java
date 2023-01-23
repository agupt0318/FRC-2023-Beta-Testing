package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public final class Odometry 
        extends CommandBase
{
    private Pose2d p2;

    private final Drivetrain drive;
    private final DifferentialDriveKinematics KINE;
    private final DifferentialDriveOdometry odom;

    public Odometry(Drivetrain drive)
    {
        KINE = new DifferentialDriveKinematics(Drivetrain.kTrackWidth);
        this.drive = drive;
        odom = new DifferentialDriveOdometry(drive.m_gyro.getRotation2d(), , 0D);
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

    public double expose_left_encoder_dist()
    {
        return drive.m_leftEncoder.getDistance();
    }

    public double expose_right_encoder_dist()
    {
        return drive.m_rightEncoder.getDistance();
    }

    @Override public void periodic()
    {
        odom.update(drive.m_gyro.getRotation2d(), drive.m_leftEncoder.getDistance(), drive.m_RightEncoder.getDistance());
    }
}