// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // ---------- Robot Subsystems ---------- \\
  private final Drivetrain drive = new Drivetrain();
  private final Elevator elevatorController = new Elevator();
  private final Claw clawController = new Claw();
  
  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  // Create a Sendable Chooser, which allows us to select between Commands (in
  // this case, auto commands)
  private final SendableChooser<Command> chooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxDriver = new XboxController(3);
    this.xboxOperator = new XboxController(2);

    // Controler Throttle Mappings
    this.drive.setDefaultCommand(
        new Drive(drive, xboxDriver));
    
    this.elevatorController.setDefaultCommand(
        new ElevatorController(elevatorController, xboxOperator)); // added, works
    
    this.clawController.setDefaultCommand(
        new ClawController(clawController, xboxOperator)); // added, untested
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command chosen = chooser.getSelected();
    return chosen;
  }
}
