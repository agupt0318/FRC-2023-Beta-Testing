// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FourBarArm m_armSubsystem;
  private final XboxController controller; // controller is arm's controller
  // private final DigitalInput toplimitSwitch;
  // private final DigitalInput bottomlimitSwitch;

  // Creates a new ExampleCommand.
  public FourBarArmController(FourBarArm armSubsystem, XboxController controller) {
    this.controller = controller;
    m_armSubsystem = armSubsystem;
    // toplimitSwitch = new DigitalInput(2);
    // bottomlimitSwitch = new DigitalInput(1);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double victorInputPercent = controller.getLeftY(); // not sure which joystick yet
    double sparkInputPercent = controller.getRightY(); // not sure which joystick
    m_armSubsystem.setPercentOutputVictor(victorInputPercent); 
    m_armSubsystem.setSpeedSpark(sparkInputPercent); // takes percent. trust
    
    /*
    System.out.println(bottomlimitSwitch.get());

    if (inputPercent > 0) {
        if (toplimitSwitch.get()) {
            // We are going up and top limit is tripped so stop
             m_elevatorSubsystem.setPercentOutput(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
             m_elevatorSubsystem.setPercentOutput(inputPercent);
        }
    } else {
        if (!bottomlimitSwitch.get()) {
            // We are going down and bottom limit is tripped so stop
             m_elevatorSubsystem.setPercentOutput(0);
        } else {
            // We are going down but bottom limit is not tripped so go at commanded speed
            m_elevatorSubsystem.setPercentOutput(inputPercent);
        }
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
