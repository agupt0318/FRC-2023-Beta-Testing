// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorPID;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorPIDController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorPID m_elevatorPID;
  private final XboxController e_controller; // e_controller is elevator's controller
  private double desiredHeight;
  private double nextHeight;
  private boolean lastToggle;
  
  private final double maxHeight = 1.0; // not actual value
  private final double minHeight = 0.0;
  
  // we don't have limit switches atm
//   DigitalInput toplimitSwitch = new DigitalInput(2);
//   DigitalInput bottomlimitSwitch = new DigitalInput(1);

  public ElevatorPIDController(ElevatorPID elevatorPIDSubsystem, XboxController controller) {
    e_controller = controller;
    m_elevatorPID = elevatorPIDSubsystem;
    desiredHeight = 0;
    nextHeight = 0;
    lastAToggle = false;
    lastYToggle = false;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if Y button is pushed, move up 0.1 meters
    if (e_controller.getYButton().getAsBoolean() && !lastYToggle) {
      lastYToggle = true;
      nextHeight = desiredHeight + 0.1;
      if (nextHeight < maxHeight)
        desiredHeight += 0.1;
    } 
    
    // if A button is pushed, move down 0.1 meters
    if (e_controller.getAButton().getAsBoolean() && !lastAToggle) {
      lastAToggle = true;
      nextHeight = desiredHeight - 0.1;
      if (nextHeight > minHeight)
        desiredHeight -= 0.1;
    } 

    if (!e_controller.getYButton().getAsBoolean()) {
      lastYToggle = false;
    }
    
    if (!e_controller.getAButton().getAsBoolean()) {
      lastAToggle = false;
    }
    
    m_elevatorPID.setSetpoint(desiredHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorPID.m_controller.atGoal();
  }
}
