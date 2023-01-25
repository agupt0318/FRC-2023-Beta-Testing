// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Arrays;

import com.pathplanner.lib.*;

public final class Autos {
  public static final PathPlannerTrajectory[] AUTO_PATHS = 
  {
    PathPlanner.generatePath(new PathConstraints(4, 5), Arrays.asList(
      new PathPoint[]
      {
        new Translation2d(10, Rotation2d.fromDegrees(300));
      }
    ))
  };
  public static void load(final CharSequence fileName, PathConstraints defaultConstraints)
  {
    
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
