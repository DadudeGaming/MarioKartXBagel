// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmMoveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private DoubleSupplier changeDoubleSupplier;

  private double change;
  /**
   * Creates a new ExampleCommand.
   * 
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmMoveCommand(ArmSubsystem subsystem, double input) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    change = input;
    if(change > 0)
      m_subsystem.up = true;
    else if(change < 0)
      m_subsystem.down = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if((change < 0.0 && m_subsystem.angle > 0.0) || (change > 0.0 && m_subsystem.angle < 90.0))
    //   m_subsystem.changeAngle(change);
    //   System.out.println(change);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    // if((!m_subsystem.up || !m_subsystem.down) && ((change < 0.0 && m_subsystem.angle > 0.0) || (change > 0.0 && m_subsystem.angle < 90.0)))
    //   m_subsystem.changeAngle(change);
    //   System.out.println(change);
    if((change < 0.0 && m_subsystem.angle > 0.0) || (change > 0.0 && m_subsystem.angle < 90.0))
      m_subsystem.changeAngle(change);
      System.out.println(change);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
