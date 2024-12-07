// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Intake;
 
public class ManualArm extends Command {
  /** Creates a new RunIntake. */

  Arm m_arm;
  double m_speed;

  public ManualArm(Arm Arm, double Speed) {
    
    m_arm = Arm;
    m_speed = Speed;

    addRequirements(Arm);
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    m_arm.RunAngleWithLimits(m_speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
    m_arm.RunAngleWithLimits(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
}
