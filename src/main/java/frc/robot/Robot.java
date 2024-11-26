// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/* Primeira Versão do Smash 2025
 * Autor: Francisco das Chagas (Mentor)
 * Inspiração: Aurora #9168 - Agrobot
 * 
 * Versão: 0.1
 */

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.SwerveDrive.SwerveModules;


public class Robot extends TimedRobot {

    SwerveModules swerve;

  //#region Robot Metodos
  @Override
  public void robotInit() {
    swerve = new SwerveModules();
  }
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    swerve.SwerveDrivingInit();
  }

  @Override
  public void teleopPeriodic() {
    swerve.SwerveDriving();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {    swerve.SwerveDisable();}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
  //#endregion
}