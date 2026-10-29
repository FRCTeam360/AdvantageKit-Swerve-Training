// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  // Constants
  private static final double ROLLER_VELOCITY_RPM = 2000.0;
  private static final double ROLLER_DUTY_CYCLE = 0.80;
  private static final double PREVENT_JAM_DUTY_CYCLE = -0.04;
  private static final double UNJAMMING_DUTY_CYCLE = -0.95;
  private static final double REVERSING_DUTY_CYCLE = -0.5;

  // State variables
  private IntakeRollerStates wantedState = IntakeRollerStates.IDLE;
  private IntakeRollerStates currentState = IntakeRollerStates.IDLE;
  private IntakeRollerStates previousState = IntakeRollerStates.IDLE;
  private ControlState controlState = ControlState.SUPERSTRUCTURE;

  /** Creates a new IntakeRoller. */
  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  public enum IntakeRollerStates {
    IDLE,
    INTAKING,
    ASSIST_SHOOTING,
    REVERSING,
    JAM_DETECTED,
    UNJAMMING,
    OFF,
    PREVENT_JAM,
  }

  // State machine methods

  public IntakeRollerStates getState() {
    return currentState;
  }

  public void setWantedState(IntakeRollerStates state) {
    wantedState = state;
  }

  public void setControlState(ControlState controlState) {
    this.controlState = controlState;
  }

  private void updateState() {
    previousState = currentState;

    switch (wantedState) {
      case INTAKING:
        currentState = IntakeRollerStates.INTAKING;
        break;
      case PREVENT_JAM:
        currentState = IntakeRollerStates.PREVENT_JAM;
        break;
      case UNJAMMING:
        currentState = IntakeRollerStates.UNJAMMING;
        break;
      case REVERSING:
        currentState = IntakeRollerStates.REVERSING;
        break;
      case OFF:
      default:
        currentState = IntakeRollerStates.OFF;
        break;
    }
  }

  private void applyState() {
    switch (currentState) {
      case INTAKING:
        setVelocity(ROLLER_VELOCITY_RPM);
        break;
      case PREVENT_JAM:
        setDutyCycle(PREVENT_JAM_DUTY_CYCLE);
        break;
      case UNJAMMING:
        setDutyCycle(UNJAMMING_DUTY_CYCLE);
        break;
      case REVERSING:
        setDutyCycle(REVERSING_DUTY_CYCLE);
        break;
      case OFF:
      default:
        stop();
        break;
    }
  }

  // IO delegation methods

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setVelocity(double rpm) {
    io.setVelocity(rpm);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setDutyCycleCommand(double dutyCycle) {
    return runEnd(() -> setDutyCycle(dutyCycle), () -> setDutyCycle(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/IntakeRoller", inputs);

    if (controlState == ControlState.SUPERSTRUCTURE) {
      updateState();
      applyState();
    }
    Logger.recordOutput("Superstructure/Subsystems/IntakeRoller/WantedState", wantedState);
    Logger.recordOutput("Superstructure/Subsystems/IntakeRoller/CurrentState", currentState);
    Logger.recordOutput("Superstructure/Subsystems/IntakeRoller/PreviousState", previousState);
    Logger.recordOutput("Superstructure/Subsystems/IntakeRoller/ControlState", controlState);
  }
}
