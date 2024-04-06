package frc.examples.templates.manipulators.arm.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.examples.templates.manipulators.arm.Constants.*;

public class ArmSBS extends SubsystemBase{

    private final CANSparkMax left;
    private final CANSparkMax right; // assumed two motors are used for the arm subsystem

    private final RelativeEncoder encoder; // change relative with absolute encoder if absolute is used
    private final SparkPIDController armPID;

    private final ArmFeedforward armFeedforward =
        new ArmFeedforward(
        ArmConstants.kSVolts, ArmConstants.kGVolts,
        ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    public ArmSBS (int leftID, int rightID) {
        left = new CANSparkMax(leftID, MotorType.kBrushless);
        right = new CANSparkMax(rightID, MotorType.kBrushless);
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setSmartCurrentLimit(30);
        right.setSmartCurrentLimit(30);
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        left.setInverted(false);
        right.setInverted(true);

        encoder = left.getEncoder();
        armPID = left.getPIDController();
        armPID.setFeedbackDevice(encoder);

        armPID.setP(ArmConstants.kP);
        armPID.setI(ArmConstants.kI);
        armPID.setD(ArmConstants.kD);
        armPID.setIZone(ArmConstants.kIz);
        armPID.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

        left.burnFlash();
        right.burnFlash();
    }

    public void runOpenLoop(double supplier) {
            left.set(supplier);
            right.set(supplier);
    }

  
    public void hold(TrapezoidProfile.State setpoint) {
        double feedforward = armFeedforward.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
        armPID.setReference(setpoint.position, ControlType.kPosition,0, feedforward);
    }

    public void runToPosition(TrapezoidProfile.State setpoint) {
            double feedforward = armFeedforward.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
            armPID.setReference(setpoint.position, ControlType.kPosition, 0, feedforward);
    }

    public double getPos() {
        return encoder.getPosition();
    }

    public double[] getTemp() {
       double result[] = {left.getMotorTemperature(), right.getMotorTemperature()};
       return result;
    }
    
    public double[] getCurrent() {
        double result[] = {left.getOutputCurrent(), right.getOutputCurrent()};
        return result;
    }
}