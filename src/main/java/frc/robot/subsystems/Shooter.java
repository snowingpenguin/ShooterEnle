package frc.robot.subsystems;
import java.security.spec.EncodedKeySpec;
import javax.sound.sampled.AudioFormat.Encoding;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.Constants.ShooterConstants;
public class Shooter {
            private static final double setpoint = 0;
                private final TalonFX m_flywheelLead = new TalonFX(ShooterConstants.flywheelL);
                private final TalonFX m_flywheelFollwer = new TalonFX(ShooterConstants.flywheelF);
                private final TalonFX m_pivot = new TalonFX(ShooterConstants.pivot);
                final DutyCycleOut m_request = new DutyCycleOut(0);
                final VoltageOut m_volt = new VoltageOut(0);
            
                private void motorcontrols(){
                    m_flywheelLead.setControl(m_volt.withOutput(12.0));
                }
                private void follower(){
                        m_flywheelFollwer.setControl(new Follower(m_flywheelLead.getDeviceID(), MotorAlignmentValue.Aligned));
                        m_flywheelLead.setControl(m_request.withOutput(1.0));
                        m_request.UpdateFreqHz = 50;
                }
                
                private void PID(){

                    var configs = new TalonFXConfiguration();
                    configs.Slot0.kP = 2.4;
                    configs.Slot0.kI = 0;
                    configs.Slot0.kD = 0.1;
                    m_pivot.getConfigurator().apply(configs);
                    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
            
                }
                private final Encoder m_leftEncoder =
                    new Encoder(
                    ShooterConstants.kLeftEncoderPorts[0],
                    ShooterConstants.kLeftEncoderPorts[1],
                    ShooterConstants.kLeftEncoderReversed);

                private void BangBang(){
                    BangBangController controller = new BangBangController();
                  m_flywheelLead.set(controller.calculate(m_leftEncoder.getRate(), setpoint));   
                  BangBangController m_shooterFeedback;
                 m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
                Encoder m_shooterEncoder;
                    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
                setDefaultCommand(
                    runOnce(
                () -> {
                  m_shooterMotor.disable();
                })
                .andThen(run(() -> {}))
                .withName("Idle")); 
    }
    
    
        
    public Command shootCommand(double setpointRotationsPerSecond) {
        return parallel(
                run(
                    () -> {
                      m_shooterMotor.set(
                          m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                              + m_shooterFeedback.calculate(
                                  m_shooterEncoder.getRate(), setpointRotationsPerSecond));
                    }));
        
                }
    
                
            
}

