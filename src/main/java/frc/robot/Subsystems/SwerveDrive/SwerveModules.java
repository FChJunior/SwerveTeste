package frc.robot.Subsystems.SwerveDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

public class SwerveModules {

    private CANSparkMax tractionMotor1;
    private CANSparkMax tractionMotor2;
    private CANSparkMax tractionMotor3;
    private CANSparkMax tractionMotor4;

    private CANSparkMax angulationMotor1;
    private CANSparkMax angulationMotor2;
    private CANSparkMax angulationMotor3;
    private CANSparkMax angulationMotor4;

    double speedTrMotor1 = 0;
    double speedTrMotor2 = 0;
    double speedTrMotor3 = 0;
    double speedTrMotor4 = 0;

    double posAnMotor1 = 0;
    double posAnMotor2 = 0;
    double posAnMotor3 = 0;
    double posAnMotor4 = 0;

    private XboxController player;

    private Pigeon2 pigeon;

    double ref_encoder = 16.84;
    double ref_PID_P = 0.3;
    double ref_Wrapping = 0.3;

    boolean var1, var2;

    boolean Analog_Simulated(int x, int y, double deadZone) {
        return (player.getRawAxis(x) <= -deadZone || player.getRawAxis(x) >= deadZone
                || player.getRawAxis(y) <= -deadZone || player.getRawAxis(y) >= deadZone);
    }

    void Spark_ConfigPID_Traction(CANSparkMax motor) {
        motor.getPIDController().setP(1);
        motor.getPIDController().setI(0);
        motor.getPIDController().setD(0);
        motor.getPIDController().setOutputRange(-1, 1);
        motor.setInverted(true);
    }

    void Spark_ConfigPID_Angulation(CANSparkMax motor, double wrapping) {
        motor.getPIDController().setP(0.05); // Ajuste este valor conforme necessário
        motor.getPIDController().setI(0.0); // Normalmente 0 para começar
        motor.getPIDController().setD(0.0); // Ajuste para controlar oscilações
        motor.getPIDController().setOutputRange(-wrapping, wrapping);
        motor.getPIDController().setPositionPIDWrappingEnabled(true);
        motor.getPIDController().setPositionPIDWrappingMaxInput(ref_encoder); // Certifique-se do limite correto
        motor.getPIDController().setPositionPIDWrappingMinInput(0);
        //motor.getEncoder().setPositionConversionFactor(ref_encoder);
    }

    void Spark_Config_P(CANSparkMax motor1, CANSparkMax motor2, CANSparkMax motor3, CANSparkMax motor4, double p) {
        motor1.getPIDController().setP(p);
        motor2.getPIDController().setP(p);
        motor3.getPIDController().setP(p);
        motor4.getPIDController().setP(p);
    }

    public SwerveModules() {
        player = new XboxController(0);
        pigeon = new Pigeon2(10);

        tractionMotor1 = new CANSparkMax(1, MotorType.kBrushless);
        angulationMotor1 = new CANSparkMax(2, MotorType.kBrushless);
        angulationMotor1.setInverted(true);

        tractionMotor2 = new CANSparkMax(3, MotorType.kBrushless);
        angulationMotor2 = new CANSparkMax(4, MotorType.kBrushless);
        angulationMotor2.setInverted(true);

        tractionMotor3 = new CANSparkMax(5, MotorType.kBrushless);
        angulationMotor3 = new CANSparkMax(6, MotorType.kBrushless);
        angulationMotor3.setInverted(true);

        tractionMotor4 = new CANSparkMax(7, MotorType.kBrushless);
        angulationMotor4 = new CANSparkMax(8, MotorType.kBrushless);
        angulationMotor4.setInverted(true);

        tractionMotor1.restoreFactoryDefaults();
        tractionMotor2.restoreFactoryDefaults();
        tractionMotor3.restoreFactoryDefaults();
        tractionMotor4.restoreFactoryDefaults();

        angulationMotor1.restoreFactoryDefaults();
        angulationMotor2.restoreFactoryDefaults();
        angulationMotor3.restoreFactoryDefaults();
        angulationMotor4.restoreFactoryDefaults();

        Spark_ConfigPID_Traction(tractionMotor1);
        Spark_ConfigPID_Traction(tractionMotor2);
        Spark_ConfigPID_Traction(tractionMotor3);
        Spark_ConfigPID_Traction(tractionMotor4);

        Spark_ConfigPID_Angulation(angulationMotor1, ref_Wrapping);
        Spark_ConfigPID_Angulation(angulationMotor2, ref_Wrapping);
        Spark_ConfigPID_Angulation(angulationMotor3, ref_Wrapping);
        Spark_ConfigPID_Angulation(angulationMotor4, ref_Wrapping);

        tractionMotor1.getEncoder().setPosition(0);
        tractionMotor2.getEncoder().setPosition(0);
        tractionMotor3.getEncoder().setPosition(0);
        tractionMotor4.getEncoder().setPosition(0);

        angulationMotor1.getEncoder().setPosition(0);
        angulationMotor2.getEncoder().setPosition(0);
        angulationMotor3.getEncoder().setPosition(0);
        angulationMotor4.getEncoder().setPosition(0);

        pigeon.setYaw(0);

    }

    public void SwerveDrivingInit() {
        var1 = false;
        var2 = false;

        tractionMotor1.setIdleMode(IdleMode.kBrake);
        tractionMotor2.setIdleMode(IdleMode.kBrake);
        tractionMotor3.setIdleMode(IdleMode.kBrake);
        tractionMotor4.setIdleMode(IdleMode.kBrake);
        angulationMotor1.setIdleMode(IdleMode.kBrake);
        angulationMotor2.setIdleMode(IdleMode.kBrake);
        angulationMotor3.setIdleMode(IdleMode.kBrake);
        angulationMotor4.setIdleMode(IdleMode.kBrake);

        angulationMotor1.getEncoder().setPosition(0);
        angulationMotor2.getEncoder().setPosition(0);
        angulationMotor3.getEncoder().setPosition(0);
        angulationMotor4.getEncoder().setPosition(0);

        pigeon.setYaw(0);

    }

    public void SwerveDriving() {
        double speed = player.getRawAxis(3) / 4;

        double leftJoystick_DEG = Math.toDegrees(Math.atan2(player.getLeftX(), -player.getLeftY()));
        double rightJoystick_DEG = Math.toDegrees(Math.atan2(player.getRightX(), -player.getRightY()));

        double yaw = -pigeon.getYaw().getValueAsDouble();
        double absYaw = yaw % 360;

        if (absYaw < 0)
            absYaw += 360;

        if (Analog_Simulated(4, 5, 0.15) && !Analog_Simulated(0, 1, 0.15)) {
            if (!var1) {
                var1 = true;

                angulationMotor1.getPIDController().setP(ref_PID_P);
                angulationMotor2.getPIDController().setP(ref_PID_P);
                angulationMotor3.getPIDController().setP(ref_PID_P);
                angulationMotor4.getPIDController().setP(ref_PID_P);

                angulationMotor1.getPIDController().setReference(45 / ref_encoder, ControlType.kPosition);
                angulationMotor2.getPIDController().setReference(315 / ref_encoder, ControlType.kPosition);
                angulationMotor3.getPIDController().setReference(115 / ref_encoder, ControlType.kPosition);
                angulationMotor4.getPIDController().setReference(225 / ref_encoder, ControlType.kPosition);
            }

            speed = player.getRawAxis(4) / 4;

            speedTrMotor1 = speed;
            speedTrMotor2 = speed;
            speedTrMotor3 = speed;
            speedTrMotor4 = speed;

        } else
            var1 = false;

        if (!var1) {
            if (Analog_Simulated(0, 1, 0.15)) {
                if (!var2) {
                    var2 = true;
                    Spark_Config_P(angulationMotor1, angulationMotor2, angulationMotor3, angulationMotor4, ref_PID_P);
                }

                posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;

                if (player.getRightX() > 0.15 || player.getRightX() < -0.15) {

                    if ((rightJoystick_DEG >= 135 || rightJoystick_DEG < -135)
                            && ((leftJoystick_DEG >= 135 || leftJoystick_DEG < -135))) {
                        if (absYaw >= 315 || absYaw < 45) {
                            System.err.println("POS1");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 45 || absYaw < 135) {
                            System.err.println("POS2");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 135 || absYaw < 225) {
                            System.err.println("POS3");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 225 || absYaw < 315) {
                            System.err.println("POS4");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        }
                    } else if ((rightJoystick_DEG >= -135 || rightJoystick_DEG < -45)
                            && ((leftJoystick_DEG >= -135 || leftJoystick_DEG < -45))) {
                        if (absYaw >= 315 || absYaw < 45) {
                            System.err.println("POS5");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 45 || absYaw < 135) {
                            System.err.println("POS6");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 135 || absYaw < 225) {
                            System.err.println("POS7");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 225 || absYaw < 315) {
                            System.err.println("POS8");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        }
                    } else if ((rightJoystick_DEG >= 45 || rightJoystick_DEG < 135)
                            && ((leftJoystick_DEG >= 45 || leftJoystick_DEG < 135))) {
                        if (absYaw >= 315 || absYaw < 45) {
                            System.err.println("POS9");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 45 || absYaw < 135) {
                            System.err.println("POS10");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 135 || absYaw < 225) {
                            System.err.println("POS11");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 225 || absYaw < 315) {
                            System.err.println("POS12");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        }
                    } else if ((rightJoystick_DEG >= -45 || rightJoystick_DEG < 45)
                            && ((leftJoystick_DEG >= -45 || leftJoystick_DEG < 45))) {
                        if (absYaw >= 315 || absYaw < 45) {
                            System.err.println("POS13");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 45 || absYaw < 135) {
                            System.err.println("POS14");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 135 || absYaw < 225) {
                            System.err.println("POS15");
                            posAnMotor1 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + rightJoystick_DEG) / ref_encoder;
                        } else if (absYaw >= 225 || absYaw < 315) {
                            System.err.println("POS16");
                            posAnMotor1 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor2 = (absYaw + leftJoystick_DEG) / ref_encoder;
                            posAnMotor3 = (absYaw + rightJoystick_DEG) / ref_encoder;
                            posAnMotor4 = (absYaw + leftJoystick_DEG) / ref_encoder;
                        }
                    }
                }

                angulationMotor1.getPIDController().setReference(posAnMotor1, ControlType.kPosition);
                angulationMotor2.getPIDController().setReference(posAnMotor2, ControlType.kPosition);
                angulationMotor3.getPIDController().setReference(posAnMotor3, ControlType.kPosition);
                angulationMotor4.getPIDController().setReference(posAnMotor4, ControlType.kPosition);

                speedTrMotor1 = speed;
                speedTrMotor2 = speed;
                speedTrMotor3 = speed;
                speedTrMotor4 = speed;
            } else {
                if (!player.getAButton()) {
                    Spark_Config_P(angulationMotor1, angulationMotor2, angulationMotor3, angulationMotor4, 0);
                }
                var2 = false;
            }

            if (player.getAButton()) {
                Spark_Config_P(angulationMotor1, angulationMotor2, angulationMotor3, angulationMotor4, ref_PID_P);
                angulationMotor1.getPIDController().setReference(0, ControlType.kPosition);
                angulationMotor2.getPIDController().setReference(0, ControlType.kPosition);
                angulationMotor3.getPIDController().setReference(0, ControlType.kPosition);
                angulationMotor4.getPIDController().setReference(0, ControlType.kPosition);
            }

            if (!Analog_Simulated(0, 1, 0.15) && !Analog_Simulated(4, 5, 0.15) && player.getRawAxis(3) < 0) {
                tractionMotor1.set(0);
                tractionMotor2.set(0);
                tractionMotor3.set(0);
                tractionMotor4.set(0);
            }
            tractionMotor1.set(speedTrMotor1);
            tractionMotor2.set(speedTrMotor2);
            tractionMotor3.set(speedTrMotor3);
            tractionMotor4.set(speedTrMotor4);

            SmartDashboard.putNumber("Pigeon YAW", yaw);
            SmartDashboard.putNumber("Pigeon YAW ABS", absYaw);
            SmartDashboard.putNumber("Joystick LEFT", leftJoystick_DEG);
            SmartDashboard.putNumber("Joystick RIGHT", rightJoystick_DEG);

            SmartDashboard.putNumber("Encoder Motor 1", angulationMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("Encoder Motor 2", angulationMotor2.getEncoder().getPosition());
            SmartDashboard.putNumber("Encoder Motor 3", angulationMotor3.getEncoder().getPosition());
            SmartDashboard.putNumber("Encoder Motor 4", angulationMotor4.getEncoder().getPosition());
            SmartDashboard.putNumber("Speed", speed);

        }

    }

    public void SwerveDisable() {
        tractionMotor1.setIdleMode(IdleMode.kCoast);
        tractionMotor2.setIdleMode(IdleMode.kCoast);
        tractionMotor3.setIdleMode(IdleMode.kCoast);
        tractionMotor4.setIdleMode(IdleMode.kCoast);
        angulationMotor1.setIdleMode(IdleMode.kCoast);
        angulationMotor2.setIdleMode(IdleMode.kCoast);
        angulationMotor3.setIdleMode(IdleMode.kCoast);
        angulationMotor4.setIdleMode(IdleMode.kCoast);
    }
}
