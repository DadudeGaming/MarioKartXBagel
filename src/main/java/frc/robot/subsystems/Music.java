package frc.robot.subsystems;

import java.util.ArrayList;

import javax.sound.midi.Instrument;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TelescopeConstants;

public class Music extends SubsystemBase {
    private final TalonFX m_motor1 = new TalonFX(ArmConstants.CANIDs.frontMotorCANID);
    private final TalonFX m_motor2 = new TalonFX(ArmConstants.CANIDs.backMotorCANID);
    private final TalonFX m_motor3 = new TalonFX(IntakeConstants.CANID);
    private final TalonFX m_motor4 = new TalonFX(TelescopeConstants.MotorCANID);

    public Orchestra m_orchestra = new Orchestra();
    public String musicFile = "doom.chrp";

    public Music(SwerveSubsystem swerveSubsystem) {
        // AudioConfigs audioConfigs = new AudioConfigs();
        // audioConfigs.AllowMusicDurDisable = true;

        swervelib.SwerveModule[] motors;
        TalonFX[] motors2;
        motors = swerveSubsystem.getSwerveDrive().getModules();
        motors2 = new TalonFX[] {m_motor1, m_motor2, m_motor3, m_motor4};

        for (int i = 0; i < motors.length; i++) {
            Object motor = motors[i].getDriveMotor().getMotor();
            if (motor instanceof ParentDevice parentDevice) {
                m_orchestra.addInstrument(parentDevice, 0);
                m_orchestra.addInstrument(motors2[i], i+1);
            }
        }
        // Add a single device to the orchestra
        // m_orchestra.addInstrument(m_motor);

        // Attempt to load the chrp
        var status = m_orchestra.loadMusic(musicFile);

        if (!status.isOK()) {
            // log error
        }

        // m_orchestra.play();
    }

    public void loadPlayMusic(String file) {
        // Attempt to load the chrp
        var status = m_orchestra.loadMusic(file);

        if (!status.isOK()) {
            // log error
        }

        m_orchestra.play();
    }
}
