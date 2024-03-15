package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ClimberConsts;

public class Climbers extends SafeSubsystem {
    private static final CANSparkMax m_LeftClimberMotor = 
        new CANSparkMax(ClimberConsts.kLeftClimberID, MotorType.kBrushless);

    @Override
    public void log() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
}
