package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.DriveConstants.S_VOLTS, Constants.DriveConstants.V_VOLT_SECONDS_PER_METER, Constants.DriveConstants.A_VOLT_SECONDS_SQUARED_PER_METER);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withEnableFOC(true);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, new CANBus("1912CANivore"));
        //angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

       // this.angleOffset = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, new CANBus("1912CANivore"));
        TalonFXConfiguration myAngleFXConfig = Robot.ctreConfigs.swerveAngleFXConfig;
        myAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        myAngleFXConfig.Feedback.FeedbackRemoteSensorID = moduleConstants.cancoderID;

        mAngleMotor.getConfigurator().apply(myAngleFXConfig);
        //resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, new CANBus("1912CANivore"));
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    @SuppressWarnings("deprecation")
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()).withEnableFOC(true));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.DriveConstants.MAX_SPEED_METERS_PER_SECOND;
            mDriveMotor.setControl(driveDutyCycle.withEnableFOC(true));
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.ModuleConstants.WHEEL_CIRCRUMFERENCE_METERS);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity.withEnableFOC(true));
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void reset() {
        mAngleMotor.setPosition(0);
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.ModuleConstants.WHEEL_CIRCRUMFERENCE_METERS), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.ModuleConstants.WHEEL_CIRCRUMFERENCE_METERS), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public double getCurrentDrive() {
        return mDriveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getCurrentTurn() {
        return mAngleMotor.getSupplyCurrent().getValueAsDouble();
    }


}