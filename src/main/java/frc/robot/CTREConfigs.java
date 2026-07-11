package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.ModuleConstants.CANCODERINVERT;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.ModuleConstants.ANGLEMOTORINVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.ModuleConstants.kTurnGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ModuleConstants.ANGLEENABLECURRENTLIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.ModuleConstants.ANGLECURRENTLIMIT;
        //swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ModuleConstants.ANGLECURRENTTHRESHOLD;
        //swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.ModuleConstants.ANGLECURRENTTHRESHOLDTIME;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.ModuleConstants.kPModuleTurningController;
        swerveAngleFXConfig.Slot0.kI = Constants.ModuleConstants.kIModuleTurningController;
        swerveAngleFXConfig.Slot0.kD = Constants.ModuleConstants.kDModuleTurningController;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.ModuleConstants.DRIVEMOTORINVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.ModuleConstants.WHEEL_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ModuleConstants.DRIVEENABLECURRENTLIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.ModuleConstants.DRIVECURRENTLIMIT;
        //swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ModuleConstants.DRIVECURRENTTHRESHOLD;
        //swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.ModuleConstants.DRIVECURRENTTHRESHOLDTIME;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.ModuleConstants.kPModuleDriveController;
        swerveDriveFXConfig.Slot0.kI = Constants.ModuleConstants.kIModuleDriveController;
        swerveDriveFXConfig.Slot0.kD = Constants.ModuleConstants.kDModuleDriveController;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.ModuleConstants.OPENLOOPRAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.ModuleConstants.OPENLOOPRAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.ModuleConstants.CLOSEDLOOPRAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.ModuleConstants.CLOSEDLOOPRAMP;
    }
}