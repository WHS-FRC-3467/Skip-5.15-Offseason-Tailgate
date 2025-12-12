// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggerHelper;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonSRX top;
    private final TalonSRX bottom;

    public IndexerSubsystem(TalonSRX top, TalonSRX bottom)
    {
        this.top = top;
        this.bottom = bottom;
    }

    public Command run()
    {
        return this.runOnce(() -> top.set(ControlMode.PercentOutput, 0.5));
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(IndexerSubsystemConstants.NAME, this);
    }

    public void close()
    {
        top.DestroyObject();
        bottom.DestroyObject();
    }
}
