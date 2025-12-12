// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.Ports;

/** Add your docs here. */
public class IndexerSubsystemConstants {
    public static String NAME = "Indexer";

    public static IndexerSubsystem get()
    {
        return switch (Constants.currentMode) {
            case REAL -> new IndexerSubsystem(
                new TalonSRX(Ports.indexerTop.id()),
                new TalonSRX(Ports.indexerBottom.id()));
            case SIM -> new IndexerSubsystem(
                new TalonSRX(Ports.indexerTop.id()),
                new TalonSRX(Ports.indexerBottom.id()));
            case REPLAY -> new IndexerSubsystem(null, null);
        };
    }
}
