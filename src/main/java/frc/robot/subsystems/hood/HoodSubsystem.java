// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.rotary.RotaryMechanism;
import frc.lib.util.LoggerHelper;

public class HoodSubsystem extends SubsystemBase {

    private final RotaryMechanism io;
    public final Trigger stationaryTrigger;


    public HoodSubsystem(RotaryMechanism io)
    {
        this.io = io;
        stationaryTrigger = new Trigger(() -> (io.getVelocity().lt(RotationsPerSecond.of(0.01))));
    }

    @Override
    public void periodic()
    {
        LoggerHelper.recordCurrentCommand(HoodSubsystemConstants.NAME, this);
        io.periodic();
    }


    public boolean nearGoal(Angle targetPosition)
    {
        return io.nearGoal(targetPosition, HoodSubsystemConstants.TOLERANCE);
    }

    public Command move(Angle position)
    {
        return Commands.sequence(
            this.runOnce(() -> io.runPosition(
                position,
                HoodSubsystemConstants.CRUISE_VELOCITY,
                HoodSubsystemConstants.ACCELERATION,
                HoodSubsystemConstants.JERK,
                PIDSlot.SLOT_0)),
            Commands.waitUntil(() -> nearGoal(position)));
    }

    public Command jog(double magnitude)
    {

        return this.runOnce(() -> io.runVelocity(
            Units.RadiansPerSecond.of(magnitude),
            Units.RadiansPerSecond.of(magnitude).per(Second),
            PIDSlot.SLOT_0)).withName("jog");
    }

    public Command stop()
    {
        return runOnce(() -> io.runBrake()).withName("Stop");
    }

    public Command homeZero()
    {
        return Commands.sequence(
            this.runOnce(() -> io.runVoltage(Volts.of(3))),
            Commands.waitUntil(stationaryTrigger),
            this.runOnce(io::runBrake),
            this.runOnce(() -> io.setEncoderPosition(Rotations.zero())));
    }

    public AngularVelocity getVelocity()
    {
        return io.getVelocity();
    }

    public void close()
    {
        io.close();
    }
}
