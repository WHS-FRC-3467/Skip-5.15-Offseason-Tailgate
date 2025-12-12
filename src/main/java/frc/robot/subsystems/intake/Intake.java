/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.motor.MotorIO.PIDSlot;
import frc.lib.mechanisms.flywheel.FlywheelMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Intake extends SubsystemBase {
    private final FlywheelMechanism io;

    @RequiredArgsConstructor
    @SuppressWarnings("Immutable")
    @Getter
    public enum State {
        STAY(Units.RadiansPerSecond.of(0.0)),
        SUCK(Units.RadiansPerSecond.of(2 * Math.PI)),
        UNSUCK(Units.RadiansPerSecond.of(2 * -Math.PI));

        private final AngularVelocity state;

    }

    Intake(FlywheelMechanism io)
    {
        this.io = io;
        setState(State.STAY).ignoringDisable(true).schedule();

    }

    public Command setState(State state)
    {
        return this.runOnce(
            () -> io.runVelocity(state.getState(), state.state.per(Second), PIDSlot.SLOT_0));


    }
}
