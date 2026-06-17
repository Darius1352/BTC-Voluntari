package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class RunUntilTimeCommand(val boolean: Boolean) : Command {
    override fun run(packet: TelemetryPacket): Boolean {
        return if (boolean) {
            true
        } else {
            false
        }
    }

    fun interface BooleanFunction {
        fun run(): Boolean
    }
}