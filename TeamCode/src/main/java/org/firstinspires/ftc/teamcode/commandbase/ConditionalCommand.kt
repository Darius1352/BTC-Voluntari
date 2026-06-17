package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class ConditionalCommand(
    private val condition: BooleanFunction,
    private val command: Command
) : Command {
    override fun run(packet: TelemetryPacket): Boolean {
        return if (condition.run()) {
            command.run(packet)
        } else {
            true
        }
    }

    fun interface BooleanFunction {
        fun run(): Boolean
    }
}