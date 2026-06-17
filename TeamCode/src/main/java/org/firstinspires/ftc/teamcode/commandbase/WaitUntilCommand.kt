package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class WaitUntilCommand(val booleanFunction: BooleanFunction) : Command {
    override fun run(packet: TelemetryPacket): Boolean {
        return booleanFunction.run()
    }

    fun interface BooleanFunction {
        fun run(): Boolean
    }
}