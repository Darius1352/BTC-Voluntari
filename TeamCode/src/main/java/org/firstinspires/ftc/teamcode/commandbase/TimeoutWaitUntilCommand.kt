package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.ElapsedTime

class TimeoutWaitUntilCommand(val timeout: Double, val booleanFunction: BooleanFunction) : Command {
    val timeoutCommand: SleepCommand = SleepCommand(timeout)
    override fun run(packet: TelemetryPacket): Boolean {
        return booleanFunction.run() || timeoutCommand.run(packet)
    }

    fun interface BooleanFunction {
        fun run(): Boolean
    }
}