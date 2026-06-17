package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.ElapsedTime

class RunUntilCommand(val runnableCommand: Command, val untilCommand: Command) : Command {
    override fun run(packet: TelemetryPacket): Boolean {
        runnableCommand.run(packet)
        return untilCommand.run(packet)
    }
}