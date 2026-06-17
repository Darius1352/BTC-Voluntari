package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.ElapsedTime

class RaceCommand(val firstCommand: Command, val secondCommand: Command) : Command {
    override fun run(packet: TelemetryPacket): Boolean {
        return firstCommand.run(packet) || secondCommand.run(packet)
    }
}