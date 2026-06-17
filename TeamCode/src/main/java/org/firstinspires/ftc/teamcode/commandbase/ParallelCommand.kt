package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class ParallelCommand(vararg commands: Command): Command {
    var commandList = commands.asList()

    override fun run(packet: TelemetryPacket): Boolean {
        commandList = commandList.filter { !it.run(packet) }
        return commandList.isEmpty()
    }
}