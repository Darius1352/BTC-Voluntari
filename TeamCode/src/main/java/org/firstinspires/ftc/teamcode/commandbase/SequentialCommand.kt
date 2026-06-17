package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class SequentialCommand(vararg commands: Command): Command {
    var commandList = commands.asList()

    override fun run(packet: TelemetryPacket): Boolean {
        if(commandList.isEmpty()) return true
        if(commandList.first().run(packet)) {
            commandList = commandList.drop(1)
            run(packet)
        }
        return false
    }

}