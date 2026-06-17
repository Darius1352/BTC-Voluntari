package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class InstantCommand(val lambdaCommand: LambdaCommand): Command {
    override fun run(packet: TelemetryPacket): Boolean {
        lambdaCommand.run()
        return true
    }

    fun interface LambdaCommand{
        fun run()
    }
}