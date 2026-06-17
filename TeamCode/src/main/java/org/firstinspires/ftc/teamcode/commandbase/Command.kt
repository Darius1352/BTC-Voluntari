package org.firstinspires.ftc.teamcode.commandbase

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

interface Command {
    fun run(packet: TelemetryPacket): Boolean
}