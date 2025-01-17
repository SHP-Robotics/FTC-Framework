package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.commands.LynxDatagram;
import com.qualcomm.hardware.lynx.commands.core.LynxDekaInterfaceCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.nio.ByteBuffer;

/**
 * Created by bob on 2016-03-07.
 */
public class UnhingedSetMotorTargetVelocityCommand extends LynxDekaInterfaceCommand<LynxAck>
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public final static int cbPayload = 3;
//    public final static int apiVelocityFirst = -32767;
//    public final static int apiVelocityLast  =  32767;

    private byte motor;
    private short velocity;    // in encoder ticks per second / sec, +CW, -CCW

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public UnhingedSetMotorTargetVelocityCommand(LynxModuleIntf module)
    {
        super(module);
    }

    public UnhingedSetMotorTargetVelocityCommand(LynxModuleIntf module, int motorZ, int velocity)
    {
        this(module);
        LynxConstants.validateMotorZ(motorZ);
//        if (velocity < apiVelocityFirst || velocity > apiVelocityLast) throw new IllegalArgumentException(String.format("illegal velocity: %d", velocity));
        this.motor = (byte)motorZ;
        this.velocity = (short)velocity;
    }

    //----------------------------------------------------------------------------------------------
    // Operations
    //----------------------------------------------------------------------------------------------

    @Override
    public byte[] toPayloadByteArray()
    {
        ByteBuffer buffer = ByteBuffer.allocate(cbPayload).order(LynxDatagram.LYNX_ENDIAN);
        buffer.put(this.motor);
        buffer.putShort(this.velocity);
        return buffer.array();
    }

    @Override
    public void fromPayloadByteArray(byte[] rgb)
    {
        ByteBuffer buffer = ByteBuffer.wrap(rgb).order(LynxDatagram.LYNX_ENDIAN);
        this.motor = buffer.get();
        this.velocity = buffer.getShort();
    }

    @Override
    public boolean isDangerous()
    {
        return velocity != 0;
    }
}
