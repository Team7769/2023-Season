package frc.robot.Lib;

import edu.wpi.first.wpilibj.DigitalInput;

public class Photoeye extends DigitalInput {
    
    public Photoeye(int port)
    {
        super(port);
    }

    public boolean isBlocked()
    {
        return !this.get();
    }
}
