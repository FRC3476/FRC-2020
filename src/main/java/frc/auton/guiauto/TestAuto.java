package frc.auton.guiauto;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class TestAuto extends AbstractGuiAuto {

    public TestAuto() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/auto/test.json"));
    
    }
    
}
