package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class SetHeight extends InstantCommand {
    private Elevator m_elevator;
    private double m_height;

    public SetHeight (double height, Elevator elevator) {
        m_elevator = elevator;
        m_height = height;
    }

    public void initialize() {
        m_elevator.EnablePID();
        m_elevator.setheight(m_height);
    }
}
