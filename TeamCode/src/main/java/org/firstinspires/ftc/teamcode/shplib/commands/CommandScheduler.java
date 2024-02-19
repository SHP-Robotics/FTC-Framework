package org.firstinspires.ftc.teamcode.shplib.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public final class CommandScheduler {
    private static CommandScheduler instance;

    private final ArrayList<Command> commands = new ArrayList<>();
    private final ArrayList<Class<?>> removalQueue = new ArrayList<>();
    private final ArrayList<Subsystem> subsystems = new ArrayList<>();

    private Telemetry telemetry;

    public static CommandScheduler getInstance() {
        if (instance == null) instance = new CommandScheduler();
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.clearAll();
        this.telemetry = telemetry;
    }

    public void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public boolean containsInstance(Class<?> commandType) {
        for (Command command: commands) {
            if (command.getClass() == commandType) {
                return true;
            }
        }
        return false;
    }

    public void removeInstances(Class<?> commandType) {
        if (containsInstance(commandType) && !removalQueue.contains(commandType)) {
            removalQueue.add(commandType);
        }
    }

    public void scheduleCommand(Command command) {
        // if scheduled command is a command group, toss the object and schedule everything
        if (command instanceof SequentialCommandGroup ||
                command instanceof ParallelCommandGroup) {
            scheduleWithCommands(command);
            scheduleNextCommands(command);
            return;
        }
        // make sure that duplicate commands aren't scheduled
        for (Command c : commands) {
            if (c.getClass().equals(command.getClass())
                    && !(c instanceof RunCommand)
                    && !(c instanceof WaitCommand))
                return;
        }
        command.init();
        commands.add(command);
        scheduleWithCommands(command);
    }

    private void scheduleNextCommands(Command command) {
        ArrayList<Command> nextCommands = command.getNextCommands();
        if (nextCommands.size() == 0) return;
        Command nextCommand = nextCommands.remove(0);
        nextCommand.then(nextCommands);
        scheduleCommand(nextCommand);
    }

    private void scheduleWithCommands(Command command) {
        for (Command withCommand : command.getWithCommands())
            scheduleCommand(withCommand);
    }

    // probably not the most efficient way of scheduling, but works pretty well for what we need
    public void run() throws InterruptedException {
        ArrayList<Subsystem> idleSubsystems = new ArrayList<>(subsystems);

        // run subsystem periodic
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic(telemetry);
        }

        for (int i = 0; i < commands.size(); i++) {
            if (removalQueue.contains(commands.get(i).getClass())) {
                removalQueue.remove(commands.get(i).getClass());
                commands.remove(i);
                continue;
            }

            Command command = commands.get(i);

            // check if command subsystems are idle
            Subsystem[] commandSubsystems = command.getSubsystems();
            boolean canRun = true;
            for (Subsystem subsystem : commandSubsystems) {
                // if subsystem is not idle, it cannot execute
                if (!idleSubsystems.contains(subsystem)) {
                    canRun = false;
                    break;
                }
            }
            if (!canRun) continue;

            // after ensuring command subsystems are idle, make them no longer idle
            for (Subsystem subsystem : commandSubsystems) {
                idleSubsystems.remove(subsystem);
            }

            command.execute();

            if (command.isFinished()) {
                command.end();
                commands.remove(i);
                i--;
                scheduleNextCommands(command);
            }
        }

        // for any idle subsystems, run their default command
        for (Subsystem subsystem : idleSubsystems) {
            Command defaultCommand = subsystem.getDefaultCommand();
            if (defaultCommand != null) defaultCommand.execute();
        }
    }
}
