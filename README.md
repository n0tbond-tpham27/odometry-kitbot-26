# Command-Kitbot-Starter-NoBindings

*Advanced Robotics Course — Command-Based Kitbot Unit.*

## What this is

This is an example kitbot codebase — drivetrain, intake, loader, and flywheel are all implemented and working. The one thing missing is in `RobotContainer.java`: right
now only the default drive command is wired up. Nothing else is connected to a
controller button yet.

That's on purpose. Before getting into the theory behind command-based programming —
what a `Subsystem` actually is, how the scheduler works, all the ways to combine
commands — you're going to open this repo and get something working first. You won't
understand everything you see, and that's expected right now. Your job is narrower:
find out what a piece of code can already do, and use it.

## Repo map

A command-based FRC project is organized the same way every time, once you know what to
look for:

| File / folder | Role |
|---|---|
| `Robot.java` | Almost empty on purpose — mostly just tells the scheduler to run, every cycle. |
| `RobotContainer.java` | The wiring hub — creates each subsystem, connects buttons to commands, hands off the autonomous routine. **This is the file you'll spend most of your time in.** |
| `subsystems/` | Each file owns one piece of hardware and offers a menu of things it can do. |
| `commands/` | Composed or standalone actions, where the logic doesn't belong to just one subsystem. |
| `Constants.java` | Tunable values (motor IDs, current limits, speeds) collected in one place instead of scattered through the code. |

Inside a subsystem file, there are two kinds of things. Near the top: private fields —
the actual hardware, motor controllers, sensors. Below that: public methods — the
things that subsystem can actually be told to do. **That second group is what you're
looking for.**

You may also notice `ExampleSubsystem.java` / `ExampleCommand.java` sitting in
`subsystems/`/`commands/`. Those are leftover template files, not part of the real
robot — safe to ignore for this lesson.

## Meet the kitbot

| Mechanism | What it does |
|---|---|
| Drivetrain | Drives and steers the robot. |
| Intake | Collects game pieces from the floor. |
| Loader | Feeds game pieces from the intake toward the flywheel. |
| Flywheel / Launcher | Spins up and launches game pieces. |

Each mechanism above has its own subsystem file — that's where you'll find out what it
can actually do.

## What you'll do

Right now, pressing buttons on the controller does nothing except drive the robot
around. Your task is to open the subsystem files, read what each one already offers,
and wire some of it up to buttons yourself — then see how the robot's behavior changes
depending on *how* you press a button, not just *which* one.

The full task list lives in the lesson handout, not here — this file is just your map
before you start reading code.

## Before you start

- Fork this repo, clone your fork, and commit as you go — your commit history is part
  of how this gets graded, same as every task in this unit.
- Know where the Driver Station's disable button (Enter) is before you deploy anything.
