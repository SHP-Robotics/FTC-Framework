### Welcome to the tutorial file!

Here are some important notes to make navigating easier:

Ctrl/Cmd + B - This will navigate you to the declaration of the function or class. 
    For example: 
        DriveSubsystem drive = new DriveSubsystem(hardwaremap);
        If you put your cursor on "DriveSubsystem" and call Ctrl+B, it will navigate you to the 
        DriveSubsystem class. This is particularly helpful if you can't find something.

Okay breaking it down:

### There are FOUR important things you need to know:
    1. TeleOp - This is where all of your working code will go
    2/3. Basebot/Constants - This is where you will keep track of most of your variables and systems
    4. Subsystems - These will separate components of your robot and organize it.

### TeleOp

Navigate to the ExampleTeleOp file.
    TeamCode > java > org.firstinspires.ftc.teamcode > telops > ExampleTeleOp

This file will run all of your code for the robot. 
There are 3 main parts to a teleop.
    1. init() - This is where you put all the code you want to happen when you initialize the robot.
        Generally, leave the default command to drive and don't add anything
    2. start() - When you start the robot, you can have anything you want to run once in here.
        It's not commonly used.
    3. loop() - This will run the entire time. Most of your code will go in here. 

### Subsystems
For ease of coding and decomposition (very important coding practice), we split things into subsytems.
Any major robot system gets its own subsystem. Default subsytems are normally DriveSubsystem and
VisionSubsystem. This example also has an ArmSubsystem. 

TemplateSubsystem is your friend. 

