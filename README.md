# 2021-Infinite-Recharge
This repository is for code for the 2021 robot for the Infinite Recharge game.  It also contains some custom vision code that we wrote as an alternative to the limelight we were using last year.  This file should contain the steps necessary to get up and running with this code on your computer.

## FIRST Game Tools
Before you do anything else, make sure that you have the FRC Game Tools and WPILib installed on your system.  Instructions and download links can be found at https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html and https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html.  Please note that you will need Windows 10 in order to run the driver station, but you can compile and deploy the code on Linux or Mac.

## Download and Installation
The first thing that you'll want to do is clone this repository onto your computer, which can be done through Github Desktop by going to Current Repository->Add->URL, and entering the URL for this repo.  You can also clone it from the command line by using `git clone <url>`.

Next, open a terminal in the `2021-Infinite-Recharge` project directory (not the Github repo directory) and type `./gradlew build`.  Assuming you did everything correctly, it should say the build was successful and display the time taken.  Be aware that the first build can take a while, especially on old PC's or ones with a slow internet connection.

## Using the Code
If you want to get the code onto the robot, open the `2021-Infinite-Recharge` project folder (again not the Github repo directory) and type `./gradlew deploy`.  Make sure the robot is on and you are connected to it either by WiFi, Ethernet, or USB.  The project will build, and then deploy onto the robot, and will be ready to go.  Now go to the driver station and you are ready to drive or test the robot.

## Vision
Before proceeding with this section, please see https://docs.wpilib.org/en/stable/docs/software/vision-processing/ and make sure that your Raspberry Pi for vision processing is set up correctly.  One good tip we've found is to use a USB stick in a Pi 4's USB 3.0 port (the blue ones) as the boot drive instead of the SD card.  This helps to prevent annoying corruption if sudden power loss occurs.

Inside the `Custom-Vision` folder, you can run the same `./gradlew build` command as in the robot project to compile the project.  Connect the Raspberry Pi that you are using for vision to either your computer's Ethernet port or the robot network.  Go to http://wpilibpi.local and go to the `Application` tab.  In the `Vision Application Configuration` section, select `Uploaded Java jar` from the dropdown menu and then click the `Browse` button.  Go to the folder with the vision project, and go to build->libs.  Select and upload the `****-all.jar` file.  Make sure that the Pi is set to writable, then click `Upload and Save`.  Wait 5 seconds, and the application should start and begin vision processing.  You can go to http://wpilibpi.local:1186 to view the processed image stream, or http://wpilibpi.local:1181 to see the unprocessed image stream.

## Robot Controls
The robot is controlled by two USB joysticks.  The first joystick should be in position 0 on the FRC Driver Station and is responsible for controlling the movement of the robot.  The y-axis on the left most analog stick of this controller controls the throttle, and the x-axis of the right most analog stick of this controller controls steering.  The other controller should be in position 1 on the FRC Driver Station, and is responsible for controlling the manipulator on the robot.  A summary of all buttons are as follows:<br>
`A` - Sets the RPM target of the flywheel to 5600<br>
`B` - Sets the RPM target of the flywheel to 6100<br>
`X` - Holding this button down runs the kicker, releasing it turns the kicker off<br>
`Y` - Toggles the LEDs on the vision system, behavior does not always seem consistent<br>
`SELECT` - Runs the turret initialization routine<br>
`START` - Toggles target auto tracking<br>
`LBUMP` - Drops the flywheel RPM target by 50<br>
`RBUMP` - Increases the flywheel RPM target by 50<br>
`DOWN` - Increases the turret angle (decreases the firing arc)<br>
`UP` - Decreases the turret angle (increases the firing arc)<br>
`LEFT` - Toggles the intake belts<br>
`RTRIG` - Toggles the flywheel, flywheel will attempt to reach its target RPM<br>
`LTRIG` - Holding down runs the intake, releasing stops the intake<br>
