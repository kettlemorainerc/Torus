# Robot template

---
This is a basic template for team 2077 bots. There's a few important files to know about
as you star editing the code for a new robot. 

- [DriveStation](src/main/java/org/usfirst/frc/team2077/DriveStation.java)
  - It's the location where we set up the robot for user control.
    Typically, you set up controllers, their button bindings, and any default
    commands for subsystems.
- The [command](src/main/java/org/usfirst/frc/team2077/command) package
  - You will add new commands to this package that will then be used in
    autonomousInit/DriveStation
- [RobotHardware](src/main/java/org/usfirst/frc/team2077/RobotHardware.java)
  - This is the location where subsystems will be added as well as initializing our chassis
- [Robot](src/main/java/org/usfirst/frc/team2077/Robot.java)
    - Normally you'll only need to really touch the autonomousInit method in order
      to set up and execute a commands in autonomous mode.


## Getting Started

- Create a new, blank, uninitialized github repository
  - The repository URL for the new repository will be identified as `<target repo>`
    - so if the name of the repository is "new-robot" the link will most likely be `https://github.com/kettlemorainerc/new-robot.git`
  - The folder of the new robot repo name will be identified as `<target dir>`
- Clone the repository
  - Either using terminal/cmd commands
    - ```shell
      git clone --recurse-submodules https://github.com/kettlemorainerc/robot-template.git <target dir>
      cd <target dir>
      ```
  - Or using IntelliJ's built-in repository cloning
    - `File > New > Project from Version Control...`
    - Set the URL to `https://github.com/kettlemorainerc/robot-template.git`
    - Optionally change the Directory
    - Click "Clone"
- Modify the remote URL of the local repository
  - In a terminal you can do
    - ```shell
      git remote rename origin template
      git remote add origin https://github.com/kettlemorainerc/<taget repo>
      ```
  - In IntelliJ 
    - go to `Git > Manage Remotes`
    - Update the "origin" option's name to "target"
    - Then add a new remote with the name "origin" and a URL of `<target repo>`
