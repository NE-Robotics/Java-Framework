# Java-Framework
An opensource FRC java framework to make blending simulated &amp; real robot code a breeze since as every programmer knows you will always get the bot last, always :)

## Features
- Super, sim, real: Super classes allow for a simple merging of sim and real code with an example differential drive given.
- Keep it constants: All key variables are stored within a constants file for easy organization.
- Auto auto: Inserted pathplanner autos are instantly added to an auto chooser.
- Lots o' logs: AdvantageKit logging is built in by default, including metadata.
- Tons o' tags: Support for multiple apriltag detectors with blended uncertainties based on the robots position.

## What does this give you?
- Full match playback off the logs & localization data when using AdvantageScope or In Control.
- Accurate autos from reliable location data.
- Easy testing without your robot with work done in simulation transfering easily to the real bot.
- A simple starting point for advanced features for newer teams.
- Full support from Northeastern Robotics, if you struggle with this code you can contact us at any time.

## Getting Started
To begin with this framework just clone the repository & navigate to Rio/.wpilib/wpilib_preferences.json and set the team number to match your own.
If you are using WPILib vscode you may want to open the Rio folder while writing robot code instead of the primary folder as the FRC vscode extension does not always function properly if not in the correct directory.

Simulation support is ready from the start with a basic differential drive sim but this can be swapped ti match your robot or planner robot.

## Note 
A similar but more advanced framework including 1000 hertz simulation with proper physics and wrappered systems based on work done by [gerth2](https://github.com/gerth2/SwerveBase2023) is under development, for anyone interested please let me know and early access to trial it will be opened up.
