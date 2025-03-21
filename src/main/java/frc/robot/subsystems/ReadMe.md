# Subsystem Spec
## An Overview
This document serves to explain the basic functionally of each subsytem. 

This spec is a living breathing document. Much like Doctor Frankestiens monster, at times it's going to be scared, confused, and a little ugly. Treat it kindly, and it will love you like a Father. Descriptions will update as robot design decisions (both design and programmatically) are made. Please let us know if there is anything that seems out of date or inaccurate.

This does not describe how each subsystem internally algorithms work. That will be done in a different location. It simply discusses what each subsystem is meant to do, it's states are, and what mechanical motors/pneumatics/sensors it has.

Thanks!
 

## Spec Author
Carl Lee Landskron

## Dev Team
Andrew 'Andy' Lohr </br>
Elliott Menon </br>
Grayson </br>
Edi Lazar

## Addition Mentors
Nelu Lazar

### Resources
[WPILIB Subsystems](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html)


## Nongoals
Under-dev

# Drivetrain
UnderDev

## States
Under Dev 

## Hardware
Under Dev, left as an example!
Swerve Drive Specialties MK4i L2 using </br>
4 Kracken motors for driving (6.75:1) </br>
4 Kracken motors for module rotation (150:7)

# Feeder
## States
UnderDev

## Hardware
1 Neo Vortexs //Todo: Add gear ratio

2 Lasercan for Coral Detection

# Armevator
UnderDev

## States
UnderDev

## Hardware
2 Neos Vortexs for lift 
### Gear Ratio
1.055544 inches per rotation

1 Neo Vortex for Arm
### Gear Ratio
58.7755 : 1

1 Mavcoder plugged into the coral motor for rotation

1 Limit Switch plugged into Neo Vortex for Elevator


# Climber
UnderDev


## States
UnderDev

## Hardware
1 Neo Vortex

1 lidar for distance checking

# Manipulator
UnderDev

## States
UnderDev

## Hardware
1 Neo Vortex for coral
1 Neo Vortex for algae


# Writing Specs
Specs written based on approach by Joel Spolsky.

[Painless Functional Specifications](https://www.joelonsoftware.com/2000/10/02/painless-functional-specifications-part-1-why-bother/)

[A Practical Guide to Writing Techincal Specs](https://stackoverflow.blog/2020/04/06/a-practical-guide-to-writing-technical-specs/)

[Markdown and Visual Code Docs](https://code.visualstudio.com/docs/languages/markdown)

[Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#links)
