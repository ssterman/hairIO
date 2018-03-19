# HairIO

Human hair is a cultural material, with a rich history displaying individuality, cultural expression and group identity.  It is malleable in length,  color and style,  highly visible,  and embedded in a range of personal and group interactions. As wearable technologies move ever closer to the body, and embodied interactions become more common and desirable, hair presents a unique and little-explored site for novel interactions. In this paper, we present an exploration and working prototype of hair as a site for novel interaction, leveraging its position as something both public and private,  social and personal, malleable and permanent. We develop applications and interactions around this new material in HäirIÖ: a novel integration of hair-based technologies and braids that combine capacitive touch input and dynamic output through color and shape change. Finally, we evaluate this hair-based interactive technology with users, including the integration of HäirIÖ within the landscape of existing wearable and mobile technologies.

Work by: Christine Dierk, Sarah Sterman, Molly Jane Nicholas, Eric Paulos

Hybrid Ecologies Lab
hybrid-ecologies.org

(Video)[https://www.youtube.com/watch?v=8JV2D7gJ5HI]

(Paper)[https://dl.acm.org/citation.cfm?id=3173232]

## Arduino sketches

In this repo you will find example sketches for use on the HairIO PCB, demonstrating capacitive touch, actuation, and bluetooth control.  

*demo_captouch:* A basic sketch showing only capacitive touch sensing with manually defined sensing thresholds. The onboard LED will light when a touch is identified.

*demo_timing*: A basic sketch showing only actuation.  Every period of x seconds, the braid will actuate, then turn off.  The onboard LED will light when the braid is actuating.

*demo_pcb_bluetooth_with_drive_captouch*: All-inclusive demo that uses the Adafruit Bluetooth app to provide touch feedback and control actuation through the bluetooth chip. 

All hardware pins are defined assuming the PCB version of the HairIO circuit, though all pin mappings should be double-checked when you make your own hardware. 


## Eagle Files

In *hairio_full_3* you will find the Eagle schematic and board file for the integrated PCB version of the HairIO circuit. 

## Instructable

For a full description of this project, please see the (Instructable)[https://www.instructables.com/id/HairIO-Hair-As-Interactive-Material/] 

