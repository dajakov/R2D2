# R2D2

![](images/R2D2_full_body.png)

This repository contains all source files for a robotic control system for a life-sized R2D2 robot. The 3D-files are made by [Michael Baddeley](https://www.patreon.com/mrbaddeley).

I started this project because I needed a smart and reliable way to control R2D2 via an android app. So I started developing a simple proof of concept app in Android Studio with just a joystick for the velocity control, two buttons to rotate the dome and a text field to show debugging messages.
The communication to the hardware works via a websocket connection to an ESP01.

![](images/R2D2_app.png)

