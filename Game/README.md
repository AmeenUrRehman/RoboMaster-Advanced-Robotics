# RoboMaster Quest (school-visit game)

A single web page for students. No installation, no code to read:

1. Open `index.html` in any browser (double-click it; it works offline except for the fonts, which fall back to system fonts).
2. **1 Drive Robo**: collect 8 stars in 30 s with W A S D (forward, back, slide), Q E (turn), or the on-screen buttons. Switch between **Cable (USB)** and **Wi-Fi** and turn up the *lag magnifier*.
3. **2 Find the Tag**: drag the square code (T3) anywhere, choose which way Robo faces, press **GO!**. Robo searches, lines up, follows a smooth speed plan, and stops 20 cm away. The four wheel-power meters show the motor voltages live.
4. **3 Cable vs Wi-Fi**: a message race that shows why the cable is steadier.
5. **How Robo thinks**: the four steps, the magic code, and the speed plan.

The game is a **simulation**. It uses the same state machine, the same cubic speed plan (peak 0.137 m/s) and the same motor-voltage formula as the real ROS 2 package in `Assignment 2/`, and the delay shapes copy the measured cable and Wi-Fi behaviour of `Assignment 1/`. The triangle (T1) and circle (T2) are decoys that Robo ignores; the real printed tags are in `Assets/`.
