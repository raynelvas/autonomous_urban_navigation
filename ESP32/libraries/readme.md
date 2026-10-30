
# Instructions for adding custom libraries to the Arduino IDE

## Description
This document explains how to copy and install the custom libraries required for your projects in **Arduino IDE**.

---

## Steps

1. **Open the Arduino IDE**

   Launch the Arduino development environment.

2. **Go to Preferences**

   In the menu bar, select:

        File → Preferences

3. **Locate the Sketchbook folder**

   Within the preferences window, look for the **Sketchbook location** field.
By default, the path is usually:

        /home/$USER/snap/arduino/current/Arduino


4. **Open the Libraries folder**

   Within the previous location, access the folder:

        libraries


5. **Copy custom libraries**

   Copy the following folders into the directory `libraries`:

        motorControl
        SabertoothSimplified

## Verificación

1. Restart **Arduino IDE**.  
2. Open the menu:

        Sketch → Include Library → Manage Libraries...

3. Verify that the libraries appear in the list or that you can include them in your code using:
        ```cpp
        #include <SabertoothSimplified.h>
        #include <motorControl.h>
        ```
---

**Author:** Luis Raynel Vasquez Quishpe

**Last updated:** Octubre 2025
