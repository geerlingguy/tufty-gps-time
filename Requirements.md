# Requirements (initial prompt)

I would like a little app to display GPS based time on a Pimoroni Tufty 2350 running 2.0.2 of their firmware. Here is the product https://shop.pimoroni.com/products/tufty-2350 and here are the docs https://badgewa.re/docs

I already have an icon and the folder "gps_time" for the app, but help me write the init python script to:

  1. Display time from the internal RTC if it is available, both as UTC and on the next line 'Local' with UTC minus a configurable offset (via 'LOCAL_OFFSET' variable like '-6' for central time)

  2. Display 'NO FIX' in red in the top left corner until a GPS fix is acquired and time data is present

  3. Display "0 sats" in red the top right corner until GPS satellites are acquired

As satellites are acquired, update the top right corner '0 sats' to have the number of sats. Keep it red for '0 sats', orange for 1-4 sats, and green for 5+ sats.

Once a GPS fix is established (I'm using an Adafruit Mini GPS PA1010D connected via QWIIC):

  1. Update the system RTC (if it is available) with GPS time once per hour (or at some other useful interval)

  2. Display the time in both UTC and a configurable local time (local time using AM/PM with offset from a 'LOCAL_OFFSET' variable)

## Fixes completed

  - Add Tufty battery life indicator in top right (like the one on the home screen), and move the 'X sats' indicator to the top middle
  - When there is no fix, make the UTC timestamp orange
  - Add my name 'Jeff Geerling' just above the UTC time the same font size as local time
  - Add GPS info button on bottom right to show all GPS data (show like sats in view, fix status, any other neat and cool stuff Claude enjoys), with back button on bottom left from that screen to go back to main GPS clock display
  - If I plug in the GPS module while the app is running, it goes to 00:00:00Z and it seems like the whole thing locks up. Is it possible to hot-plug the GPS module while running?
  - Add two info pages with GPS data.
  - Add GPS sky view plot as 3rd info page.
  - Add settings page (maybe with 'B' and then back) where you can adjust the local offset in 0.5 increments (default to the setting in code but allow user to override)
    - Maybe also setting for 24H vs 12H time for local time display

## Fixes to be implemented

  - Add 'Home' PNG icon (similar style, size, and transparency to the settings cog) above button A on the bottom left. When clicked, take me back to the Tufty home screen.

## PPS feature

  - Add PPS led blink on back when PPS pulse is received
  - Add code to discipline to PPS signal
