## Fixes completed

TODO.

## Fixes to be implemented

  - This is code to run a gps_time app on a Tufty 2350
  - Add Tufty battery life indicator in top right (like the one on the home screen), and move the 'X sats' indicator to the top middle
  - When there is no fix, make the UTC timestamp orange
  - Add 'Back' above button A on the bottom left and when clicked, take me back to the home screen (making sure RTC time is updated from GPS before going home, if GPS has a fix)
  - Add my name 'Jeff Geerling' just above the UTC time the same font size as local time
  - Add GPS info button on bottom right to show all GPS data (show like sats in view, fix status, any other neat and cool stuff Claude enjoys), with back button on bottom left from that screen to go back to main GPS clock display
  - If I plug in the GPS module while the app is running, it goes to 00:00:00Z and it seems like the whole thing locks up. Is it possible to hot-plug the GPS module while running?

## PPS feature

  - Add PPS led blink on back when PPS pulse is received
  - Add code to discipline to PPS signal