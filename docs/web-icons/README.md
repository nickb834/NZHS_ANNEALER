# Annealer monitor web icons

These assets support browser favicons, iOS Home Screen Web Clips and the web
application manifest served by the Uno R4 WiFi firmware.

The visual reference is the MGNZ Makes wordmark displayed on
[mgnz-makes.com](https://www.mgnz-makes.com/). The generated mark preserves the
recognisable gold M maker symbol on black and adds a restrained induction-coil
ring for the annealer monitor. It deliberately contains no small text.

Files:

- `annealer-monitor-source.png` — generated project source;
- `apple-touch-icon.png` — 180x180 iOS icon;
- `icon-192.png` — web-app manifest icon;
- `icon-512.png` — nearest-neighbour expansion of the 192 px icon to minimise
  firmware size while satisfying the manifest size;
- `favicon.ico` — combined 16, 32 and 48 px browser icon.

Generation used the built-in image-generation tool with the official wordmark
as the visual reference. Final prompt summary: create a square, high-contrast,
black-and-gold app icon preserving the left-hand MGNZ maker mark, retain the
vertical pin/case element and partial frame, add one subtle induction-coil arc,
and remove all words, photographic detail, shadows and small texture so it
remains readable at 32 px.
