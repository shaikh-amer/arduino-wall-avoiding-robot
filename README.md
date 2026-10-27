# Robot project

Arduino Uno firmware, built and tested in the browser.

- `sketch.ino` is the program.
- `diagram.json` is the circuit: the parts and the pins they use.
- The agent compiles the sketch and runs it on a simulated board wired to those parts; the Hardware tab in the preview shows the circuit, the robot arena (when there is one) and every run.

Try asking the agent: "Add a servo on pin 9 that sweeps from 0 to 180 while the button is held", or "Make a two-wheel robot that drives through the corner arena without touching the walls".
