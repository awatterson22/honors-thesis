# Honors Thesis of Abby Watterson

The primary purpose of this project is to measure stress levels using heart rate variability and
temperature in dogs and test the effectiveness of Dog Appeasing Pheromones on stress by
releasing them from a collar and/or harness when they are above a specific threshold that
indicates stress.

## Installation

Clone the Repository

```bash
git clone https://github.com/awatterson22/honors-thesis.git
```

Install and Configure Arduino Cli on Raspberry Pi

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
arduino-cli config init
```

## Usage on Raspberry Pi

```bash
cd honors-thesis/main

~/bin/arduino-cli -b=arduino:avr:uno compile
~/bin/arduino-cli -b arduino:avr:uno -p /dev/ttyACM0 upload

arduino-cli sketch main
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.
