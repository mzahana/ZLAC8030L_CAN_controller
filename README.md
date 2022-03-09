# ZLAC80380L_CAN_controller
A Python package for CANopen communication with the ZLAC8030L motor driver

# Prerequisites
A modified `canopen` package is required, [here](https://github.com/mzahana/canopen/tree/python2)
```bash
git clone https://github.com/mzahana/canopen.git
cd canopen
git checkout python2
python setup.py develop --user
```

See the remaining prerequisites in the `requirements.txt` file.
This branch `python2` is suitable for Python 2.7, and specific Python

# Installation
After installing the custom `canopen` version as mentioned above, go ahead and install this packge.

To setup/install package in develop mode
```bash
git clone https://github.com/mzahana/ZLAC8030L_CAN_controller.git
cd ZLAC8030L_CAN_controller
git checkout python2
python setup.py develop --user
```
# Usage
See the `tests/speed_test.py` script for example use.

