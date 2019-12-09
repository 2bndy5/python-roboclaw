
.. image:: https://readthedocs.org/projects/python-roboclaw/badge/?version=latest
    :target: https://python-roboclaw.readthedocs.io/en/latest/?badge=latest
    :alt: Documentation Status

Introduction
----------------

Roboclaw driver library and examples adapted for python3 and circuitpython and micropython

Optimizations applied to the original code include UART Serial I/O & CRC checking. These optimizations are meant to allow your application to run faster than the vanilla python library offered by BasicMicro.

Installation
------------

The best way to make sure you have the latest version of this library is by cloning the repository, and running the python setup script.

.. code-block:: shell

    git clone htpps://github.com/2bndy5/python-roboclaw.git
    cd python-roboclaw
    python setup.py install

Usage
------

Once you have installed the library, you can import it into your python application. Please note that the roboclaw requires a USB serial connection as well as the main power source (or battery) for the motors connected to the "+" & "-" terminals for proper communication. In your applications code, you need only import the ``Roboclaw`` driver class.

.. code-block:: python

    from python_roboclaw import Roboclaw
    from serial import Serial

    serial_obj = Serial('dev/ttyUSB0', 38400) # default baudrate is 38400
    rclaw = RoboClaw(serial_obj)
    rclaw.forward_backward_mixed(64) # stops both motors
