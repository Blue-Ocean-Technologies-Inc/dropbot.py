# dropbot #

Firmware for the [DropBot digital microfluidics control system][1] and a Python
module for communicating with it over a serial connection.

<!-- vim-markdown-toc GFM -->

* [Install](#install)
* [Upload firmware](#upload-firmware)
* [Conda package contents](#conda-package-contents)
* [Usage](#usage)
    * [Example interactive session](#example-interactive-session)
* [Develop](#develop)
    * [Adding new remote procedure call (RPC) methods](#adding-new-remote-procedure-call-rpc-methods)
    * [Set up development environment (within a Conda environment)](#set-up-development-environment-within-a-conda-environment)
    * [Build firmware](#build-firmware)
    * [Flash/upload firmware](#flashupload-firmware)
    * [Unlink development working copy](#unlink-development-working-copy)
* [Contributors](#contributors)

<!-- vim-markdown-toc -->

-------------------------------------------------------------------------------

Install
-------

The latest [`dropbot` release][3] is available as a [Conda][2] package from the
[`sci-bots`][4] channel.

To install `dropbot` in an **activated Conda environment**, run:

    conda install -c wheeler-microfluidics -c conda-forge dropbot

-------------------------------------------------------------------------------

## Upload firmware ##

To upload the pre-compiled firmware included in the Python package, from an
**activated Conda environment** run the following command:

    python -m dropbot.bin.upload

-------------------------------------------------------------------------------

Conda package contents
----------------------

The `dropbot` Conda package includes:

 - `dropbot.SerialProxy` **Python class** providing a high-level interface to
   the DropBot hardware.
 - **Compiled firmware binary** for the DropBot hardware.

The installed components (relative to the root of the Conda environment) are
shown below:

    ├───Lib
    │   └───site-packages
    │       └───dropbot (Python package)
    │
    └───Library
        └───bin
            └───platformio
                └───dropbot (compiled firmware binaries)
                    │   platformio.ini   (PlatformIO environment information)
                    │
                    └───teensy31
                        firmware.hex

-------------------------------------------------------------------------------

## Usage ##

After uploading the firmware to the board, the `dropbot.Proxy` class can be
used to interact with the Arduino device.

See the session log below for example usage.

### Example interactive session ###

    >>> import dropbot

Connect to DropBot:

    >>> proxy = dropbot.SerialProxy()

Query the number of bytes free in device RAM.

    >>> proxy.ram_free()
    409

Query descriptive properties of device.

    >>> proxy.properties
    base_node_software_version                               0.9.post8.dev141722557
    name                                                                    dropbot
    manufacturer                                                           Sci-Bots
    url                                                                  http://...
    software_version                                                            0.1
    dtype: object

Use Arduino API methods interactively.

    >>> # Set pin 13 as output
    >>> proxy.pin_mode(13, 1)
    >>> # Turn led on
    >>> proxy.digital_write(13, 1)
    >>> # Turn led off
    >>> proxy.digital_write(13, 0)

Query number of available channels.

    >>> proxy.number_of_channels()
    120

Query state of all actuation channels.

    >>> proxy.state_of_channels


-------------------------------------------------------------------------------

Develop
-------

**The firmware C++ code** is located in the `src` directory.  The **key
functionality** is **defined in the `dropbot::Node` class in the file
`Node.h`**.


### Adding new remote procedure call (RPC) methods ###

New methods may be added to the Python API by adding new methods to the
`dropbot::Node` C++ class in the file `Node.h`.


### Set up development environment (within a Conda environment) ###

 1. **Clone `dropbot`** source code from [GitHub repository][5].
 2. Run the following command within the root of the cloned repository to
    **install run-time dependencies** and link working copy of firmware
    binaries and Python package for run-time use:

        paver develop_link

 4. **Restart terminal and reactivate Conda environment (e.g., `activate` if
    Conda was installed with default settings).**

Step **4** is necessary since at least one of the installed dependencies sets
environment variables, which are only initialized on subsequent activations of
the Conda environment (i.e., they do not take effect immediately within the
running environment).


### Build firmware ###

Run the following command within the root of the cloned repository to **build
the firmware**:

    paver build_firmware

The compiled firmware binary is available under the `.pioenvs` directory, as
shown below:

    └───.pioenvs
        └───teensy31
                firmware.hex


### Flash/upload firmware ###

To flash/upload a compiled firmware to a DropBot v3, run the following command
from the root of the repository:

    pio run --target upload --target nobuild


### Unlink development working copy ###

Run the following command within the root of the cloned repository to unlink
working copy of firmware binaries and Python package:

    paver develop_unlink

This will allow, for example, installation of a main-line release of the
`dropbot` Conda package.

-------------------------------------------------------------------------------

Contributors
------------

 - Christian Fobel ([@cfobel](https://github.com/cfobel))
 - Ryan Fobel ([@ryanfobel](https://github.com/ryanfobel))



[1]: http://sci-bots.com/dropbot
[2]: https://conda.io/
[3]: https://anaconda.org/sci-bots/dropbot
[4]: https://anaconda.org/sci-bots
[5]: https://gitlab.com/sci-bots/dropbot.py/
