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
    * [Build the package](#build-the-package)
    * [Release](#release)
    * [Build firmware](#build-firmware)
    * [Flash/upload firmware](#flashupload-firmware)
* [Contributors](#contributors)

<!-- vim-markdown-toc -->

-------------------------------------------------------------------------------

Install
-------

Releases are published as a Conda package to the [`vigne-hub/scibots`][3]
channel on prefix.dev. Add the channel (plus the channels carrying the
sci-bots dependency stack) to a [pixi][2] workspace and depend on `dropbot`:

    [workspace]
    channels = ["conda-forge", "alexsk", "vignesh229", "https://prefix.dev/vigne-hub/scibots"]

    [dependencies]
    dropbot = ">=1.74.4,<2"

-------------------------------------------------------------------------------

## Upload firmware ##

To upload the pre-compiled firmware included in the Python package, from an
**activated Conda environment** run the following command:

    python -m dropbot.bin.upload

-------------------------------------------------------------------------------

Conda package contents
----------------------

Two Conda packages are built from this repository:

 - `dropbot` (noarch python): the `dropbot.SerialProxy` **Python class**
   providing a high-level interface to the DropBot hardware. It pins the
   matching `dropbot-dev`.
 - `dropbot-dev` (noarch generic): the **compiled firmware binary** and the
   generated **Arduino library headers**, for firmware builds that depend on
   DropBot.

The installed components (relative to the root of the Conda environment) are
shown below:

    ├───Lib
    │   └───site-packages
    │       └───dropbot (Python package)
    │
    └───share
        └───platformio
            ├───include
            │   └───Dropbot (generated Arduino library headers)
            │
            └───bin
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


### Build the package ###

The Conda package is built with [pixi][2] through the rattler-build backend
(`recipe/recipe.yaml`). The build generates the RPC code, compiles the
protobufs, compiles the Teensy firmware with PlatformIO and splits the result
into the `dropbot` and `dropbot-dev` `noarch` packages:

    pixi build

The resulting `.conda` files land in the current directory
(`--output-dir` to change that).


### Release ###

Releases are automated (`.github/workflows/publish.yml`). Every push to
`master` with release-worthy [Conventional Commits][6] (`fix` -> patch,
`feat` -> minor, `BREAKING CHANGE` -> major) makes [commitizen][7] bump the
version in `pyproject.toml`, `dropbot/_version.py` and `recipe/recipe.yaml`,
update `CHANGELOG.md`, build and publish the package with `pixi publish`, and
then push the release commit and `vX.Y.Z` tag. Authentication is prefix.dev
trusted publishing (GitHub OIDC), so no API key is stored in the repository.

To force a release with an explicit version, run the workflow manually from
the Actions tab and fill in the version input.


### Build firmware ###

Run the following command within the root of the cloned repository to **build
the firmware**:

    pio run

The compiled firmware binary is available under the `.pio/build` directory, as
shown below:

    └───.pio
        └───build
            └───teensy31
                    firmware.hex


### Flash/upload firmware ###

To flash/upload a compiled firmware to a DropBot v3, run the following command
from the root of the repository:

    pio run --target upload --target nobuild


-------------------------------------------------------------------------------

Contributors
------------

 - Christian Fobel ([@cfobel](https://github.com/cfobel))
 - Ryan Fobel ([@ryanfobel](https://github.com/ryanfobel))



[1]: http://sci-bots.com/dropbot
[2]: https://pixi.sh/
[3]: https://prefix.dev/channels/vigne-hub/scibots
[6]: https://www.conventionalcommits.org/
[7]: https://commitizen-tools.github.io/commitizen/
