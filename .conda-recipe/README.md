Conda recipe to build `dropbot` package.

Build
=====

Install `conda-build`:

    conda install conda-build

Build recipe:

    conda build . -c sci-bots -m variants.yaml


Install
=======

The pre-built package may be installed from the [`alexsk`][2] channel using:

    conda install -c alexsk dropbot


[1]: https://anaconda.org/sci-bots/dropbot
[2]: https://anaconda.org/sci-bots
