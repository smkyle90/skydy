<p align="center">
<a href="https://github.com/psf/black"><img alt="code style: black" src="https://img.shields.io/badge/code%20style-black-000000.svg"></a>
<a href="https://gitlab.com/PyCQA/flake8"><img alt="code style: flake8" src="https://img.shields.io/badge/code%20style-pep8-orange.svg"></a>
<a href="https://github.com/PyCQA/bandit"><img alt="security: bandit" src="https://img.shields.io/badge/security-bandit-yellow.svg"></a>
<a href="https://github.com/pre-commit/pre-commit"><img src="https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white" alt="pre-commit" style="max-width:100%;"></a>
</p>

---

_Contents:_
**[Background](#background)** |
**[Installation](#installation)** |
**[Running](#running)** |
**[Contributing](#contributing)** |

---

## Background

Welcome to SkyDy—a *Dy*namics package for Python built by me, *S*cott *Ky*le.

The purpose of this repository is to provide a way to programmatically define an
inter-connected mechnical system (ICMS)—a collection of rigid bodies—to ultimately determine its:
- coordinate system;
- forward kinematic maps;
- kinetic and potential energy;
- kinetic energy (Riemannian) metric;
- generalised forces;
- equations of motion;
- forced and unforced equilibria.

The idea is to have a fully integrated way to define and describe ICMS, and produce useful content. Note, we refer the user to [Geometric Control of Mechanical Systems (Bullo & Lewis)](https://link.springer.com/book/10.1007/978-1-4899-7276-7) for definitions and descriptons used throughout.

ICMS are typically "simple" to scribble down on a piece of paper, but notoriously "difficult" to understand how they move (or model correctly). Even with two bodies, the book-keeping and accounting on the rotation matrices alone renders the modelling task cumbersome and error prone. This repository is here to (help) solve that.

The goal is to be able to take a schematic drawing from paper, and by methodically using code,
define the ICMS. The output of this effort can be one, or many of the following:
1. A diagram of the ICMS.
2. A latex document (and PDF), deriving equations of motion.
3. A symbolic representation, that can be used a starting point for running simulations.

This repository relies on the principle that any system is simply a collection of independent bodies connected, via joints, in different configurations. As such, the mainstays of this repository are the following classes:
- `Body`: a collection of particles. It has a mass, and some dimensions (length, width and height), and has six degrees of freedom (DOFs).
- `Joint`: a common location for two bodies to interact, and how the bodies can move relative to each other. A `Joint` allows motion in certain directions (DOFs) and/or enforces constraints. Thus, a `Joint` dictates the spatial coordinates (or variables) each body will have contribute to the ICMS.
- `Connection`: defined by an input and output `Body`, and a `Joint`. By definition, the location of the joint is defined in both the input body and output body's coordinate frames. Therefore, the configuration of the output `Body` can be written in terms of the input `Body`. A connection is also where we enforce the constraints of the `Joint`.
- `BodyCoordindate`: defines an (x, y, z) triple in the respective body coordinate frame, from its centre of mass. As we connect bodies to one another, we start to translate and rotate these coordinates by the position and rotation matrices (`Configuration`) of each `Body`.
- `MultiBody`: a sequence of `Connections`. If we are diligent with our definitions of coordinates, bodies and joints, the creation of a `MultiBody` is straightforward.
- `Ground`: every system needs reference to a global, or fixed coordinate frame. This is defined as the `Ground`. It does not translate. It does not rotate. It is our origin (0, 0, 0). Every `MultiBody` needs one.

For this methodology, **all definitions and descriptions are done in a Body's coordinate frame**.

## Installation & Usage

### Installation

1. Build from this repository:
    1. Build a local copy of the package (see note):
        1. `git clone https://github.com/smkyle90/skydy.git`
        2. `cd skydy`
        3. `pip install --upgrade .`
    2. Using Docker:
        1. `git clone https://github.com/smkyle90/skydy.git`
        2. `cd skydy`
        3. `make build`
2. Using pip and [PyPi](https://pypi.org/project/skydy/) (see note):
    1. `pip install skydy`.

**Note: For `1.1` and `2.1`, OS level dependencies include `python3-tk` and `pdflatex`.**

### Usage

We encourage the reader to review the [`examples`](https://github.com/smkyle90/skydy/tree/main/examples) folder for some basic examples. There are useable `.py` files in the `python` directory, as well as interative notebooks. The collection of examples are the simplest, yet most common, systems this modelling methodology can be used on, and include:
1. A one-dimensional cart that moves in the x-coordinate.
2. A one-dimensional pendulum that rotates about an axis.
3. A cart-pendulum—a combination of the two systems above.
4. A hovercraft—an object that can move in two-dimensions and rotate.
5. A double pendulum.


For step-by-step development, the user is encouraged to run their code in an interactive notebook. This will leverage the power of `Sympy` and the `skydy` process. The Docker image associated with this repository has Jupyter installed. To enter an interactive session, simply run `jupyter notebook --allow-root` from the container and copy and paste the address the terminal provides into your browser of choice.

### Documentation
Skydy documentation can be found [here](https://skydy.readthedocs.io/en/latest/) on Read The Docs! The docs are aligned with the latest Github release.

## Running

For this repository to function as intended, a few tools have been provided to ensure the application can be containerised.

### Makefile

The content of the `Makefile` should only be modified if the standard behaviour is not achieved using the default. Standard commands are as follows:

| Command  | Action | Image Tag (local and remote)
----------------------|---|---
`make run` | Runs a local image | Git commit's tag, otherwise `latest`

### Scripts

The `scripts` folder must maintain the following, which are indirectly run from the Makefile in the root directory. The `build` script is customizable per the  application, but it must build a local image of the application. The `run` script  Thruns the container without local screen priviledged. The `dev` script allows the user to use the screen of their local machine.

| Script   | Inputs |Output|
|----------|------ |---
| build.sh  | NAME TAG | Application image is created locally, tagged with input args |
| run.sh    | NAME TAG | Application image is run locally |
| dev.sh    | NAME TAG | Application image is run locally, with screen priviledsges for plotting and development purposes |

## Developing & Contributing
The guidelines for contributing are laid out here. All members of the team have write access.

### TODO
What I want to get done:
- [x] Documentation (Completed June 13, 2021. Docstrings added and infrastructure for Read The Docs & Sphinx autodocs.)
- [ ] Decent Documentation
- [ ] Thorough Documentation
- [ ] Prettier Latex printing
- [ ] Add mass data to a Body for MassMatrix and InertiaMatrix objects
- [ ] Kinematically constrained objects
- [ ] Translationall and rotational springs and dampers
- [ ] *Faster* Forward Kinemtics
- [ ] Dig into `sympy.physics.mechanics` and their Lagrangian capabilties
- [ ] Rotating forces
- [ ] Simulations
- [ ] Systems analysis, including:
    - [ ] Stability analysis
    - [ ] Controllability (both Linear and Nonlinear)
- [ ] Control Design
- [ ] Decrease the size of the Docker image
- [ ] Add an image to Docker hub

### Development Environment
- Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) for creating a nice virtual container to run in.
- See [Running](#Running).

### Testing
No untested code will be allowed to merge onto Master. A 90% coverage and test passing report is required for all Master PRs.

#### Using PyTest
This library uses pytest for testing. To run the full test suite use the command `pytest tests/ --cov=skydy --cov-report=html`.

Take note of the marks in `pytest.ini`. To run specific tests, say only the "rigidbody" module, run `pytest tests/ --cov=skydy --cov-report=html -m rigidbody`.

One can use the args `-vv -s` to get detailed print outs during testing, i.e., `pytest tests/ --cov=skydy --cov-report=html -vv -s`

### Code Style
Code style is handled and enforced with [Black](https://github.com/psf/black), [Flake8](https://gitlab.com/pycqa/flake8) and some additional stylers and formatters. A pre-commit hook is provided with this repository so that all code is automatically kept consistent. If there are any issues with formatting, please submit a formal PR to this repository.

Docstrings will be formatted according to the Google docstring formatting, and as best as possible, styled as per the [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guide.
