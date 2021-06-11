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

The purpose of this repository is to provide a way to programmatically define an
inter-connected mechnical system (IMCS) -- a collection of rigid bodies -- to ultimately determine its:
- Coordinate system
- Forward Kinematic Maps
- Kinetic and Potential Energy
- Kinetic Energy Metric
- Generalised Forces
- Equations of motion
- Forced and Unforced Equilibria

The idea is to have a fully integrated way to define and describe IMCS, and produce useful content. Note, we refer the user to Geometric Control of Mechanical Systems (Bullo & Lewis) for definitions and descriptons used throughout.

IMCS are typically "simple" to scribble down on paper, but notoriously "difficult" to model correctly. Even with two bodies, the book-keeping and accounting on the rotation matrices alone, makes the accurate modelling task cumbersome and error prone. This repository is here to (help) solve that.

The goal is to be able to take a schematic drawing from paper, and by methodically using code,
define the body, or collection of bodies. The output of this effort can be one, or many of the following:
1. A diagram of the IMCS.
2. A latex document (and PDF), deriving the equations of motion.
3. A symbolic representation, that can be used a starting point for running simulations.

This repository relies on the principle that any system is simply a collection of independent bodies connected, via joints, in different configurations. **All definitions and descriptions are done in a Body's coordinate frame.** As such, the mainstays of this repository are the following classes:
- `Body`: a collection of particles. It has a mass, and some dimensions (length, width and height).
- `Joint`: a common location for two bodies. It is a description of a location of where they meet, and how the bodies move relative to each other. It is something that allows motion in certain directions (degrees of freedom), or enforces constraints. A `Joint` ultimately dictates the spatial coordinates (or variables) each body will have in the ICMS.
- `Connection`: defined by an input and output `Body`, and a `Joint`. By definition, the location of the joint is defined in both the input body and output body's coordinate frames, and as the configuration of the output `Body` can be written in terms of the input `Body`. A connection is also where we enforce the constraints of the `Joint`.
- `BodyCoordindate`: defines an (x,y,z) triple in the respective body coordinate frame, from its centre of mass. As we connect bodies to one another, we start to translate and rotate these coordinates by the position and rotation matrices (`Configuration`)of each `Body`.
- `MultiBody`: a sequence of `Connections`. If we are diligent with our definitions of coordinates, bodies and joints, defining connections is straightforward.
- `Ground`: every system needs reference to a global, or fixed coordinate frame. This is defined as the `Ground`. It does not translate. It does not rotate. It is our origin (0, 0, 0). Every `MultiBody` needs one.

## Installation & Usage

### Installation

1. Build from this repository:
    1. Build a local copy of the package (note for this to work, you must have python3-tk and pdflatex installed on your machine):
        1. `git clone https://github.com/smkyle90/skydy.git`
        2. `cd skydy`
        3. `pip install --upgrade .`
    2. Using Docker:
        1. `git clone https://github.com/smkyle90/skydy.git`
        2. `cd skydy`
        3. `make build`
2. Using pip and PyPi:
    1. `pip install skydy`. OS level dependencies for this method include `python3-tk` and `pdflatex`.

### Usage

We encourage the reader to review the [`examples`](https://github.com/smkyle90/skydy/tree/main/examples) folder for some basic examples. There are useable `.py` files in the `python` directory, as well as interative notebooks. The collection of examples are the simplest, yet most common systems that this modelling methodology is completed on and include:
1. A one-dimensional cart that moves in the x-coordinate.
2. A one-dimensional pendulum that rotates about an axis.
3. A cart-pendulum -- a combination of the two systems above.
4. A hovercraft -- an object that can move in two-dimensions and rotate.
5. A double pendulum.

For step-by-step development, the user is encouraged to run their code in an interactive notebook. The Docker image associated with this reporsitory has Jupyter installed. To enter an interactive session, simply run `jupyter notebook --allow-root` from the container and copy and paste the address the terminal provides into your browser of choice.

## Running

For this repository to function as intended, a few tools have been provided to ensure the application can be containerised.

### Makefile

The content of the `Makefile` should only be modified if the standard behaviour is not achieved using the default. Standard commands are as follows:

| Command  | Action | Image Tag (local and remote)
----------------------|---|---
`make run` | Runs a local image | Git commit's tag, otherwise `latest`

### Scripts

The `scripts` folder must maintain the following, which are indirectly run from the Makefile in the root directory. The `build` script is customizable per the  application, but it must build a local image of the application which can be uploaded the container repository for use in the cluster pipeline. The `upload` script **should not be modified**. The `run` script is listed as an empty placeholder script, and commented code is included in the Makefile to show how to integrate a `run` script into the code repository. The operation of the `run` scripts are application specific and do not need to share common functionality at this time.

| Script   | Inputs |Output|
|----------|------ |---
| build.sh  | NAME TAG | Application image is created locally, tagged with input args |
| run.sh    | NAME TAG | Application image is run locally |
| dev.sh    | NAME TAG | Application image is run locally, with screen privilidges for plotting and development purposes |

## Developing & Contributing
The guidelines for contributing are laid out here. All members of the team have write access.

### TODO
What I want to get done:
[] Documentation
[] Decent Documentation
[] Thorough Documentation
[] Add mass data to a Body for MassMatrix and InertiaMatrix objects
[] Kinematically constrained objects
[] Linear and rotational springs and dampers
[] Faster Forward Kinemtics
[] Rotating forces
[] Simulations
[] Systems analysis, including:
    [] Stability analysis
    [] Controllability (both Linear and Nonelinear)
[] Control Design

### Development Environment
- Install [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) for creating a nice virtual container to run in.
- See [Running](#Running).

### Testing
No untested code will be allowed to merge onto Master. A 90% coverage and test passing report is required for all Master PRs.

#### Using PyTest
This library uses pytest for testing. To run the full test suite use the command `pytest tests/ --cov=skydy --cov-report=html`.

Take note of the marks in `pytest.ini`. To run specific tests, say only on the "rigidbody" module, run `pytest tests/ --cov=skydy --cov-report=html -m rigidbody`.

One can use the args `-vv -s` to get detailed print outs during testing, i.e., `pytest tests/ --cov=skydy --cov-report=html -vv -s`

### Code Style
Code style is handled and enforced with [Black](https://github.com/psf/black), [Flake8](https://gitlab.com/pycqa/flake8) and some additional stylers and formatters. A pre-commit hook is provided with this repository so that all code is automatically kept consistent. If there are any issues with formatting, please submit a formal PR to this repository.

Docstrings will be formatted according to the Google docstring formatting, and as best as possible, styled as per the [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guide.
