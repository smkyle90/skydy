FROM python:3.9.5-slim-buster

# Install system packages that are considered standard for all Python applications
RUN apt-get update && apt-get -y --no-install-recommends install \
    # software-properties-common \
    # build-essential \
    make \
    git \
    python3-tk \
    texlive-latex-base \
    texlive-fonts-recommended \
    texlive-fonts-extra \
    && apt-get -y autoremove \
    && apt-get clean autoclean

# Install application specific system packages
# RUN apt-get install -y PACKAGE

RUN pip install --upgrade pip

# Copy and install third party dependencies
# packages, thus the two COPY / RUN statement pairs should speed up rebuilds
COPY ./requirements.txt requirements.txt
RUN pip install -r requirements.txt

RUN pre-commit install

# Copy the current directory contents into the container at /skydy
COPY . /skydy/

# Set the working directory to /skydy
WORKDIR /skydy
