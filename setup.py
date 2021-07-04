import os

import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

path = "skydy"
skydy_packages = [path]

with os.scandir(path) as it:
    for entry in it:
        if entry.is_dir() and not entry.name.startswith("__"):
            skydy_packages.append(f"{path}.{entry.name}")

setuptools.setup(
    name="skydy",
    version="0.0.2",
    author="Scott Kyle",
    author_email="scott.m.kyle@gmail.com",
    description="A package to programmatically model inter-connected mechanical systems.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/smkyle90/skydy",
    project_urls={
        "Documentation": "https://skydy.readthedocs.io/",
        "Bug Tracker": "https://github.com/smkyle90/skydy/issues",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    packages=skydy_packages,
    python_requires=">=3.6",
    install_requires=["numpy", "sympy", "matplotlib"],
)
