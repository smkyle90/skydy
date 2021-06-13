import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

skydy_packages = [
    "skydy",
    "skydy.connectors",
    "skydy.configuration",
    "skydy.inertia",
    "skydy.multibody",
    "skydy.connectors",
    "skydy.output",
    "skydy.rigidbody",
]

setuptools.setup(
    name="skydy",
    version="0.0.1b",
    author="Scott Kyle",
    author_email="scott.m.kyle@gmail.com",
    description="A package to programmatically model inter-connected mechanical systems.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/smkyle90/skydy",
    project_urls={
        "Bug Tracker": "https://github.com/smkyle90/skydy/issues",
        "Documentation": "https://skydy.readthedocs.io/",
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
