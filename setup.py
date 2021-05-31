import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="PyDy",
    version="0.0.1",
    author="Scott Kyle",
    author_email="scott.m.kyle@gmail.com",
    description="A package to programmatically model mechanical systems.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    # url="https://github.com/smkyle90/sampleproject",
    # project_urls={
    #     "Bug Tracker": "https://github.com/smkyle90/sampleproject/issues",
    # },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "PyDy"},
    packages=setuptools.find_packages(where="PyDy"),
    python_requires=">=3.6",
)