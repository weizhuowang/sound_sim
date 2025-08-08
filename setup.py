from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="sound-sim",
    version="0.1.0",
    author="Weizhuo(Ken) Wang",
    description="Real-time sound synthesis for robot simulations",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/sound_sim",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy==1.23.0",
        "pyaudio>=0.2.11",
        "scipy>=1.10.0",
    ],
    extras_require={
        "mujoco": ["mujoco>=3.0.0"],
        "dev": [
            "pytest>=7.0",
            "black>=22.0",
            "isort>=5.0",
        ],
        "examples": [
            "mujoco>=3.0.0",
            "tqdm",
            "matplotlib",
        ],
    },
)
