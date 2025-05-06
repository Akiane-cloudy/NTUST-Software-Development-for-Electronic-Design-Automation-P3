# Detailed Routing for Photonic Integrated Circuits (PICs)

## Overview

This project implements a Detailed Routing for Photonic Integrated Circuits (PICs) using A* search.

## Course Information

- Course: Software Development for Electronic Design Automation
- Instructor: Prof. Shao-Yun Fang
- Project: Detailed Routing for Photonic Integrated Circuits (PICs)

## Program Architecture

```
/b11115004-p3
│
├── main.cpp
├── Makefile
├── report.pdf
└── README.md
```

## Development Tools

- Programming Language:
  - Utilized: C++
  - Standard: C++17
  - Compiler: g++
- Development Environment:
  - Local Platform: macOS
- Compilation Script:
  - Makefile

## Prerequisites

- Ensure that all dependencies required for compiling C++ code are installed.

## Compilation Process

1. Navigate to the directory containing the Makefile by using `cd <path_to_directory>`.
2. Execute the compilation by entering `make`.

## Usage

1. Change directory to the location where the executable 'legalizer' is present using `cd <path_to_directory>`.
2. Run the program using the following command (replace the contents of "[]" with the input file and output file:

```
./picRouting [input file name] [output file name]
```

3. The program will produce an output file named `[output file name]`.

4. (Optional) Execute `make clean` to remove all object files and the executable.

**Notes**:

- Further details regarding the underlying data structures and algorithms will not be provided in this document.
- For any inquiries, please contact B11115004@mail.ntust.edu.tw.