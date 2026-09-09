# AGENTS.md for Pico-ASHA

## About project and structure

- Implements a USB audio to [ASHA](https://source.android.com/docs/core/connect/bluetooth/asha) adapter using a Raspberry Pi Pico W development board.
- Firmware written in C/C++20 using the Pico C/C++ SDK. Uses TinyUSB for USB audio and Bluekitchen BTStack for bluetooth.
- Firmware source in `src/` directory.
- Adapter can be interfaced over USB CDC serial connection. Protocol documented in `include/asha_comms.cpp`.
- Qt Application exists in `gui/` directory to interface with adapter.
- Website/documentation is written using `mkdocs` in the `website/` directory. Documentation is published to Github Pages.

## Tooling

- Use existing Python virtual environment which has `mkdocs` installed, if it exists.
- The Pico SDK path is set by the VS Code extension at the top of the root `CMakeLists.txt`.

## Code formatting and structure

- Use existing source code for guidelines on code formatting and structure.
- Do not use one-line conditionals without braces.
- Write comments where the code or function is not obvious.

## Safety and permissions

- Do not download any tools, scripts or executables.
- Do not download any packages without permission.
- Do not download any resources or bundles without permission.
- Do not write to any files outside the project root.
- Do not delete files or folders without permission.

## Git commits

- Do not commit without permission.
- Do not create or delete any branches.
- Do not push any changes to any remote.
- Use logical atomic commits.
- Add yourself as co-author to all commits.
- Write imperative commit messages.
- Write a concise commit message explaining the reason for the chage.

## When stuck
- ask a clarifying question

## Confirmation

Confirm you have read this AGENTS.md at the start of a session.