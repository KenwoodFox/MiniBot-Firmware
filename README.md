[![Docs](https://github.com/KenwoodFox/MiniBot-Firmware/actions/workflows/docs_workflow.yml/badge.svg)](https://github.com/KenwoodFox/MiniBot-Firmware/actions/workflows/docs_workflow.yml)
[![Firmware](https://github.com/KenwoodFox/MiniBot-Firmware/actions/workflows/firmware_workflow.yml/badge.svg)](https://github.com/KenwoodFox/MiniBot-Firmware/actions/workflows/firmware_workflow.yml)


# MiniBot-Firmware

![Banner](Static/Banner.png)

This repo contains all the firmware and hardware for the MiniBot-Firmware.

If you're looking for the latest docs/builds, see our [Releases Page](https://github.com/KenwoodFox/MiniBot-Firmware/releases).

# Getting Started

First, clone this repo (and optionally checkout a branch)

```shell
git clone https://github.com/KenwoodFox/MiniBot-Firmware.git
cd MiniBot-Firmware
```

# Init Submodules

Some libraries and resources are included as submodules, run the following
command to initialize them before opening the main sch

(If you get a missing library error, make sure to do this)

```shell
git submodule update --init --recursive
```


## Project Layout

If you want to use this project template for yourself, you can find it [here!](https://github.com/KenwoodFox/Project-Template)
