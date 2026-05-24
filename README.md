# Rebellion Template

[![Build and Test](https://github.com/10014Rebellion/rebellion-template/actions/workflows/build.yml/badge.svg)](https://github.com/10014Rebellion/rebellion-template/actions/workflows/build.yml)
<!-- //TODO Update this URL so it points to the right repo -->

## Setting up the environment

### Requirements

- git (w/ git bash on Windows)
- WPILib

### Installing

Open git bash or your sh of choice and run the following commands:

```sh
git clone <the URL of this repository>.git
cd <the name of the repository>
./scripts/install-hooks.sh
```

Then open the root directory of the repository in WPILib VSCode

## Flow

All code changes should happen on a branch, and then a PR should be opened and reviewed by multiple senior members or leads before being merged. Commits should contain a single set of related changes, and commit names should be descriptive of their contents. Err on the side of granularity when committing, as it's easier to clean up later. Push whenever code is in a reasonable state and prior to leaving. Pull often, and keep up with changes on `main` and other branches. Reviews should be thorough, lest you tear your hair out debugging later (do *not* LGTM). No code should go into `main` that hasn't been simulated thoroughly and/or tested on a robot, and no incomplete or in-progress code should be merged. All code should be formatted on build, and should follow WPILib command-based programming best practices.

## Lib Directory

The `lib` directory (`rebellion-template/src/main/java/frc/lib`) contains reusable files and utilities. This should be copy-paste-able between robot projects (relying on nothing outside the `lib` directory) and should be self contained. These files can be moved, added, or removed as needed.

## Github Pages Javadoc

A Github Action will automatically generate Javadocs for the robot project, which will live on the `javadocs` branch and be posted to the project's Github Pages site
