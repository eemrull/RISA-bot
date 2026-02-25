# RISA-Bot Scripts & Tools

This folder contains the automated scripts designed to effortlessly set up and configure the RISA-bot software stack from scratch on a new or existing robot.

## Available Scripts

### 1. `install.sh`

The primary setup wizard. Running this script will handle laying down the entire repository's architecture without needing manual copying or installation of third-party libraries.

**What it does:**

- Verifies workspace structure (`~/risabotcar_ws/src/RISA-bot`)
- Auto-generates `udev` hardware binding rules
- Installs all relevant ROS 2 `apt` dependencies and pip libraries like `Rosmaster_Lib`
- Builds the `YDLidar-SDK` natively
- Initiates `colcon build` to compile the monolithic robot codebase
- Generates `~/.bash_aliases` and links your `~/.bashrc` automatically

**Usage:**

```bash
cd ~/risabotcar_ws
bash tools/install.sh
```

---

### 2. `install_bashalias.sh`

A modular, lightweight script to configure your terminal environment without rebuilding any C++ code. This is called automatically by `install.sh`, but can be run independently if you wish to ONLY update your aliases.

**What it does:**

- Completely regenerates the `~/.bash_aliases` file with quick-commands like `cb`, `astra`, `run_risabot`, etc.
- Appends the necessary `setup.bash` sourcing definitions to your `~/.bashrc`
- Injects the `run_risabot`, `run_trisabot`, and `fix_astra` bash functions into your profile

**Usage:**

```bash
cd ~/risabotcar_ws
bash tools/install_bashalias.sh
```

---

### 3. `setup_autostart.sh`

A utility script used to place the robot into an automatic run state upon boot.

**What it does:**

- Copies the `risabot_autostart.desktop` file into `~/.config/autostart/`
- When the robot boots into its Ubuntu desktop environment, this desktop file launches a new terminal executing `run_risabot` to bring the robot fully online hands-free.

**Usage:**

```bash
cd ~/risabotcar_ws
sudo bash tools/setup_autostart.sh
```
