# Ubuntu 24.04 / ROS 2 Jazzy migration

The robot runtime targets Ubuntu 24.04 LTS, ROS 2 Jazzy, and the system Python
3.12. Ubuntu's release upgrader does not migrate Humble or old Python environments.

Install dependencies over SSH (the script prompts for sudo and starts no nodes):

```bash
bash scripts/setup_jazzy.sh
```

The installer backs up the existing ROS apt source and switches its suite to
`noble`. Other third-party repositories are not changed. If apt reports errors,
resolve those before proceeding. Review apt's proposed changes: resolving Noble
dependencies can remove incompatible Humble/Rolling packages. The installer does
not explicitly purge either old distribution.

Back up `~/.bashrc`, then replace its Humble source command with
`source "$HOME/sim2real/scripts/source_ros.sh"`. This loads both Jazzy and the
Python environment. Remove old workspace overlays from aliases unless
those workspaces have also been rebuilt with Jazzy. Open a new SSH session so
Humble's Python and library paths are no longer inherited.

Before the first build, move the old `build`, `install`, and `log` directories
to a backup outside the workspace. Do not reuse Humble CMake caches or generated
Python 3.10 files. Then run:

```bash
source scripts/build.sh
```

`source_ros.sh` refuses to mix a previously sourced ROS distribution with Jazzy.
If you use a Python virtual environment, recreate it with Python 3.12 and
`--system-site-packages` to access the apt-installed ROS Python modules. The
installer creates `.venv-jazzy` with system site packages and installs PyTorch
2.9.1 from the official CPU wheel index. `source_ros.sh` activates it. Verify
policy loading and inference before starting the controller. Model retraining is
not required by this OS migration, but TorchScript compatibility must be checked.

Verify package discovery, launch arguments (`--show-args`), offline tests, and
policy inference before hardware operation. Never use a motor-driving launch as
an installation smoke test. Configure remote ROS clients for Jazzy as well.
