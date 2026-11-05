# Ubuntu 24.04 / ROS 2 Jazzy migration

The robot runtime targets Ubuntu 24.04 LTS, ROS 2 Jazzy, and the system Python
3.12. Ubuntu's release upgrader does not migrate Humble or old Python environments.

Install ROS 2 Jazzy and its dependencies for Ubuntu 24.04 using the official
[Debian package instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Review apt's proposed changes when resolving dependencies left over from
Humble/Rolling. The old `scripts/setup_jazzy.sh` installer is no longer present.

Back up `~/.bashrc`, then replace its Humble source command with
`source "$HOME/sim2real/scripts/source_ros.sh"`. This loads Jazzy without
activating a virtual environment; Python uses its default packages. Remove old
workspace overlays from aliases unless those workspaces have also been rebuilt
with Jazzy. Open a new SSH session so Humble's Python and library paths are no
longer inherited.

Before the first build, move the old `build`, `install`, and `log` directories
to a backup outside the workspace. Do not reuse Humble CMake caches or generated
Python 3.10 files. Then run:

```bash
source scripts/build.sh
```

`source_ros.sh` refuses to mix a previously sourced ROS distribution with Jazzy.
`source_ros.sh` uses the default Python environment and does not activate
`.venv-jazzy`. When reloaded in a terminal still using that old environment, it
deactivates it. Other explicitly activated virtual environments are left alone.
For user `qr1`, PyTorch CPU and its additional dependencies are installed in
`~/.local/lib/python3.12/site-packages`, which `/usr/bin/python3` loads by default.
ROS and NumPy continue to come from apt-installed packages. This default applies
to `qr1`; other users and services need their own dependency configuration.

The existing default PyTorch `2.14.1+cpu` was verified against the old venv's
`2.9.1+cpu`: the deployed `models/policy.pt` produced identical outputs for 64
deterministic test inputs of shape `(1, 5, 45)`. Keep the old `.venv-jazzy`
directory as a fallback; it is no longer needed for normal bringup. Verify policy
loading and inference after changing PyTorch or the deployed model. Model
retraining is not required by this OS migration.

On this PC, `~/.bashrc` already loads Jazzy and the workspace for interactive
terminals. Open a new terminal or run `source ~/.bashrc` to apply this default.

Verify package discovery, launch arguments (`--show-args`), offline tests, and
policy inference before hardware operation. Never use a motor-driving launch as
an installation smoke test. Configure remote ROS clients for Jazzy as well.
