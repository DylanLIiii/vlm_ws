
## 1. Workspace Layout

```
vlm_ws/
├── src/
│   ├── my_python_pkg/
│   ├── another_pkg/
├── build/
├── install/
└── log/
```

* Always create packages inside `src/` — keeps root clean ([docs.ros.org][1]).
* It’s fine to mix ament\_python and ament\_cmake packages, but **no nested packages** ([manual.ro47003.me.tudelft.nl][2]).

---

## 2. Package Creation

```bash
cd ~/vlm_ws/src
ros2 pkg create my_pkg --build-type ament_python --dependencies rclpy [--license Apache-2.0] [--node-name my_node]
```

* Use `ament_python` for pure Python packages ([manual.ro47003.me.tudelft.nl][2]).
* Add dependencies (`rclpy`, std\_msgs etc.) upfront to avoid missing deps ([get-help.theconstruct.ai][3]).
* `--node-name` auto-generates a sample node script.

---

## 3. Package Structure

```
my_pkg/
├── package.xml
├── setup.py
├── setup.cfg           # needed if you have console_scripts
├── resource/
│   └── my_pkg         # marker file
├── my_pkg/
│   ├── __init__.py
│   ├── node.py
│   └── library/
│       ├── _impl.py
│       └── __init__.py
└── test/
    ├── test_flake8.py
    └── test_pep257.py
```

* Use a clean modular Python package: split implementations into private `_*.py`, export only necessary API ([manual.ro47003.me.tudelft.nl][2], [ros2-tutorial.readthedocs.io][4]).
* Good folder structure with clear roles: `test/`, `resource/`, and nested libraries .

---

## 4. setup.py and setup.cfg

In `setup.py`, define:

* `packages=[‘my_pkg’, ‘my_pkg.library’]`
* `install_requires=['setuptools'] + external python deps (if any)`
* `entry_points={'console_scripts': ['my_node = my_pkg.node:main']}` ([theconstruct.ai][5]).

Ensure `setup.cfg` exists if you have executables, so ROS2 can detect them ([docs.ros.org][1]).

---

## 5. External Dependencies & rosdep

* Declare external dependencies in `package.xml` using `<depend>…</depend>` tags ([get-help.theconstruct.ai][3]).
* Install dependencies via rosdep from workspace root:

```bash
rosdep install -i --from-path src --rosdistro <distro> -y
```

([manual.ro47003.me.tudelft.nl][6])

---

## 6. Virtual Environments (Optional)

* Use `venv` to isolate Python environment: `python3 -m venv ~/venv && source ~/venv/bin/activate` ([ros2-tutorial.readthedocs.io][7]).
* If using, always build and run under the same Python interpreter to avoid mismatches ([automaticaddison.com][8]).
* Add `site-packages` to `PYTHONPATH` if needed so colcon can discover modules ([robotics.stackexchange.com][9]).

---

## 7. Build & Workflow

```bash
colcon build --symlink-install
```

* Use `--symlink-install` to auto-update changes in scripts without rebuilding ([roboticsunveiled.com][10]).
* After building:

  ```bash
  source /opt/ros/<distro>/setup.bash
  source ~/vlm_ws/install/local_setup.bash
  ```

Order matters: source underlay (ROS distro) first, then overlay (workspace) .

---

## 8. Node Implementation

* Implement nodes as classes inheriting `rclpy.node.Node`, not scripts ([roboticsunveiled.com][10]).
* Wrap execution entry with:

````python
def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
``` :contentReference[oaicite:38]{index=38}.

---

## 9. Testing & CI

- Include tests (unit, lint) in `test/` directory, e.g., pytest, flake8, pep257 :contentReference[oaicite:39]{index=39}.
- Run tests via `colcon test` in CI pipeline.

---

## 10. Documentation & Code Style

- Include a clear `README.md` with:
  - Package purpose & usage
  - Build & install instructions
  - Run examples and test instructions
  - Contribution and license guidelines :contentReference[oaicite:40]{index=40}.
- Add `CHANGELOG.md`, `CONTRIBUTING.md`, and proper license file.
- Insert copyright notice in headers and maintain code style.

---

## 11. Handling Custom Interfaces

- Python packages cannot generate interface types (msg/srv/action); those must reside in a separate ament_cmake package :contentReference[oaicite:41]{index=41}.

---

## 12. Suppressing Build Warnings

- To ignore setuptools install warnings add to your shell:

```bash
export PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"
``` :contentReference[oaicite:42]{index=42}.

---

## 🛠 TL;DR Workflow

1. `mkdir -p ~/vlm_ws/src && cd src`
2. `ros2 pkg create ... --build-type ament_python ...`
3. Populate code, add deps and console_scripts
4. `rosdep install -i --from-path src -y`
5. `colcon build --symlink-install`
6. Source before use
7. Use `ros2 run my_pkg my_node`
8. Add tests, docs, CI

