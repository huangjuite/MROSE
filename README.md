# Radar Inertial Odometry

C++ library and Python package for Radar-Inertial Odometry, with ROS2 support.

## Dependencies

**C++ library:**
- CMake >= 3.22
- TBB
- Ceres-Solver 2.2
- yaml-cpp, fmt, spdlog
- [GTSAM 4.3a0](https://github.com/borglab/gtsam/tree/4.3a0)

**Python package** (additional):
- Python >= 3.12

## Building

### Manual

Install the dependencies above, then build GTSAM 4.3a0 and install it to your prefix (see `env/build_gtsam.sh`). Then:

**C++ library:**
```bash
cmake -S src -B build/cpp
cmake --build build/cpp --parallel
```

**Python package:**
```bash
uv sync --verbose
# or
pip install -e . --no-build-isolation
```

### Using pixi

[pixi](https://pixi.sh) resolves all dependencies automatically. Install it first:

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

Then install the environment and the Python package:

```bash
pixi install
pixi run install-python
```

Or build the standalone C++ library:

```bash
pixi install
pixi run build-cpp
```

## Project Structure

```
src/          C++ core library (factors, managers, velocity estimation)
python/       Python package (rpl_rio) + nanobind bindings
ros/          ROS2 package (launch files, CMakeLists, package.xml)
cfg/          Sensor configuration YAML files
env/          Build helper scripts (build_gtsam.sh)
```
