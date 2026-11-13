# MROSE: Multi-Radar Inertial Odometry

C++ library and Python package for radar-inertial odometry.

## Dependencies

**C++ library:**
- CMake >= 3.22 and a C++17 compiler
- Eigen 3.4
- Boost
- TBB
- Ceres-Solver 2.2
- yaml-cpp, fmt, spdlog
- [GTSAM 4.3a0](https://github.com/borglab/gtsam/tree/4.3a0)

**Python package** (additional):
- Python >= 3.12
- nanobind, scikit-build-core
- numpy, pyyaml

## Building

### Using pixi (recommended)

[pixi](https://pixi.sh) resolves all C++ and Python dependencies automatically. The project is pinned to the GTSAM 4.3a0 tag, which pixi builds from source using the recipe in `third_party/gtsam/`. This happens once, on the first `pixi install`, and takes several minutes; later installs reuse the cached package.

Install pixi:

```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

Then set up everything with one command:

```bash
pixi install
```

This installs the dependencies, builds GTSAM, and compiles and installs the `mrose` Python package (C++ core and bindings) in editable mode. Run things inside the environment with `pixi run`, for example `pixi run python your_script.py`; `pixi run` also performs the install if it has not been done yet.

After changing C++ code, rebuild with:

```bash
pixi reinstall mrose
```

### Manual

Install the dependencies above. GTSAM 4.3a0 must be built from source and installed to your prefix; the CMake flags we use are listed in `third_party/gtsam/recipe.yaml`. Then:

**C++ library:**
```bash
cmake -S src -B build/cpp -DCMAKE_BUILD_TYPE=Release
cmake --build build/cpp --parallel
```

**Python package:**
```bash
uv sync --verbose
# or
pip install -e . --no-build-isolation
```

## Usage

```python
import mrose

manager = mrose.DIOManager.from_config("cfg/config_hugin_dual.yaml")
```

## Project Structure

```
src/          C++ core library (factors, managers, velocity estimation)
python/       Python package (mrose) and nanobind bindings
cfg/          Sensor configuration YAML files
third_party/  Build recipes for dependencies (gtsam/recipe.yaml, built by pixi)
```

## Citation

If you use this work, please cite:

```bibtex
@inproceedings{huang2024multi,
  title={Multi-radar inertial odometry for 3d state estimation using mmwave imaging radar},
  author={Huang, Jui-Te and Xu, Ruoyang and Hinduja, Akshay and Kaess, Michael},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={12006--12012},
  year={2024},
  organization={IEEE}
}
```

## License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE) for the full text.
