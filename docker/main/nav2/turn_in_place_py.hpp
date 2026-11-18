// Мост к rob_box_teleop.turn_in_place.free_spin. Решение о курсе живёт в Python
// и проверяется pytest; здесь только вызов. false — модуля нет или функция
// отказала: Spin не слать, дерево идёт в FollowPath.
//
// points и path уже в base_footprint — поза (0,0,0), не map-yaw. Импорт
// пробуется снова, пока пакет не появился на PYTHONPATH (том образа nav2).
// После первого успеха модуль не перечитывается.

#pragma once

#include <Python.h>

#include <cmath>
#include <utility>
#include <vector>

namespace rob_box
{

inline bool free_heading_spin_rad(
  const std::vector<std::pair<double, double>> & points,
  const std::vector<std::pair<double, double>> & path,
  double clearance_m, double & spin_rad)
{
  static bool ready = false;
  if (!ready) {
    if (Py_IsInitialized() == 0) {
      Py_Initialize();
    }
    PyObject * module = PyImport_ImportModule("rob_box_teleop.turn_in_place");
    if (module == nullptr) {
      PyErr_Clear();
      return false;
    }
    Py_DECREF(module);
    ready = true;
  }

  const char * code =
    "from rob_box_teleop.proximity import Body, body_from_nav2_params\n"
    "from rob_box_teleop.turn_in_place import free_spin\n"
    "import yaml\n"
    "with open('/config/nav2/nav2_params.yaml', encoding='utf-8') as fh:\n"
    "    body = body_from_nav2_params(yaml.safe_load(fh))\n"
    "choice = free_spin(body, (0.0, 0.0, 0.0), points, path, float(clearance_m))\n"
    "spin = None if choice is None else float(choice)\n";

  auto as_list = [](const std::vector<std::pair<double, double>> & src) {
      PyObject * rows = PyList_New(static_cast<Py_ssize_t>(src.size()));
      for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(src.size()); ++i) {
        PyObject * row = PyTuple_Pack(
          2, PyFloat_FromDouble(src[static_cast<size_t>(i)].first),
          PyFloat_FromDouble(src[static_cast<size_t>(i)].second));
        PyList_SET_ITEM(rows, i, row);
      }
      return rows;
    };

  PyObject * globals = PyDict_New();
  PyObject * point_list = as_list(points);
  PyObject * path_list = as_list(path);
  PyDict_SetItemString(globals, "points", point_list);
  PyDict_SetItemString(globals, "path", path_list);
  PyDict_SetItemString(globals, "clearance_m", PyFloat_FromDouble(clearance_m));
  Py_DECREF(point_list);
  Py_DECREF(path_list);

  PyObject * locals = PyDict_New();
  PyObject * ran = PyRun_String(code, Py_file_input, globals, locals);
  bool ok = false;
  if (ran != nullptr) {
    PyObject * value = PyDict_GetItemString(locals, "spin");
    if (value != nullptr && value != Py_None && PyFloat_Check(value)) {
      spin_rad = PyFloat_AsDouble(value);
      ok = std::isfinite(spin_rad) && spin_rad != 0.0;
    }
    Py_DECREF(ran);
  } else {
    PyErr_Clear();
  }
  Py_DECREF(locals);
  Py_DECREF(globals);
  return ok;
}

}  // namespace rob_box
