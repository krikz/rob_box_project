"""Парсер адреса падения controller_server (#5180). Роботу не нужен.

Четыре случая: адрес внутри librclcpp.so, внутри libc.so.6, адрес вне всех
строк maps, пустой ввод. Плюс: shell-сборщик не открывает сеть и отказывает
на любом аргументе, кроме print.
"""

import subprocess
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
LOCATE = REPO / "scripts" / "diagnostics" / "nav2_sigill_locate.py"
CAPTURE = REPO / "scripts" / "diagnostics" / "nav2_sigill_capture.sh"

sys.path.insert(0, str(REPO / "scripts" / "diagnostics"))
import nav2_sigill_locate as locate  # noqa: E402

MAPS = """\
ffff80000000-ffff80020000 r--p 00000000 fe:01 1 /usr/lib/aarch64-linux-gnu/libc.so.6
ffff80020000-ffff80180000 r-xp 00020000 fe:01 1 /usr/lib/aarch64-linux-gnu/libc.so.6
ffff80180000-ffff801c0000 r--p 00180000 fe:01 1 /usr/lib/aarch64-linux-gnu/libc.so.6
ffff8a1c0000-ffff8a1c2000 r--p 00000000 fe:01 2 /opt/ros/humble/lib/librclcpp.so
ffff8a1c2000-ffff8a1e4000 r-xp 00020000 fe:01 2 /opt/ros/humble/lib/librclcpp.so
ffff8a1e4000-ffff8a1e8000 r--p 00240000 fe:01 2 /opt/ros/humble/lib/librclcpp.so
"""

DMESG_RCLCPP = (
    "traps: controller_server[4321] illegal instruction " "ip:ffff8a1c2f40 sp:ffffc0d0 error:0 in librclcpp.so\n"
)
DMESG_LIBC = (
    "traps: controller_server[4321] general protection fault " "ip:ffff80021000 sp:ffffc0d0 error:0 in libc.so.6\n"
)


class LocateTests(unittest.TestCase):
    def test_address_inside_librclcpp(self):
        result = locate.resolve(DMESG_RCLCPP, MAPS)
        self.assertEqual(result["signal"], "SIGILL")
        self.assertEqual(result["fault"], "illegal instruction")
        self.assertEqual(result["pid"], 4321)
        self.assertEqual(result["module"], "librclcpp.so")
        self.assertTrue(result["in_module"])
        # r-xp начинается на ffff8a1c2000, ip = ffff8a1c2f40 → смещение 0xf40
        self.assertEqual(result["ip"], 0xFFFF8A1C2F40)
        self.assertEqual(result["offset"], 0xF40)
        self.assertEqual(result["file_offset"], 0x20000 + 0xF40)
        self.assertIn("librclcpp.so+0xf40", locate.format_line(result))

    def test_address_inside_libc(self):
        result = locate.resolve(DMESG_LIBC, MAPS)
        self.assertEqual(result["signal"], "SIGSEGV")
        self.assertEqual(result["module"], "libc.so.6")
        # r-xp libc: start ffff80020000, ip ffff80021000 → 0x1000
        self.assertEqual(result["offset"], 0x1000)
        self.assertNotEqual(result["module"], "librclcpp.so")

    def test_address_outside_modules(self):
        dmesg = "traps: controller_server[99] illegal instruction ip:ffff90000000\n"
        result = locate.resolve(dmesg, MAPS)
        self.assertFalse(result["in_module"])
        self.assertIsNone(result["module"])
        self.assertIsNone(result["offset"])
        self.assertIsNotNone(result["nearest_module"])
        self.assertGreater(result["gap_to_nearest"], 0)
        line = locate.format_line(result)
        self.assertIn("outside modules", line)
        self.assertIn("nearest", line)

    def test_empty_input(self):
        result = locate.resolve("", "")
        self.assertIsNone(result["ip"])
        self.assertIsNone(result["signal"])
        self.assertIsNone(result["module"])
        self.assertFalse(result["in_module"])
        self.assertIsNone(result["gap_to_nearest"])
        self.assertEqual(locate.format_line(result), "none none no module")

    def test_empty_maps_keeps_address(self):
        result = locate.resolve(DMESG_RCLCPP, "")
        self.assertEqual(result["ip"], 0xFFFF8A1C2F40)
        self.assertEqual(result["signal"], "SIGILL")
        self.assertFalse(result["in_module"])
        self.assertIsNone(result["nearest_module"])

    def test_prefers_executable_mapping(self):
        # Адрес на границе r--p|r-xp не должен сесть в неисполняемую страницу,
        # если исполняемая начинается ровно с него.
        dmesg = "traps: controller_server[1] illegal instruction ip:ffff8a1c2000\n"
        result = locate.resolve(dmesg, MAPS)
        self.assertEqual(result["offset"], 0)
        self.assertEqual(result["module"], "librclcpp.so")
        self.assertEqual(result["file_offset"], 0x20000)


class CaptureScriptTests(unittest.TestCase):
    def test_print_does_not_touch_network(self):
        proc = subprocess.run(
            ["bash", str(CAPTURE), "print"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("dmesg", proc.stdout)
        self.assertIn("/proc/", proc.stdout)
        self.assertIn("coredumpctl", proc.stdout)
        self.assertIn("core_pattern", proc.stdout)
        self.assertIn("librclcpp.so", proc.stdout)
        # Скрипт печатает команды, но сам не вызывает ssh и не называет узел.
        self.assertNotIn("ssh ", proc.stdout)
        self.assertNotIn("10.1.1.", proc.stdout)

    def test_refuses_anything_but_print(self):
        proc = subprocess.run(
            ["bash", str(CAPTURE), "run"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(proc.returncode, 2)
        self.assertIn("не открывает", proc.stderr)


if __name__ == "__main__":
    unittest.main()
