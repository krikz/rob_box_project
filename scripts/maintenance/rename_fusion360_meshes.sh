#!/bin/bash
# rename_fusion360_meshes.sh
# Переименование mesh файлов из Fusion 360 экспорта в читаемые английские названия

set -e

MESH_DIR="FROM_FUSION_360/meshes"

echo "🔧 Renaming Fusion 360 mesh files to English..."
echo ""

cd "$MESH_DIR"

# Структурированные категории:
# 1. Chassis/Body
# 2. Wheels (колеса)
# 3. Rockers (коромысла)
# 4. Accessories (аксессуары)

echo "📦 Chassis components..."
# base_link.stl - уже на английском ✓
# Крышка = Cover/Lid
[ -f "Крышка_1_1.stl" ] && mv "Крышка_1_1.stl" "body_cover.stl"
echo "  ✓ Крышка_1_1.stl → body_cover.stl"

# ЛеваяПара = Left Pair (assembly)
[ -f "ЛеваяПара.stl" ] && mv "ЛеваяПара.stl" "left_assembly.stl"
echo "  ✓ ЛеваяПара.stl → left_assembly.stl"

# ПраваяПара = Right Pair (assembly)
[ -f "ПраваяПара_1.stl" ] && mv "ПраваяПара_1.stl" "right_assembly.stl"
echo "  ✓ ПраваяПара_1.stl → right_assembly.stl"

echo ""
echo "🔷 Rockers (suspension)..."
# Коромысло = Rocker
[ -f "Коромысло.stl" ] && mv "Коромысло.stl" "rocker_left.stl"
echo "  ✓ Коромысло.stl → rocker_left.stl"

[ -f "Коромысло(Mirror).stl" ] && mv "Коромысло(Mirror).stl" "rocker_right.stl"
echo "  ✓ Коромысло(Mirror).stl → rocker_right.stl"

echo ""
echo "⚙️ Wheels..."
# Колесо = Wheel
[ -f "Колесо.stl" ] && mv "Колесо.stl" "wheel_front_left.stl"
echo "  ✓ Колесо.stl → wheel_front_left.stl"

[ -f "Колесо(Mirror).stl" ] && mv "Колесо(Mirror).stl" "wheel_front_right.stl"
echo "  ✓ Колесо(Mirror).stl → wheel_front_right.stl"

[ -f "Колесо(Mirror)(Mirror).stl" ] && mv "Колесо(Mirror)(Mirror).stl" "wheel_rear_left.stl"
echo "  ✓ Колесо(Mirror)(Mirror).stl → wheel_rear_left.stl"

[ -f "Колесо(Mirror) (1).stl" ] && mv "Колесо(Mirror) (1).stl" "wheel_rear_right.stl"
echo "  ✓ Колесо(Mirror) (1).stl → wheel_rear_right.stl"

echo ""
echo "🔌 Accessories..."
# Зарядка = Charging port
[ -f "Зарядка_1.stl" ] && mv "Зарядка_1.stl" "charging_port.stl"
echo "  ✓ Зарядка_1.stl → charging_port.stl"

# HORN - speaker/horn (уже английское)
[ -f "HORN.stl" ] && mv "HORN.stl" "horn_speaker.stl"
echo "  ✓ HORN.stl → horn_speaker.stl"

# HEAD_LEAD - cable/connector
[ -f "HEAD_LEAD_1.stl" ] && mv "HEAD_LEAD_1.stl" "head_connector.stl"
echo "  ✓ HEAD_LEAD_1.stl → head_connector.stl"

# Component10, Component16 - нужно понять что это
# Оставим как есть пока не разберемся
echo ""
echo "❓ Unknown components (keeping original names)..."
echo "  ⚠ Component10_1_1.stl - needs identification"
echo "  ⚠ Component16_1.stl - needs identification"

echo ""
echo "✅ Mesh files renamed successfully!"
echo ""
echo "📊 Final structure:"
ls -lh

cd ../..
