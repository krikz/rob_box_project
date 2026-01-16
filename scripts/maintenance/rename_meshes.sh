#!/bin/bash
set -e
cd FROM_FUSION_360/meshes
[ -f "Крышка_1_1.stl" ] && mv "Крышка_1_1.stl" "body_cover.stl" && echo "✓ body_cover.stl"
[ -f "ЛеваяПара.stl" ] && mv "ЛеваяПара.stl" "left_assembly.stl" && echo "✓ left_assembly.stl"
[ -f "ПраваяПара_1.stl" ] && mv "ПраваяПара_1.stl" "right_assembly.stl" && echo "✓ right_assembly.stl"
[ -f "Коромысло.stl" ] && mv "Коромысло.stl" "rocker_left.stl" && echo "✓ rocker_left.stl"
[ -f "Коромысло(Mirror).stl" ] && mv "Коромысло(Mirror).stl" "rocker_right.stl" && echo "✓ rocker_right.stl"
[ -f "Колесо.stl" ] && mv "Колесо.stl" "wheel_front_left.stl" && echo "✓ wheel_front_left.stl"
[ -f "Колесо(Mirror).stl" ] && mv "Колесо(Mirror).stl" "wheel_front_right.stl" && echo "✓ wheel_front_right.stl"
[ -f "Колесо(Mirror)(Mirror).stl" ] && mv "Колесо(Mirror)(Mirror).stl" "wheel_rear_left.stl" && echo "✓ wheel_rear_left.stl"
[ -f "Колесо(Mirror) (1).stl" ] && mv "Колесо(Mirror) (1).stl" "wheel_rear_right.stl" && echo "✓ wheel_rear_right.stl"
[ -f "Зарядка_1.stl" ] && mv "Зарядка_1.stl" "charging_port.stl" && echo "✓ charging_port.stl"
[ -f "HORN.stl" ] && mv "HORN.stl" "horn_speaker.stl" && echo "✓ horn_speaker.stl"
[ -f "HEAD_LEAD_1.stl" ] && mv "HEAD_LEAD_1.stl" "head_connector.stl" && echo "✓ head_connector.stl"
echo "✅ Done!"
ls -lh
