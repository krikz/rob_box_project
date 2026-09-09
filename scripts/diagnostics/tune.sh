# 1. Временно увеличить (до перезагрузки)
sudo sysctl -w net.core.rmem_max=2147483647

# 2. Сделать настройку постоянной
echo 'net.core.rmem_max=2147483647' | sudo tee /etc/sysctl.d/99-cyclone.conf

# 3. Применить без перезагрузки
sudo sysctl --system