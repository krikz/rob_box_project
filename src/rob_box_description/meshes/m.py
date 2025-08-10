import numpy as np
from stl import mesh

def analyze_stl(file_path):
    # Загрузка STL-файла
    stl_mesh = mesh.Mesh.from_file(file_path)
    
    # Получение всех вершин
    vertices = np.concatenate((stl_mesh.v0, stl_mesh.v1, stl_mesh.v2))
    
    # Вычисление геометрического центра
    center = np.mean(vertices, axis=0)
    print(f"Геометрический центр: {center}")
    
    # Определение ориентации (направления осей)
    normals = stl_mesh.normals
    average_normal = np.mean(normals, axis=0)  # Средняя нормаль
    average_normal /= np.linalg.norm(average_normal)  # Нормализация
    
    print(f"Средняя нормаль (ориентация): {average_normal}")
    
    # Вычисление углов поворота (RPY)
    # Предполагаем, что колесо должно быть выровнено так, чтобы его ось вращения была вдоль оси Y
    rpy = calculate_rpy(average_normal)
    print(f"Углы поворота (RPY): {rpy}")
    
    return center, rpy

def calculate_rpy(normal):
    # Вычисляем углы поворота (RPY) на основе нормали
    roll = np.arctan2(normal[1], normal[2])
    pitch = np.arctan2(-normal[0], np.sqrt(normal[1]**2 + normal[2]**2))
    yaw = 0  # Предполагаем, что колесо не повернуто вокруг своей оси
    return roll, pitch, yaw

if __name__ == "__main__":
    # Путь к STL-файлу колеса
    stl_file = "D:/PROJECTS/rob_box_project/src/rob_box_description/meshes/wheel.stl"
    
    # Анализ модели
    center, rpy = analyze_stl(stl_file)
    
    print("\nРезультат:")
    print(f"Origin (xyz): {center}")
    print(f"RPY (radians): {rpy}")
    print(f"RPY (degrees): {[np.degrees(angle) for angle in rpy]}")