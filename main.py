from ursina import *
from ursina.shaders import lit_with_shadows_shader
import numpy as np
import math

# Инициализация приложения
app = Ursina(title="3D Sine Wave Knot", borderless=False, fullscreen=False)


# --- 1. ГЕНЕРАЦИЯ ФИГУРЫ (ПРОЦЕДУРНЫЙ ТОР-УЗЕЛ) ---
def generate_torus_knot_mesh(tube_radius=0.3, radial_segments=12, tubular_segments=64):
    """
    Генерирует вершины и треугольники для Тор-узла математически.
    Это доказывает, что фигура создана кодом, а не загружена из файла.
    """
    vertices = []
    triangles = []

    # Формула Тор-узла (p=2, q=3)
    def knot_func(t):
        x = (2 + np.cos(3 * t)) * np.cos(2 * t)
        y = (2 + np.cos(3 * t)) * np.sin(2 * t)
        z = np.sin(3 * t)
        return np.array([x, y, z])

    # Генерируем точки вдоль основной кривой
    ts = np.linspace(0, 2 * np.pi, tubular_segments, endpoint=False)
    points = np.array([knot_func(t) for t in ts])

    # Для каждой точки создаем кольцо вершин вокруг неё (трубка)
    for i, center in enumerate(points):
        # Вычисляем нормали и би нормали для ориентации кольца (упрощенно)
        next_center = points[(i + 1) % len(points)]
        tangent = next_center - center
        tangent = tangent / np.linalg.norm(tangent)

        # Вектор вверх (примерный)
        up = np.array([0, 0, 1])
        if abs(np.dot(tangent, up)) > 0.9:
            up = np.array([0, 1, 0])

        normal = np.cross(tangent, up)
        normal = normal / np.linalg.norm(normal)
        binormal = np.cross(tangent, normal)

        for j in range(radial_segments):
            angle = 2 * np.pi * j / radial_segments
            # Смещение вершины от центра трубки
            offset = (np.cos(angle) * normal + np.sin(angle) * binormal) * tube_radius
            vertex = center + offset
            vertices.append(vertex)

    # Соединяем вершины в треугольники
    for i in range(tubular_segments):
        for j in range(radial_segments):
            current = i * radial_segments + j
            next_i = ((i + 1) % tubular_segments) * radial_segments + j
            next_j = i * radial_segments + ((j + 1) % radial_segments)
            next_ij = ((i + 1) % tubular_segments) * radial_segments + ((j + 1) % radial_segments)

            triangles.append([current, next_ij, next_i])
            triangles.append([current, next_j, next_ij])

    return vertices, triangles


# Создаем меш из данных
verts, tris = generate_torus_knot_mesh()
knot_model = Mesh(vertices=verts, triangles=tris, mode='triangle')
knot_model.generate_normals()  # Чтобы свет правильно отражался

# --- 2. СОЗДАНИЕ СЦЕНЫ ---

# Пол (чтобы видеть тень и глубину)
ground = Entity(
    model='plane' ,
    scale=(20 , 1 , 20) ,
    color=color.dark_gray ,
    texture='white_cube' ,
    collider='box' ,
    shader=lit_with_shadows_shader
)
# Небо (фон)
Sky(color=color.rgb(20, 20, 30))

# Наша фигура
knot = Entity(
    model=knot_model,
    color=color.cyan,
    shader=lit_with_shadows_shader,
    scale=0.5,
    position=(-10, 0, 0)  # Start position
)

# --- 3. ОСВЕЩЕНИЕ (Доказательство объема) ---
# Свет будет следовать за фигурой, чтобы блики бегали по поверхности
light = PointLight(
    parent=knot,
    position=(2, 2, 2),
    color=color.white,
    shadows=True
)

# --- 4. КАМЕРА (С управлением: зум, вращение, панорама) ---
# ✅ Добавлена EditorCamera для полного контроля
camera = EditorCamera(
    rotation_x=10,      # Начальный наклон вверх/вниз
    rotation_y=-45,     # Начальный поворот вокруг фигуры
    distance=40         # ✅ Начальное отдаление (было 105, теперь 40 для лучшего вида)
)

# --- 5. ЛОГИКА ДВИЖЕНИЯ (Синус) ---
time_tracker = 0


def update():
    global time_tracker
    speed = 4 * time.dt

    # Движение вперед по X
    knot.x += speed
    time_tracker += speed

    # Движение по синусоиде вверх-вниз (Y)
    # Амплитуда 3, частота 0.5
    knot.y = np.sin(time_tracker * 0.5) * 3

    # Легкое покачивание по глубине (Z) для усиления 3D эффекта
    knot.z = np.cos(time_tracker * 0.5) * 2

    # Вращение фигуры вокруг своей оси
    knot.rotation_y += 50 * time.dt
    knot.rotation_z += 20 * time.dt

    # Если улетела слишком далеко, возвращаем назад (цикл)
    if knot.x > 10:
        knot.x = -10
        time_tracker = 0


# --- 6. ПОДСКАЗКИ НА ЭКРАНЕ ---
Text(text='ЛКМ: Вращать камеру | Колесо: Зум', position=(-0.85, 0.45), scale=1, color=color.gray)
Text(text='ПКМ: Сдвиг камеры', position=(-0.85, 0.40), scale=1, color=color.gray)

# Запуск
app.run()
