import cv2 as cv
import numpy as np
import json

# Загружаем изображение
image_path = "image3.png"  # Укажите путь к вашему изображению
frame = cv.imread(image_path)
if frame is None:
    print("Не удалось загрузить изображение. Проверьте путь к файлу.")
    exit()

# Начальные значения диапазона HSV (сохраненные диапазоны можно загрузить из файла)
try:
    with open("hsv_range.json", "r") as f:
        saved_values = json.load(f)
        lower_blue = np.array(saved_values["lower_blue"])
        upper_blue = np.array(saved_values["upper_blue"])
except FileNotFoundError:
    lower_blue = np.array([0, 50, 50])
    upper_blue = np.array([10, 255, 255])

def save_hsv_values():
    """Функция для сохранения текущего диапазона HSV в файл."""
    with open("hsv_range.json", "w") as f:
        json.dump({"lower_blue": lower_blue.tolist(), "upper_blue": upper_blue.tolist()}, f)
    print("Диапазон HSV сохранен в hsv_range.json")

print("Используйте клавиши для настройки диапазона HSV:")
print("W/S - увеличить/уменьшить нижнюю границу H")
print("E/D - увеличить/уменьшить верхнюю границу H")
print("R/F - увеличить/уменьшить нижнюю границу S")
print("T/G - увеличить/уменьшить верхнюю границу S")
print("Y/H - увеличить/уменьшить нижнюю границу V")
print("U/J - увеличить/уменьшить верхнюю границу V")
print("Нажмите 'P для сохранения значений диапазона в файл")
print("Нажмите 'Esc' для выхода")

while True:
    # Преобразуем изображение в HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Применяем маску с текущим диапазоном HSV
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    res = cv.bitwise_and(frame, frame, mask=mask)

    # Отображаем окна
    cv.imshow('Original Image', frame)
    cv.imshow('Mask', mask)
    cv.imshow('Result', res)

    # Обработка нажатий клавиш
    key = cv.waitKey(1) & 0xFF

    # Изменение нижних и верхних границ H, S и V
    if key == ord('w'):
        lower_blue[0] = min(lower_blue[0] + 1, 179)
    elif key == ord('s'):
        lower_blue[0] = max(lower_blue[0] - 1, 0)
    elif key == ord('e'):
        upper_blue[0] = min(upper_blue[0] + 1, 179)
    elif key == ord('d'):
        upper_blue[0] = max(upper_blue[0] - 1, 0)
    elif key == ord('r'):
        lower_blue[1] = min(lower_blue[1] + 1, 255)
    elif key == ord('f'):
        lower_blue[1] = max(lower_blue[1] - 1, 0)
    elif key == ord('t'):
        upper_blue[1] = min(upper_blue[1] + 1, 255)
    elif key == ord('g'):
        upper_blue[1] = max(upper_blue[1] - 1, 0)
    elif key == ord('y'):
        lower_blue[2] = min(lower_blue[2] + 1, 255)
    elif key == ord('h'):
        lower_blue[2] = max(lower_blue[2] - 1, 0)
    elif key == ord('u'):
        upper_blue[2] = min(upper_blue[2] + 1, 255)
    elif key == ord('j'):
        upper_blue[2] = max(upper_blue[2] - 1, 0)

    # Сохранение значений диапазона в файл
    elif key == ord('p'):
        save_hsv_values()

    # Выход по клавише ESC
    elif key == 27:
        break

cv.destroyAllWindows()
