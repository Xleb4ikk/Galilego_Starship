## Орбиты и манёвры (Principia-style)

- **Горизонт предсказания в планировщике** (`FlightPlan.PredictionLengthSeconds` → `UniverseManager.PreviewTimeOffsetSeconds`) задаёт **только**: насколько далеко рисуется траектория манёвра и где стоит маркер конца отрезка. Положение небесных тел всегда по **текущему времени симуляции**.
- На **карте орбиты**, если есть `ManeuverEvaluator`, встроенная «сырая» линия `Ship_Trajectory` отключена — остаётся траектория манёвра.

Визуализация орбит/траекторий в проекте построена на `LineRenderer` и обновляется в реальном времени:

- **Лунные орбиты и траектория корабля**: `Scripts/Systems/UniverseManager.cs`
  - Отрисовка орбит включается в режиме `SpaceCameraMode.OrbitMap`.
  - Толщина линий адаптируется к зуму через `ResolveMoonOrbitLineWidth()` и `ResolveWorldLineWidthForPixels(...)`.
  - Для читаемости применяется градиент (хвост → голова/свечение).

- **Манёвры / будущая траектория**: `Scripts/Gameplay/ManeuverEvaluator.cs`
  - Траектория строится по сегментам между узлами `FlightPlan`.
  - Сегменты после манёвра рисуются пунктиром (`Shaders/DashedLine.shader`).
  - Толщина линий и размер маркера времени подстраиваются под текущий зум OrbitMap.

### Быстрый чек-лист если “что-то не видно”

- **Корабль не виден в ближней камере**:
  - `UniverseManager` теперь включает фактический слой `ShipVisualTransform` в `cullingMask` камер (даже если слой `Ship` не создан).

- **Траектории/орбиты не видны**:
  - Проверь, что включён режим `OrbitMap` и `showMoonOrbits / showShipTrajectory`.
  - Проверь наличие слоя `Trajectory` (желательно), иначе будет fallback на `Default`.

