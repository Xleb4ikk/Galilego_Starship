# АНАЛИЗ И ИСПРАВЛЕНИЯ: Maneuver Planner / Trajectory Prediction

## НАЙДЕННЫЕ ПРОБЛЕМЫ

### Проблема #1: Разный timestep (КРИТИЧНО)
**TrajectoryPredictor** (пурпурная): `predictionStepSeconds = 2.0`
**ManeuverEvaluator** (зелёная): `predictionStepSeconds = 30.0`

Разная точность интеграции → разные орбиты даже при Δv=0.

### Проблема #2: framePos не обновляется при применении Δv
`framePos` вычислен в начале сегмента, а Δv применяется в конце.
Нужно обновлять `framePos` перед каждым применением Δv.

### Проблема #3: PreviewTimeOffsetSeconds влияет на начальную точку
`PreviewTimeOffsetSeconds` используется для визуального сдвига маркера,
но не должен влиять на начальную точку траектории.

### Проблема #4: CalculateWorldDeltaV получает мировую позицию
`CalculateWorldDeltaV(currentPos, ...)` вместо `CalculateWorldDeltaV(currentPos - framePos, ...)`

### Проблема #5: RefreshPredictionCache использует мировую позицию
Та же проблема — неправильные координаты для orbital basis.

## ИСПРАВЛЕНИЯ

1. Унифицировать timestep = 2.0 секунды для обоих рендереров
2. Обновлять framePos перед каждым применением Δv
3. Исправить CalculateWorldDeltaV для использования относительных координат
4. Исправить RefreshPredictionCache
5. Убедиться, что PreviewTimeOffsetSeconds не влияет на начальную точку
