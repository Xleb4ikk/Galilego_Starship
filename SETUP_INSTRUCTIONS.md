# Инструкция по настройке планировщика манёвров

## Быстрый старт

### 1. Добавление компонентов на сцену

Откройте вашу сцену в Unity и добавьте следующие компоненты:

#### A. OrbitAnalyzer (обязательно)
```
1. Создайте пустой GameObject: GameObject → Create Empty
2. Переименуйте в "OrbitAnalyzer"
3. Добавьте компонент: Add Component → Galilego.Gameplay → OrbitAnalyzer
4. В инспекторе привяжите UniverseManager (перетащите из сцены)
```

#### B. OrbitAnalysisUI (опционально, для UI анализа)
```
1. Создайте пустой GameObject: GameObject → Create Empty
2. Переименуйте в "OrbitAnalysisUI"
3. Добавьте компонент: Add Component → Galilego.Gameplay → OrbitAnalysisUI
4. В инспекторе привяжите:
   - OrbitAnalyzer (созданный выше)
   - UniverseManager (из сцены)
   - FlightPlanUI (если есть на сцене)
```

### 2. Проверка существующих компонентов

Убедитесь что на сцене уже есть:
- ✅ UniverseManager
- ✅ ManeuverEvaluator
- ✅ FlightPlanUI

Если их нет, добавьте их согласно документации проекта.

### 3. Горячие клавиши

После настройки будут доступны:
- **N** — открыть/закрыть планировщик манёвров (FlightPlanUI)
- **O** — открыть/закрыть анализ орбит (OrbitAnalysisUI)

## Использование в коде

### Пример 1: Создание манёвра с расчётом топлива

```csharp
using Galilego.Gameplay;
using Galilego.Physics;

// Создаём манёвр
var node = new ManeuverNode(
    time: universeManager.SimulationTimeSeconds + 600,
    prograde: 500,  // 500 м/с вперёд
    normal: 0,
    radial: 0
);

// Задаём параметры двигателя
node.Engine = new EngineParameters
{
    ThrustNewtons = 10000,           // 10 кН тяги
    SpecificImpulseSeconds = 300,    // 300 с удельный импульс
    InitialMassKg = 5000             // 5 тонн начальная масса
};

// Получаем расчёт
var calc = node.GetCalculation();
Debug.Log($"Конечная масса: {calc.FinalMassKg:F0} кг");
Debug.Log($"Длительность: {calc.DurationSeconds:F1} с");
Debug.Log($"Расход топлива: {calc.MassFlowRate:F2} кг/с");

// Добавляем в план
var flightPlan = maneuverEvaluator.GetFlightPlan();
var result = flightPlan.Insert(node, 0);

if (!result.IsOk)
{
    Debug.LogError($"Ошибка добавления манёвра: {result.Message}");
}
```

### Пример 2: Анализ орбиты

```csharp
using Galilego.Gameplay;
using Galilego.Physics;

// Получаем анализатор
OrbitAnalyzer analyzer = FindAnyObjectByType<OrbitAnalyzer>();

// Анализируем текущую орбиту вокруг Юпитера
OrbitAnalysisResult result = analyzer.AnalyzeShipOrbit(ReferenceFrameTarget.Jupiter);

if (result.IsValid)
{
    Debug.Log($"=== Орбита вокруг {result.TargetName} ===");
    Debug.Log($"Перицентр: {result.Elements.PeriapsisDistance / 1000:F0} км");
    Debug.Log($"Апоцентр: {result.Elements.ApoapsisDistance / 1000:F0} км");
    Debug.Log($"Период: {result.Elements.OrbitalPeriodSeconds / 3600:F2} ч");
    Debug.Log($"Эксцентриситет: {result.Elements.Eccentricity:F4}");
    Debug.Log($"Наклонение: {result.Elements.InclinationDegrees:F2}°");
    
    // Проверка стабильности
    if (result.WillImpact)
    {
        Debug.LogWarning($"⚠ Столкновение через {result.ImpactTime:F0} секунд!");
    }
    else if (result.WillEscape)
    {
        Debug.LogWarning($"⚠ Выход из SOI через {result.EscapeTime:F0} секунд!");
    }
    else if (result.IsStable)
    {
        Debug.Log("✓ Орбита стабильна");
    }
    
    // Время до апсид
    Debug.Log($"До перицентра: {result.TimeToPeriapsis:F0} с");
    Debug.Log($"До апоцентра: {result.TimeToApoapsis:F0} с");
}
```

### Пример 3: Анализ орбиты после манёвра

```csharp
// Получаем текущее состояние
Vector3d shipPos = universeManager.ShipBody.Position;
Vector3d shipVel = universeManager.ShipBody.Velocity;

// Создаём манёвр
var node = new ManeuverNode(time: 0, prograde: 500, normal: 0, radial: 0);

// Вычисляем Δv в мировых координатах
if (universeManager.TryGetShipRelativeState(
    ReferenceFrameTarget.Jupiter,
    out _,
    out Vector3d relativePos,
    out Vector3d relativeVel,
    out _,
    out _,
    out _))
{
    Vector3d deltaV = FlightPlan.CalculateWorldDeltaV(relativePos, relativeVel, node);
    
    // Анализируем новую орбиту
    OrbitAnalysisResult afterManeuver = analyzer.AnalyzeOrbitAfterManeuver(
        ReferenceFrameTarget.Jupiter,
        shipPos,
        shipVel,
        deltaV
    );
    
    if (afterManeuver.IsValid)
    {
        Debug.Log("=== После манёвра ===");
        Debug.Log($"Новый перицентр: {afterManeuver.Elements.PeriapsisDistance / 1000:F0} км");
        Debug.Log($"Новый апоцентр: {afterManeuver.Elements.ApoapsisDistance / 1000:F0} км");
    }
}
```

### Пример 4: Расчёт перехода Гоманна

```csharp
using Galilego.Gameplay;

// Параметры орбит
double r1 = 6371e3 + 200e3;  // 200 км над Землёй
double r2 = 6371e3 + 400e3;  // 400 км над Землёй
double mu = 3.986e14;        // μ Земли (м³/с²)

// Вычисляем требуемый Δv
double totalDeltaV = OrbitUtilities.ComputeHohmannTransferDeltaV(r1, r2, mu);
Debug.Log($"Требуемый Δv для перехода: {totalDeltaV:F1} м/с");

// Вычисляем фазовый угол
double phaseAngle = OrbitUtilities.ComputeHohmannPhaseAngle(r1, r2, mu);
Debug.Log($"Фазовый угол: {phaseAngle * 180 / Math.PI:F1}°");

// Вычисляем Δv для круглизации на целевой орбите
double a_transfer = (r1 + r2) / 2.0;
double circularizationDV = OrbitUtilities.ComputeCircularizationDeltaV(r2, a_transfer, mu);
Debug.Log($"Δv для круглизации: {circularizationDV:F1} м/с");
```

### Пример 5: Управление планом полёта

```csharp
var flightPlan = maneuverEvaluator.GetFlightPlan();

// Вставка манёвра
var node1 = new ManeuverNode(time: 100, prograde: 200);
var result = flightPlan.Insert(node1, 0);

if (result.IsOk)
{
    Debug.Log("Манёвр добавлен");
}
else
{
    Debug.LogError($"Ошибка: {result.Status} - {result.Message}");
}

// Замена манёвра
var node2 = new ManeuverNode(time: 100, prograde: 300);
result = flightPlan.Replace(node2, 0);

// Удаление манёвра
result = flightPlan.Remove(0);

// Получение манёвра
if (flightPlan.ManeuverCount > 0)
{
    var node = flightPlan.GetManeuver(0);
    Debug.Log($"Манёвр #{1}: Δv = {node.TotalDeltaV:F1} м/с");
}
```

## Проверка работоспособности

### Тест 1: Базовая функциональность
1. Запустите игру
2. Нажмите **N** — должно открыться окно планировщика
3. Нажмите **+** — должен добавиться манёвр
4. Измените Δv слайдерами — траектория должна обновиться
5. Нажмите **O** — должно открыться окно анализа орбит

### Тест 2: Анализ орбиты
1. Откройте окно анализа орбит (клавиша **O**)
2. Выберите целевое тело (Jupiter, Io, Europa и т.д.)
3. Проверьте что отображаются:
   - Periapsis / Apoapsis
   - Period
   - Eccentricity
   - Inclination
4. Добавьте манёвр в планировщике
5. Проверьте что в разделе "AFTER MANEUVER" показывается новая орбита

### Тест 3: Расчёт топлива
```csharp
// В консоли Unity выполните:
var node = new ManeuverNode(0, 500, 0, 0);
node.Engine = new EngineParameters { 
    ThrustNewtons = 10000, 
    SpecificImpulseSeconds = 300, 
    InitialMassKg = 5000 
};
var calc = node.GetCalculation();
Debug.Log($"Final mass: {calc.FinalMassKg} kg, Duration: {calc.DurationSeconds} s");
```

Ожидаемый результат: конечная масса ~4150 кг, длительность ~17 секунд

## Устранение проблем

### Проблема: Окна не открываются
**Решение:**
- Проверьте что компоненты добавлены на сцену
- Проверьте что в инспекторе привязаны все ссылки
- Проверьте консоль на наличие ошибок

### Проблема: Анализ орбиты показывает "Invalid"
**Решение:**
- Убедитесь что UniverseManager инициализирован
- Проверьте что корабль находится в пределах SOI выбранного тела
- Проверьте что у корабля есть валидная орбита (не на поверхности)

### Проблема: Траектория не обновляется после изменения манёвра
**Решение:**
- Проверьте что ManeuverEvaluator привязан к FlightPlanUI
- Проверьте что ManeuverEvaluator.MarkAsDirty() вызывается при изменениях
- Увеличьте debounceTime в ManeuverEvaluator если обновления слишком частые

### Проблема: Ошибки компиляции
**Решение:**
- Убедитесь что все файлы находятся в правильных папках:
  - ManeuverTypes.cs → Scripts/Gameplay/
  - ManeuverUtilities.cs → Scripts/Gameplay/
  - OrbitUtilities.cs → Scripts/Gameplay/
  - OrbitAnalyzer.cs → Scripts/Gameplay/
  - OrbitAnalysisUI.cs → Scripts/UI/
- Проверьте что namespace правильный: `Galilego.Gameplay` и `Galilego.Physics`
- Перезапустите Unity Editor

## Дополнительные настройки

### OrbitAnalyzer настройки
- **Auto Analyze** — автоматический анализ каждые N секунд
- **Analysis Interval** — интервал обновления (по умолчанию 0.5 с)

### OrbitAnalysisUI настройки
- **Window Rect** — позиция и размер окна
- **Toggle Key** — клавиша открытия (по умолчанию O)
- **Analysis Target** — целевое тело по умолчанию
- **Show Before Maneuver** — показывать текущую орбиту
- **Show After Maneuver** — показывать орбиту после манёвра
- **Auto Update** — автоматическое обновление
- **Update Interval** — интервал обновления (по умолчанию 0.5 с)

## Производительность

### Рекомендации:
- Используйте кэширование результатов анализа для часто запрашиваемых орбит
- Увеличьте `updateInterval` если FPS падает
- Отключите `autoUpdate` если анализ не нужен постоянно
- Используйте `ClearCache()` периодически для освобождения памяти

### Оптимизация:
```csharp
// Кэширование результата
var result = analyzer.AnalyzeShipOrbit(ReferenceFrameTarget.Jupiter);
analyzer.CacheAnalysis(0, result);

// Повторное использование
var cached = analyzer.GetCachedAnalysis(0);
if (cached.IsValid)
{
    // Используем кэшированный результат
}

// Очистка кэша
analyzer.ClearCache();
```

---

**Дата:** 2026-05-18  
**Версия:** 1.0  
**Статус:** ✅ Готово к использованию
