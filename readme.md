## Тестовое задание для СТЦ

Специальный технологический центр

Описание задания находится в файле 
`/Тестовое задание СТЦ.pdf`


### Зависимости

Eigen:

```bash
apt-get install libeigen3-dev
```

### Комментарии

В задании требуется написать численную оптимизацию с помощью градиентного
спуска. Однако в примере [2] используется не градиентный спуск, а 
оптимизация [Левенберга - Марквардта](https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm)

Этот алгоритм может рассматриваться, как комбинация 
метода градиентного спуска и метода Ньютона.

В `Eigen` есть готовая реализация этого метода.
Подробнее можно прочитать [здесь](https://stackoverflow.com/questions/18509228/how-to-use-the-eigen-unsupported-levenberg-marquardt-implementation)

По умолчанию якобиан 
не задаётся в явном виде, а вычисляется с помощью дифференциальных
разностей.

### Структура проекта

Проект состоит из двух частей: 
- прямая задача: по разностям хода от единого передатчика `D` до приёмников `A`,
`B` и `C`, значая координаты приёмников, необходимо восстановить координаты передатчика.
Демо-программа находится в файле `demo_ft.cpp`, тесты - в файле `test/testForwardTask.cpp`,
функтор оптимизации - в файле `ofunctions/ForwardTaskFunctor`
- обратная задача: по разностям хода от трёх передатчиков `D`, `E`, `F` до приёмников `A`,
`B` и `C`, значая координаты передачтиков, необходимо восстановить координаты приёмников.
Демо-программа находится в файле `demo_it.cpp`, тесты - в файле `test/testInverseTask.cpp`,
функтор оптимизации - в файле `ofunctions/InverseTaskFunctor`