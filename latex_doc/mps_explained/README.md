# SAMURAI · МПС, объяснённая с нуля

Модульный LaTeX-учебник по математической модели МПС
(модель пространства состояний) робота SAMURAI и MPC-регулятору.
Story-driven — главы идут по сценарию использования робота, а не по
формальной сложности.

Целевая аудитория: студент 3–4 курса ИТ-вуза, знаком с матаном и
линейной алгеброй, ТАУ — впервые.

## Структура

```
mps_explained/
├── main.tex                          ← головной файл (титул + \include всех глав)
├── preamble.tex                      ← пакеты, окружения, макросы (переиспользуется)
├── chapters/
│   ├── 00_intro.tex                  Зачем эта книжка
│   ├── 01_what_robot_does.tex        Робот глазами пользователя
│   ├── 02_kinematics.tex             Дифференциальная кинематика
│   ├── 03_where_state_comes_from.tex Одометрия, IMU, position_fusion
│   ├── 04_dynamics.tex               Инерционность моторов, нелинейная ОДУ
│   ├── 05_linearization.tex          Ряд Тейлора, ZOH-дискретизация
│   ├── 06_controllers_intro.tex      П → PID → ЛКР → MPC (интуиция)
│   ├── 07_lqr.tex                    ЛКР через DARE
│   ├── 08_mpc.tex                    MPC: лифтинг, QP, clip/qp solvers
│   ├── 09_scenario_forward.tex       Сценарий «D метров вперёд»
│   ├── 10_turn_phase.tex             Двухфазный TURN→DRIVE
│   ├── 11_target_picker.tex          Математика 3D-пикера цели
│   └── 12_implementation.tex         MATLAB → YAML → Python, live Apply
└── figures/                          TikZ-исходники (часть встроена в .tex)
```

## Компиляция

### Полный документ

Нужны `pdflatex` и пакеты `amsmath`, `tikz`, `tcolorbox`, `listings`,
`hyperref`, `babel[russian]` (стандартный TeX Live с `texlive-lang-cyrillic`).

```bash
cd latex_doc/mps_explained
pdflatex main.tex
pdflatex main.tex      # второй проход для оглавления и cross-refs
```

На Windows (MiKTeX или TeX Live):

```powershell
cd latex_doc\mps_explained
pdflatex main.tex
pdflatex main.tex
```

Результат — `main.pdf`.

### Отдельная глава

Каждая глава самодостаточна — её можно собрать отдельно, обернув в
mini-`main.tex`:

```latex
\documentclass[12pt, a4paper, openany]{book}
\input{../preamble}        % относительный путь к общему preamble
\begin{document}
\include{05_linearization} % одна глава
\end{document}
```

Сложите такой файл в `chapters/_solo_05.tex` и соберите его — получите
PDF одной главы. Удобно для отправки конкретного раздела в чате.

## Что внутри (короткая выжимка)

| Глава | Что отвечает |
|------:|--------------|
| 0     | Кто ты, что нужно знать, как читать |
| 1     | Что робот делает, какие три слоя проекта |
| 2     | $v_L, v_R$ от $u_v, u_\omega$ и обратно; мировая кинематика |
| 3     | Откуда $s, v, \theta, \omega, e_\text{int}$ — энкодеры, IMU, fusion |
| 4     | Почему $\dot v = (u_v - v)/\tau_v$, что такое $\tau_v, \tau_\omega$ |
| 5     | Линеаризация Тейлора → $\mathbf{A}, \mathbf{B}$ → ZOH → $\mathbf{A}_d, \mathbf{B}_d$ |
| 6     | П, PID, ЛКР, MPC — мотивация и сравнение |
| 7     | ЛКР: вывод DARE, выбор $\mathbf{Q}, \mathbf{R}$, замкнутая система |
| 8     | MPC: лифтинг, QP-форма, `clip` vs `qp`, $\mathbf{P}_f$ = DARE |
| 9     | Сценарий D метров: reference, метрики, lifecycle, safety |
| 10    | TURN→DRIVE: разные капы, wrap-around курса |
| 11    | 3D-пикер: $\varphi = \mathrm{atan2}(-z_3, x_3)$, дед-зоны |
| 12    | Где живут числа: MATLAB → YAML → Python, live Apply, тесты |

## Связи с другими документами проекта

- **Формальный референс:** `latex_doc/control_theory/main.tex` —
  сухой справочник тех же тем (4 главы вместо 13). Тон — учебный по
  Козлову, без story-driven.
- **Общая математика робота:** `latex_doc/main.tex` — 15 глав
  по фильтрам, SLAM, кватернионам, A*, кинематике. МПС там не
  затрагивается.
- **Документы модуля:** `docs/mps/architecture.md`, `api.md`,
  `scenario_forward.md` — markdown-проектная документация (что
  снаружи). Не дублирует — дополняет.
- **Спека (источник правды):**
  `docs/superpowers/specs/2026-05-05-mps-state-space-design.md` —
  проектные решения feat/mps.

## Учебник Козлова

Изложение опирается на:

> Козлов В. Н., Куприянов В. Е., Шашихин В. Н.
> *Теория автоматического управления.* СПбГПУ, 2008.

Главы 1, 3 (модель, синтез регуляторов) — там же.

## Лицензия

Та же, что у проекта SAMURAI (см. корневой `LICENSE`).
