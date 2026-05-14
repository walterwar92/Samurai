% MAIN — оркестратор offline-синтеза регулятора для робота SAMURAI.
%
% ===============================================================
%   ЧТО ДЕЛАЕТ ЭТОТ СКРИПТ:
% ===============================================================
%   1. Загружает физические параметры и веса критерия (samurai_params)
%   2. Линеаризует нелинейную ОДУ робота (linearize_samurai)
%   3. Дискретизирует модель ZOH-методом (discretize_samurai)
%   4. Анализирует устойчивость и управляемость
%   5. Синтезирует МОДАЛЬНЫЙ регулятор (pole placement, design_modal)
%   6. Синтезирует ЛКР через DARE (design_lqr)
%   7. Строит MPC-матрицы с горизонтом N (design_mpc)
%   8. Синтезирует наблюдатель Луенбергера (design_observer)
%   9. Симулирует замкнутые системы (simulate_closed_loop)
%   10. Экспортирует все матрицы в ../config.yaml (export_to_yaml)
%
% ===============================================================
%   КАК ЗАПУСКАТЬ:
% ===============================================================
%   В MATLAB, находясь в этой папке:
%       >> main
%
%   На выходе:
%     • 3 окна с переходными процессами (LQR / Modal / MPC)
%     • Обновлённый ../config.yaml — раздел `control:` с матрицами
%     • Лог в командное окно — параметры, собственные значения и т.д.
%
% ===============================================================
%   КАК ИЗМЕНИТЬ ПОВЕДЕНИЕ РОБОТА:
% ===============================================================
%   Правьте samurai_params.m → перезапустите main → перезапустите motor_node.
%   Подробнее — в README.md и latex_doc/control_theory/main.pdf, глава 4.

clear; clc; close all;

fprintf('═════════════════════════════════════════════\n');
fprintf('  SAMURAI — синтез регулятора движения\n');
fprintf('═════════════════════════════════════════════\n\n');

%% 1. Параметры
% Загружаем настройки. Меняйте здесь же samurai_params.m под свою задачу.
p = samurai_params();
fprintf('1. Параметры загружены: Ts=%g с, v0=%g м/с\n\n', p.Ts, p.v0);

%% 2. Линеаризация
% Считаем якобианы df/dx, df/du в опорной точке.
[A, B, C, D] = linearize_samurai(p);
fprintf('2. Непрерывные матрицы линеаризованной модели:\n');
fprintf('   A (5×5):\n'); disp(A);
fprintf('   B (5×2):\n'); disp(B);

%% 3. Дискретизация
% ZOH-преобразование через матричную экспоненту.
[Ad, Bd] = discretize_samurai(A, B, p.Ts);
fprintf('3. Дискретные матрицы (Ts=%g с):\n', p.Ts);
fprintf('   Ad:\n'); disp(Ad);
fprintf('   Bd:\n'); disp(Bd);

%% 4. Анализ объекта (управляемость, наблюдаемость, устойчивость)
% На этом этапе становится ясно, имеет ли смысл идти дальше:
% если объект не управляем — никакой K не поможет.
fprintf('4. Анализ свойств объекта в открытом контуре:\n');
lam_open = eig(Ad);
fprintf('   |λ| открытой системы: %s\n', mat2str(abs(lam_open), 4));
if any(abs(lam_open) >= 1 - 1e-9)
  fprintf('   ⚠  В открытом контуре есть λ на/вне единичного круга\n');
  fprintf('       — нужен стабилизирующий регулятор.\n');
end
fprintf('   rank(управляемости) = %d (нужно %d для полной управляемости)\n', ...
        rank(ctrb(Ad, Bd)), size(Ad, 1));
fprintf('   rank(наблюдаемости) = %d (нужно %d для полной наблюдаемости)\n\n', ...
        rank(obsv(Ad, C)), size(Ad, 1));

%% 5. Модальный синтез (pole placement)
% Самый прямой метод: задаём желаемые полюса, получаем K.
fprintf('5. Модальный регулятор (pole placement):\n');
[K_modal, eigs_modal] = design_modal(Ad, Bd, p.poles_continuous, p.Ts);
fprintf('\n');

%% 6. ЛКР через DARE
% Оптимальный регулятор по интегральному критерию J = Σ(x'Qx + u'Ru).
fprintf('6. ЛКР через DARE:\n');
[K_lqr, P_inf, eigs_lqr] = design_lqr(Ad, Bd, p.Q, p.R);
fprintf('\n');

%% 7. MPC (использует Pf = P_inf для гарантии устойчивости)
% Прогнозирующий регулятор с горизонтом N.
% Терминальный штраф = решение Риккати — это даёт устойчивость.
fprintf('7. MPC (N=%d, Pf = решение Риккати):\n', p.N);
mpc = design_mpc(Ad, Bd, p.Q, p.R, P_inf, p.N);
fprintf('\n');

%% 8. Наблюдатель Луенбергера
% Восстанавливает полное состояние из доступных измерений.
% Полюса в speedup раз быстрее регулятора.
fprintf('8. Наблюдатель состояния (Luenberger):\n');
[L_obs, eigs_obs] = design_observer(Ad, C, eigs_lqr, p.Ts, p.observer_speedup);
fprintf('\n');

%% 9. Симуляция замкнутых систем
% Сравниваем три синтезированных регулятора на одинаковом начальном
% возмущении. Графики показывают, как быстро и плавно каждый из них
% возвращает робота в опорную точку.
fprintf('9. Симуляция замкнутых систем:\n');
% Начальное возмущение: 1 м назад, 1 м вбок, 0.5 рад крена курса.
% Это типичная «ошибка позиционирования», которую регулятор должен исправить.
x0 = [1; 1; 0.5; 0; 0];
T  = 200;                  % 10 секунд при Ts=0.05

simulate_closed_loop(Ad, Bd, K_lqr,        x0, T, p.u_min, p.u_max, ...
                     'ЛКР — переходный процесс');
simulate_closed_loop(Ad, Bd, K_modal,      x0, T, p.u_min, p.u_max, ...
                     'Модальный регулятор — переходный процесс');
simulate_closed_loop(Ad, Bd, mpc.K_first,  x0, T, p.u_min, p.u_max, ...
                     'MPC (явное решение + clip) — переходный процесс');
fprintf('   Построено 3 окна с графиками.\n\n');

%% 10. Путь к config.yaml (экспорт — в шаге 12, после синтеза МПС)
yaml_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'config.yaml');

%% 11. МПС — каноническая модель пространства состояний [s,v,θ,ω,e_int]
% Отдельная подсистема (модуль курсовой). Аналитическая непрерывная
% модель из постоянных времени моторов → ZOH → MPC. A/B в config.yaml
% mps: пишутся НЕПРЕРЫВНЫМИ (контракт docs/mps/api.md).
fprintf('11. МПС — синтез канонической модели:\n');
[A_mps, B_mps, C_mps, D_mps] = build_canonical_mps(p.mps.tau_v, p.mps.tau_w);
fprintf('   Непрерывная A_mps (5×5):\n'); disp(A_mps);
[Ad_mps, Bd_mps] = discretize_samurai(A_mps, B_mps, p.mps.Ts);
[~, Pf_mps] = design_lqr(Ad_mps, Bd_mps, p.mps.Q, p.mps.R);
mpc_mps = design_mpc(Ad_mps, Bd_mps, p.mps.Q, p.mps.R, Pf_mps, p.mps.N);
% Переходный процесс «проехать 2 м вперёд»: x0 = ошибка позиции −2 м по s.
simulate_closed_loop(Ad_mps, Bd_mps, mpc_mps.K_first, ...
                     [-2; 0; 0; 0; 0], round(3 / p.mps.Ts), ...
                     p.mps.u_min, p.mps.u_max, ...
                     'МПС — переходный процесс (s: −2 м → 0)');
fprintf('\n');

%% 12. Экспорт обоих блоков (control: + mps:) в config.yaml
fprintf('12. Экспорт config.yaml (блоки control + mps)...\n');
mps_export = struct('A_c', A_mps, 'B_c', B_mps, 'C', C_mps, 'D', D_mps, ...
                    'tau_v', p.mps.tau_v, 'tau_w', p.mps.tau_w, ...
                    'Ts', p.mps.Ts, 'Q', p.mps.Q, 'R', p.mps.R, ...
                    'N', p.mps.N, 'u_min', p.mps.u_min, 'u_max', p.mps.u_max);
export_to_yaml(Ad, Bd, K_lqr, mpc.K_first, L_obs, P_inf, p, yaml_path, mps_export);

fprintf('\n═════════════════════════════════════════════\n');
fprintf('  ГОТОВО\n');
fprintf('═════════════════════════════════════════════\n');
fprintf('Чтобы включить регулятор:\n');
fprintf('  1. Откройте config.yaml\n');
fprintf('  2. Найдите раздел `control:`\n');
fprintf('  3. Поменяйте `mode: "off"` на `"lqr"` / `"mpc"` / `"modal"`\n');
fprintf('  4. Перезапустите motor_node\n');
fprintf('Полная документация — в latex_doc/control_theory/main.pdf.\n');
