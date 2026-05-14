function [A, B, C, D] = build_canonical_mps(tau_v, tau_w)
% BUILD_CANONICAL_MPS — непрерывная каноническая модель модуля МПС.
%
%   Вектор состояния (n=5):  x = [s, v, theta, omega, e_int]'
%   Вектор управления (r=2): u = [v_cmd, omega_cmd]'
%
%   Каноническая ОДУ-модель (после линеаризации, см. спеку МПС §4.2):
%       s_dot     = v
%       v_dot     = -(1/tau_v)*v     + (1/tau_v)*u_v
%       theta_dot = omega
%       omega_dot = -(1/tau_w)*omega + (1/tau_w)*u_omega
%       e_int_dot = -theta              <- интеграл ошибки курса (theta_ref = 0)
%
%   В ОТЛИЧИЕ от linearize_samurai.m здесь НЕТ якобианов: модель
%   аналитическая, строится напрямую из постоянных времени моторов.
%   Возвращает НЕПРЕРЫВНЫЕ A, B (контракт docs/mps/api.md — бэкенд
%   ZOH-дискретизирует их сам при mps.plant.Ts).

  A = [ 0,  1,        0,  0,        0;
        0, -1/tau_v,  0,  0,        0;
        0,  0,        0,  1,        0;
        0,  0,        0, -1/tau_w,  0;
        0,  0,       -1,  0,        0 ];

  B = [ 0,        0;
        1/tau_v,  0;
        0,        0;
        0,        1/tau_w;
        0,        0 ];

  C = eye(5);
  D = zeros(5, 2);

  % ── Проверка управляемости (учебник §3.0) ────────────────────
  if rank(ctrb(A, B)) < 5
    error('build_canonical_mps:notControllable', ...
          'Каноническая (A,B) не управляема — проверьте tau_v, tau_w ~= 0');
  end
end
