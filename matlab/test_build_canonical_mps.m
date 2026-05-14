function test_build_canonical_mps()
% Тест структуры канонической непрерывной модели [s, v, theta, omega, e_int].
% Run:  matlab -batch "addpath('matlab'); test_build_canonical_mps"
  here = fileparts(mfilename('fullpath'));
  addpath(here);

  tau_v = 0.15; tau_w = 0.10;
  [A, B, C, D] = build_canonical_mps(tau_v, tau_w);

  % -- shapes --
  assert(isequal(size(A), [5 5]), 'A must be 5x5');
  assert(isequal(size(B), [5 2]), 'B must be 5x2');
  assert(isequal(size(C), [5 5]), 'C must be 5x5');
  assert(isequal(size(D), [5 2]), 'D must be 5x2');

  % -- canonical pattern (Option D: e_int = -theta) --
  assert(A(1,2) == 1,                   'A[0][1] must be 1 (s_dot = v)');
  assert(abs(A(2,2) + 1/tau_v) < 1e-12, 'A[1][1] must be -1/tau_v');
  assert(A(3,4) == 1,                   'A[2][3] must be 1 (theta_dot = omega)');
  assert(abs(A(4,4) + 1/tau_w) < 1e-12, 'A[3][3] must be -1/tau_w');
  assert(A(5,3) == -1,                  'A[4][2] must be -1 (e_int_dot = -theta)');
  assert(abs(B(2,1) - 1/tau_v) < 1e-12, 'B[1][0] must be 1/tau_v');
  assert(abs(B(4,2) - 1/tau_w) < 1e-12, 'B[3][1] must be 1/tau_w');
  assert(isequal(C, eye(5)),            'C must be I5');
  assert(isequal(D, zeros(5,2)),        'D must be 0');

  % -- all other A entries zero --
  mask = true(5,5);
  mask(1,2) = false; mask(2,2) = false; mask(3,4) = false;
  mask(4,4) = false; mask(5,3) = false;
  assert(all(A(mask) == 0), 'non-pattern A entries must be 0');

  % -- controllable --
  assert(rank(ctrb(A, B)) == 5, 'canonical (A,B) must be controllable');

  fprintf('PASS: build_canonical_mps — структура канонической модели верна\n');
end
