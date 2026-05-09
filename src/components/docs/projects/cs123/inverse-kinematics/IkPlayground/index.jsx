import React, { useEffect, useMemo, useRef, useState } from 'react';
import {
  Activity,
  AlertTriangle,
  BookOpen,
  Calculator,
  ChevronDown,
  ChevronLeft,
  ChevronRight,
  Crosshair,
  RotateCcw,
  Sigma,
  Target,
} from 'lucide-react';

const PAGES = [
  { title: 'IK 是什么', icon: BookOpen, color: 'tw-text-rose-400' },
  { title: '解析解 (3-DoF)', icon: Calculator, color: 'tw-text-amber-400' },
  { title: 'DLS 数值法', icon: Sigma, color: 'tw-text-cyan-400' },
  { title: '三大工程难题', icon: AlertTriangle, color: 'tw-text-orange-400' },
  { title: '轨迹跟踪', icon: Target, color: 'tw-text-emerald-400' },
];

// ===== 机械臂连杆长度 (像素) =====
const L1 = 140;
const L2 = 110;
const L3 = 80;
const L_SUM = L1 + L2 + L3;
const L_INNER = Math.max(0, Math.abs(L1 - L2) - L3); // 内圈最小可达半径估计

// ===== 视图常量 =====
const ARM_VIEW = 700;
const ERR_CHART_W = 700;
const ERR_CHART_H = 110;
const ERR_HISTORY_MAX = 240;
const TRACE_MAX = 220;

// ===== DLS 默认参数 =====
const DEFAULT_LAMBDA = 0.08;
const DEFAULT_STEP = 0.35;
const ARM_DT = 0.016;

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const deg2rad = (d) => (d * Math.PI) / 180;
const rad2deg = (r) => (r * 180) / Math.PI;

// ===== 平面 3-DoF FK (输入弧度) =====
const fkPlanar = (t1, t2, t3) => {
  const c1 = Math.cos(t1);
  const s1 = Math.sin(t1);
  const c12 = Math.cos(t1 + t2);
  const s12 = Math.sin(t1 + t2);
  const c123 = Math.cos(t1 + t2 + t3);
  const s123 = Math.sin(t1 + t2 + t3);
  return {
    j1x: L1 * c1,
    j1y: L1 * s1,
    j2x: L1 * c1 + L2 * c12,
    j2y: L1 * s1 + L2 * s12,
    j3x: L1 * c1 + L2 * c12 + L3 * c123,
    j3y: L1 * s1 + L2 * s12 + L3 * s123,
    s1, c1, s12, c12, s123, c123,
  };
};

// ===== 平面 3-DoF 雅可比 (位置 2x3) =====
const jacobianPlanar = (t1, t2, t3) => {
  const s1 = Math.sin(t1), c1 = Math.cos(t1);
  const s12 = Math.sin(t1 + t2), c12 = Math.cos(t1 + t2);
  const s123 = Math.sin(t1 + t2 + t3), c123 = Math.cos(t1 + t2 + t3);
  return [
    [-L1 * s1 - L2 * s12 - L3 * s123, -L2 * s12 - L3 * s123, -L3 * s123],
    [L1 * c1 + L2 * c12 + L3 * c123, L2 * c12 + L3 * c123, L3 * c123],
  ];
};

// ===== 一步 DLS 迭代 =====
// theta: [t1,t2,t3] (rad), target: [tx, ty] (像素), lam: 阻尼, step: 学习率
// 返回 { nextTheta, error, dtheta }
const dlsStep = (theta, target, lam, step) => {
  const [t1, t2, t3] = theta;
  const fk = fkPlanar(t1, t2, t3);
  const ex = target[0] - fk.j3x;
  const ey = target[1] - fk.j3y;
  const J = jacobianPlanar(t1, t2, t3);

  // J J^T 是 2×2:
  const a = J[0][0] * J[0][0] + J[0][1] * J[0][1] + J[0][2] * J[0][2];
  const b = J[0][0] * J[1][0] + J[0][1] * J[1][1] + J[0][2] * J[1][2];
  const d = J[1][0] * J[1][0] + J[1][1] * J[1][1] + J[1][2] * J[1][2];

  // 加阻尼: A = JJT + lam^2 I
  const A00 = a + lam * lam;
  const A11 = d + lam * lam;
  const A01 = b;
  const det = A00 * A11 - A01 * A01;
  if (Math.abs(det) < 1e-9) {
    return { nextTheta: theta.slice(), error: Math.hypot(ex, ey), dtheta: [0, 0, 0] };
  }
  const inv00 = A11 / det;
  const inv01 = -A01 / det;
  const inv11 = A00 / det;

  // k = A^-1 e
  const k1 = inv00 * ex + inv01 * ey;
  const k2 = inv01 * ex + inv11 * ey;

  // dtheta = J^T k
  const dt1 = J[0][0] * k1 + J[1][0] * k2;
  const dt2 = J[0][1] * k1 + J[1][1] * k2;
  const dt3 = J[0][2] * k1 + J[1][2] * k2;

  return {
    nextTheta: [t1 + step * dt1, t2 + step * dt2, t3 + step * dt3],
    error: Math.hypot(ex, ey),
    dtheta: [dt1, dt2, dt3],
  };
};

// ===== 解析解 (3-DoF 平面臂,φ 作为输入) =====
// 返回 { ok, theta:[t1,t2,t3], reason }
const ikAnalytic = (X, Y, phi, elbowUp) => {
  const xp = X - L3 * Math.cos(phi);
  const yp = Y - L3 * Math.sin(phi);
  const r2 = xp * xp + yp * yp;
  const cosT2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (Math.abs(cosT2) > 1) {
    return { ok: false, reason: '不可达' };
  }
  const t2 = Math.acos(cosT2) * (elbowUp ? 1 : -1);
  const t1 = Math.atan2(yp, xp) - Math.atan2(L2 * Math.sin(t2), L1 + L2 * Math.cos(t2));
  const t3 = phi - t1 - t2;
  return { ok: true, theta: [t1, t2, t3] };
};

// ===== 三角形参数轨迹 =====
const TRIANGLE = [
  [200, 60],
  [60, -120],
  [240, -40],
];

const interpolateTriangle = (tSec, period = 3.0) => {
  const seg = ((tSec % period) + period) % period;
  const segLen = period / 3;
  const i = Math.floor(seg / segLen) % 3;
  const s = (seg - i * segLen) / segLen;
  const p0 = TRIANGLE[i];
  const p1 = TRIANGLE[(i + 1) % 3];
  return [p0[0] * (1 - s) + p1[0] * s, p0[1] * (1 - s) + p1[1] * s];
};

const interpolateCircle = (tSec, period = 4.0, R = 100, cx = 150, cy = 0) => {
  const w = (2 * Math.PI) / period;
  return [cx + R * Math.cos(w * tSec), cy + R * Math.sin(w * tSec)];
};

const App = () => {
  // ===== 当前 IK 状态 =====
  const [theta, setTheta] = useState([deg2rad(45), deg2rad(-30), deg2rad(-45)]);
  const [target, setTarget] = useState([180, 80]);

  // 解析解输入
  const [analyticPhi, setAnalyticPhi] = useState(0); // rad
  const [elbowUp, setElbowUp] = useState(true);

  // DLS 参数
  const [lambda, setLambda] = useState(DEFAULT_LAMBDA);
  const [stepSize, setStepSize] = useState(DEFAULT_STEP);

  // 轨迹模式 (page 5)
  const [trajMode, setTrajMode] = useState('circle'); // 'circle' | 'triangle'
  const [trajRunning, setTrajRunning] = useState(true);

  // 分页
  const [currentPage, setCurrentPage] = useState(0);
  const totalPages = PAGES.length;

  // 拖拽状态
  const [isDragging, setIsDragging] = useState(false);

  // 下拉菜单
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const dropdownRef = useRef(null);

  // ===== refs (rAF 循环里读最新值) =====
  const thetaRef = useRef(theta);
  const targetRef = useRef(target);
  const lambdaRef = useRef(lambda);
  const stepRef = useRef(stepSize);
  const currentPageRef = useRef(0);
  const trajModeRef = useRef('circle');
  const trajRunningRef = useRef(true);
  const trajTimeRef = useRef(0);
  const errorHistoryRef = useRef([]); // page 3
  const traceRef = useRef([]); // 末端轨迹
  const svgRef = useRef(null);

  useEffect(() => { thetaRef.current = theta; }, [theta]);
  useEffect(() => { targetRef.current = target; }, [target]);
  useEffect(() => { lambdaRef.current = lambda; }, [lambda]);
  useEffect(() => { stepRef.current = stepSize; }, [stepSize]);
  useEffect(() => { currentPageRef.current = currentPage; }, [currentPage]);
  useEffect(() => { trajModeRef.current = trajMode; }, [trajMode]);
  useEffect(() => { trajRunningRef.current = trajRunning; }, [trajRunning]);

  // 切到 page 3 (DLS) 时清空误差历史
  useEffect(() => {
    if (currentPage === 2) {
      errorHistoryRef.current = [];
    }
    if (currentPage === 4) {
      trajTimeRef.current = 0;
      traceRef.current = [];
    }
  }, [currentPage]);

  // 下拉菜单关闭逻辑
  useEffect(() => {
    if (!dropdownOpen) return;
    const onDocClick = (e) => {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target)) {
        setDropdownOpen(false);
      }
    };
    const onEsc = (e) => { if (e.key === 'Escape') setDropdownOpen(false); };
    document.addEventListener('mousedown', onDocClick);
    document.addEventListener('keydown', onEsc);
    return () => {
      document.removeEventListener('mousedown', onDocClick);
      document.removeEventListener('keydown', onEsc);
    };
  }, [dropdownOpen]);

  // ===== 主循环:rAF 推 DLS 一步 =====
  const [, setTick] = useState(0);
  useEffect(() => {
    let raf;
    const step = () => {
      const page = currentPageRef.current;
      let curTheta = thetaRef.current;
      let curTarget = targetRef.current;

      // 页 5:轨迹自动驱动 target
      if (page === 4 && trajRunningRef.current) {
        trajTimeRef.current += ARM_DT;
        const t = trajTimeRef.current;
        if (trajModeRef.current === 'triangle') {
          curTarget = interpolateTriangle(t);
        } else {
          curTarget = interpolateCircle(t);
        }
        targetRef.current = curTarget;
        // 仅在轨迹更新时同步 React state(节流到每 4 帧一次,减少 re-render)
      }

      // 页 1 / 3 / 4 / 5:每帧跑 1-3 步 DLS
      const iters = page === 2 ? 1 : 2; // page 3 慢一点便于看收敛
      for (let i = 0; i < iters; i++) {
        const r = dlsStep(curTheta, curTarget, lambdaRef.current, stepRef.current);
        // 防止数值发散 (lam=0 + 奇异时可能爆掉)
        const maxAbs = Math.max(
          Math.abs(r.nextTheta[0]), Math.abs(r.nextTheta[1]), Math.abs(r.nextTheta[2]),
        );
        if (Number.isFinite(maxAbs) && maxAbs < 1e3) {
          curTheta = r.nextTheta;
        }

        // page 3:记录误差曲线
        if (page === 2 && i === 0) {
          errorHistoryRef.current.push(r.error);
          if (errorHistoryRef.current.length > ERR_HISTORY_MAX) errorHistoryRef.current.shift();
        }
      }

      thetaRef.current = curTheta;

      // 末端轨迹 (页 5 用)
      if (page === 4) {
        const fk = fkPlanar(curTheta[0], curTheta[1], curTheta[2]);
        traceRef.current.push([fk.j3x, fk.j3y]);
        if (traceRef.current.length > TRACE_MAX) traceRef.current.shift();
      }

      // 同步 state 触发渲染 (节流)
      setTheta(curTheta);
      if (page === 4 && trajRunningRef.current) {
        setTarget(curTarget);
      }
      setTick((t) => (t + 1) % 1_000_000);

      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, []);

  // ===== 拖动目标 =====
  const handlePointer = (e) => {
    const svg = svgRef.current;
    if (!svg) return;
    const pt = svg.createSVGPoint();
    pt.x = e.clientX;
    pt.y = e.clientY;
    const ctm = svg.getScreenCTM();
    if (!ctm) return;
    const p = pt.matrixTransform(ctm.inverse());
    // viewBox 中心是 (0, 0),y 轴向下,我们 SVG 内部有 scale(1,-1) 翻 Y
    // 所以拖到画面上方 → DOM y 小 → 数学 y 大
    const x = p.x;
    const y = -p.y;
    setTarget([x, y]);
  };

  const onPointerDown = (e) => {
    e.currentTarget.setPointerCapture?.(e.pointerId);
    setIsDragging(true);
    if (currentPageRef.current === 4) setTrajRunning(false);
    handlePointer(e);
  };
  const onPointerMove = (e) => {
    if (e.buttons === 1) handlePointer(e);
  };
  const onPointerUp = (e) => {
    e.currentTarget.releasePointerCapture?.(e.pointerId);
    setIsDragging(false);
  };

  // ===== 派生 =====
  const fk = fkPlanar(theta[0], theta[1], theta[2]);
  const errorVec = [target[0] - fk.j3x, target[1] - fk.j3y];
  const errorMag = Math.hypot(errorVec[0], errorVec[1]);
  const targetDist = Math.hypot(target[0], target[1]);
  const reachable = targetDist <= L_SUM + 0.5;

  // 解析解的两个分支 (页 2)
  const analyticUp = useMemo(
    () => ikAnalytic(target[0], target[1], analyticPhi, true),
    [target, analyticPhi],
  );
  const analyticDown = useMemo(
    () => ikAnalytic(target[0], target[1], analyticPhi, false),
    [target, analyticPhi],
  );

  // 解析解选支跟拖动同步 (页 2)
  useEffect(() => {
    if (currentPage !== 1) return;
    const sol = elbowUp ? analyticUp : analyticDown;
    if (sol.ok) {
      setTheta(sol.theta.slice());
      thetaRef.current = sol.theta.slice();
    }
  }, [currentPage, analyticUp, analyticDown, elbowUp]);

  // 误差曲线点串
  const errorPoints = useMemo(() => {
    const hist = errorHistoryRef.current;
    if (hist.length < 2) return '';
    const maxE = Math.max(...hist, 1);
    return hist
      .map((e, i) => {
        const x = (i / Math.max(1, ERR_HISTORY_MAX - 1)) * ERR_CHART_W;
        const y = ERR_CHART_H - clamp((e / maxE) * (ERR_CHART_H - 12), 0, ERR_CHART_H - 6) - 6;
        return `${x},${y}`;
      })
      .join(' ');
  }, [theta]);

  const nextPage = () => setCurrentPage((p) => Math.min(totalPages - 1, p + 1));
  const prevPage = () => setCurrentPage((p) => Math.max(0, p - 1));

  const resetIK = () => {
    const t0 = [deg2rad(45), deg2rad(-30), deg2rad(-45)];
    setTheta(t0);
    thetaRef.current = t0;
    errorHistoryRef.current = [];
    traceRef.current = [];
  };

  return (
    <div
      style={{ minHeight: 'calc(100vh - var(--ifm-navbar-height, 60px) - 48px)' }}
      className="tw-relative tw-flex tw-flex-col md:tw-flex-row md:tw-h-[calc(100vh-var(--ifm-navbar-height,60px)-48px)] md:tw-overflow-hidden tw-font-sans tw-text-slate-100"
    >
      {/* ===== 左侧:课程内容 ===== */}
      <div className="tw-w-full md:tw-w-1/2 lg:tw-w-5/12 tw-bg-slate-800/75 tw-backdrop-blur-sm tw-border-r tw-border-slate-700 tw-shadow-sm tw-z-10 tw-flex tw-flex-col">

        {/* 顶部分页导航栏 */}
        <div className="tw-relative tw-flex tw-items-center tw-gap-2 tw-px-4 md:tw-px-6 tw-py-3 tw-border-b tw-border-slate-700 tw-bg-slate-800/90 tw-flex-shrink-0 tw-z-20">
          <button
            type="button"
            onClick={prevPage}
            disabled={currentPage === 0}
            aria-label="上一页"
            className={`tw-inline-flex tw-items-center tw-gap-1 tw-px-3 tw-py-1.5 tw-rounded-md tw-text-sm tw-font-medium tw-transition ${
              currentPage === 0
                ? 'tw-text-slate-600 tw-bg-slate-900/60 tw-cursor-not-allowed'
                : 'tw-text-slate-100 tw-bg-slate-700 hover:tw-bg-slate-600 active:tw-scale-95'
            }`}
          >
            <ChevronLeft size={16} />
            上一页
          </button>

          <div ref={dropdownRef} className="tw-relative tw-flex-1 tw-min-w-0">
            <button
              type="button"
              onClick={() => setDropdownOpen((v) => !v)}
              aria-haspopup="listbox"
              aria-expanded={dropdownOpen}
              className="tw-w-full tw-inline-flex tw-items-center tw-justify-center tw-gap-2 tw-px-3 tw-py-1.5 tw-rounded-lg tw-border tw-border-slate-700 tw-bg-slate-900/60 hover:tw-bg-slate-900 tw-text-sm tw-font-medium tw-text-slate-100 tw-truncate"
            >
              <span className="tw-text-slate-500 tw-shrink-0">第 {currentPage + 1} 页 ·</span>
              <span className="tw-truncate">{PAGES[currentPage].title}</span>
              <ChevronDown
                size={14}
                className={`tw-shrink-0 tw-transition-transform ${dropdownOpen ? 'tw-rotate-180' : ''}`}
              />
            </button>

            {dropdownOpen && (
              <ul
                role="listbox"
                className="tw-absolute tw-top-full tw-left-1/2 -tw-translate-x-1/2 tw-mt-2 tw-z-50 tw-min-w-[260px] tw-max-w-[360px] tw-rounded-lg tw-border tw-border-slate-700 tw-bg-slate-900 tw-shadow-xl tw-p-1 tw-list-none tw-m-0"
              >
                {PAGES.map((p, idx) => {
                  const active = idx === currentPage;
                  const Icon = p.icon;
                  return (
                    <li key={idx} role="option" aria-selected={active}>
                      <button
                        type="button"
                        onClick={() => { setCurrentPage(idx); setDropdownOpen(false); }}
                        className={`tw-w-full tw-flex tw-items-center tw-gap-2 tw-px-3 tw-py-2 tw-rounded-md tw-text-left tw-text-sm tw-transition ${
                          active
                            ? 'tw-bg-rose-600 tw-text-white'
                            : 'tw-text-slate-200 hover:tw-bg-slate-700'
                        }`}
                      >
                        <Icon size={14} className={active ? 'tw-text-white' : p.color} />
                        <span className="tw-font-mono tw-text-xs tw-opacity-70 tw-shrink-0">{idx + 1}</span>
                        <span className="tw-truncate">{p.title}</span>
                      </button>
                    </li>
                  );
                })}
              </ul>
            )}
          </div>

          <button
            type="button"
            onClick={nextPage}
            disabled={currentPage === totalPages - 1}
            aria-label="下一页"
            className={`tw-inline-flex tw-items-center tw-gap-1 tw-px-3 tw-py-1.5 tw-rounded-md tw-text-sm tw-font-medium tw-transition ${
              currentPage === totalPages - 1
                ? 'tw-text-slate-600 tw-bg-slate-900/60 tw-cursor-not-allowed'
                : 'tw-text-white tw-bg-rose-600 hover:tw-bg-rose-500 active:tw-scale-95 tw-shadow-sm'
            }`}
          >
            下一页
            <ChevronRight size={16} />
          </button>

          <span className="tw-text-xs tw-font-mono tw-text-slate-500 tw-shrink-0 tw-tabular-nums tw-ml-1">
            {currentPage + 1}/{totalPages}
          </span>
        </div>

        {/* 内容分页 */}
        <div className="tw-p-5 md:tw-p-6 tw-flex-grow tw-overflow-y-auto">

          {/* === Page 1: IK 是什么 === */}
          {currentPage === 0 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <BookOpen className="tw-mr-3 tw-text-rose-400" size={24} />
                1. IK 是什么
              </h2>

              <div className="tw-bg-rose-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-rose-900/60">
                <p className="tw-mb-2">
                  <strong>正运动学 FK</strong>：θ → 末端位姿，是单向的、好算的。<br />
                  <strong>逆运动学 IK</strong>：末端位姿 → θ，<strong>反过来</strong>解一个非线性方程组——通常欠定或过定，这正是它比 FK 麻烦的根源。
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-rose-200 tw-leading-loose">
                  FK : θ ──▶ p<br />
                  IK : p ──▶ θ&nbsp;&nbsp; (multi / singular / unreachable)
                </div>
                <p className="tw-text-xs tw-text-slate-300 tw-mt-2">
                  右图中红色十字就是末端目标——拖它，机械臂会用<strong>阻尼最小二乘 DLS</strong> 一步一步把末端追过去。
                </p>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-rose-300 tw-text-sm tw-mb-1.5">为什么要 IK</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    机器人任务（抓取、焊接、画圆）总是<strong>用末端位置/姿态描述</strong>，但执行器吃的是<strong>关节角</strong>——IK 就是这中间的翻译层。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1.5">两大流派</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    <strong>解析解</strong>一次到位、无误差，但只有平面臂、球腕等特殊结构能写出来；<strong>数值解 (DLS)</strong> 通用，靠雅可比线性化迭代，是工业界主力。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-orange-300 tw-text-sm tw-mb-1.5">三大工程难题</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    <strong>多解性</strong>（肘上/肘下）、<strong>奇异状态</strong>（伸直时雅可比退化）、<strong>不可达性</strong>（目标在工作空间外）。后面的页一一展开。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700">
                💡 试着把目标拖出甜甜圈外圈，看 DLS 怎么「贴边追」——这就是它优于纯伪逆的工程优势。
              </div>
            </div>
          )}

          {/* === Page 2: 解析解 === */}
          {currentPage === 1 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Calculator className="tw-mr-3 tw-text-amber-400" size={24} />
                2. 解析解 (3-DoF 平面臂)
              </h2>

              <div className="tw-bg-amber-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-amber-900/60">
                <p className="tw-mb-2">
                  把<strong>末端朝向 φ</strong> 也作为输入，前两关节就退化成 2-DoF 几何问题。先把第三连杆的影响减掉，再用余弦定理：
                </p>
                <div className="tw-font-mono tw-text-[12px] tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-amber-200 tw-leading-loose tw-overflow-x-auto">
                  x' = X − L₃cosφ &nbsp;&nbsp; y' = Y − L₃sinφ<br />
                  cosθ₂ = (x'² + y'² − L₁² − L₂²) / (2 L₁ L₂)<br />
                  θ₂ = ±acos(·) &nbsp;&nbsp; <span className="tw-text-rose-300">{'// ± 就是肘上/肘下'}</span><br />
                  θ₁ = atan2(y', x') − atan2(L₂sinθ₂, L₁ + L₂cosθ₂)<br />
                  θ₃ = φ − θ₁ − θ₂
                </div>
              </div>

              <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-2">代入当前目标</h3>
                <div className="tw-font-mono tw-text-[12px] tw-text-slate-300 tw-leading-loose tw-space-y-0.5">
                  <div>X = {target[0].toFixed(1)}, Y = {target[1].toFixed(1)}, φ = {rad2deg(analyticPhi).toFixed(1)}°</div>
                  <div className="tw-pt-2 tw-border-t tw-border-slate-700 tw-mt-2"></div>
                  <div>
                    {analyticUp.ok ? (
                      <>
                        <span className="tw-text-cyan-300">肘上：</span>θ = (
                        {rad2deg(analyticUp.theta[0]).toFixed(1)}°,{' '}
                        {rad2deg(analyticUp.theta[1]).toFixed(1)}°,{' '}
                        {rad2deg(analyticUp.theta[2]).toFixed(1)}°)
                      </>
                    ) : (
                      <span className="tw-text-rose-300">肘上：不可达</span>
                    )}
                  </div>
                  <div>
                    {analyticDown.ok ? (
                      <>
                        <span className="tw-text-purple-300">肘下：</span>θ = (
                        {rad2deg(analyticDown.theta[0]).toFixed(1)}°,{' '}
                        {rad2deg(analyticDown.theta[1]).toFixed(1)}°,{' '}
                        {rad2deg(analyticDown.theta[2]).toFixed(1)}°)
                      </>
                    ) : (
                      <span className="tw-text-rose-300">肘下：不可达</span>
                    )}
                  </div>
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-cyan-300 tw-text-sm tw-mb-1">为什么要把 φ 当输入</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    3-DoF 平面臂有 3 个关节，但平面位姿只有 (x, y, φ) 三个量，正好对得上——把 φ 提出来作为输入，余下就是 2-DoF 几何问题。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-purple-300 tw-text-sm tw-mb-1">解析解的真正价值</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    不是省 CPU，而是<strong>当数值解的参考答案</strong>。写完 DLS 后，在平面工况下和这里对比一下，能在 5 分钟内抓到 99% 的链式乘法 bug。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700">
                💡 拖动目标 + 切换肘上/肘下，可以看到同一个 (X, Y, φ) 下两组完全不同的关节角——这正是<strong>多解性</strong>的本源。
              </div>
            </div>
          )}

          {/* === Page 3: DLS 数值法 === */}
          {currentPage === 2 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Sigma className="tw-mr-3 tw-text-cyan-400" size={24} />
                3. 阻尼最小二乘 (DLS)
              </h2>

              <div className="tw-bg-cyan-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-cyan-900/60">
                <p className="tw-mb-2">
                  从当前 θ 出发，把方程线性化一步，再做带正则的最小二乘：
                </p>
                <div className="tw-font-mono tw-text-[12px] tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-cyan-200 tw-leading-loose tw-overflow-x-auto">
                  e = p<sub>target</sub> − p(θ)<br />
                  min<sub>Δθ</sub> ‖J Δθ − e‖² + λ²‖Δθ‖²<br />
                  Δθ = J<sup>T</sup>(JJ<sup>T</sup> + λ²I)<sup>−1</sup> e<br />
                  θ ← θ + step · Δθ
                </div>
                <p className="tw-text-xs tw-text-slate-300 tw-mt-2">
                  λ²I 是关键——它把奇异附近近 0 的奇异值「垫高」，使矩阵保持可逆，避免伪逆瞬间飞出去。
                </p>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-cyan-300 tw-text-sm tw-mb-1">λ 的取舍</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    λ = 0 等同于纯伪逆，奇异时会爆炸；λ 太大则永远跟不上，永远滞后。<strong>经验值 0.01 ~ 0.1</strong>。把右侧 λ 拉到 0 试试看。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1">step 的取舍</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    step = 1.0 容易震荡或越走越远；<strong>0.2 ~ 0.3</strong> 通常稳。它就是「沿 Δθ 方向走多远」的学习率。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1">实时误差曲线</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    右图下方那条曲线是 ‖e‖ 对时间的演化——指数衰减意味着 DLS 正常收敛；上下震荡或不下降，往往是 step 太大或参数失稳。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700">
                💡 拖目标到「机械臂完全伸直」那条线附近，再把 λ 滑到 0，能直观看到「奇异 + 纯伪逆」是怎么炸的。
              </div>
            </div>
          )}

          {/* === Page 4: 三大难题 === */}
          {currentPage === 3 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <AlertTriangle className="tw-mr-3 tw-text-orange-400" size={24} />
                4. 三大工程难题
              </h2>

              <div className="tw-bg-orange-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-orange-900/60">
                <p className="tw-text-xs tw-text-slate-300">
                  这一页把第 3.5 节的三类坑放到同一画布里。拖动目标，留意右图三种叠加层：<strong>淡色双解</strong>、<strong>奇异线高亮</strong>、<strong>甜甜圈外的红色阴影</strong>。
                </p>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-cyan-300 tw-text-sm tw-mb-1.5">① 多解性 (Multiple Solutions)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    <strong>淡青色 / 淡紫色</strong>两套幽灵机械臂分别是肘上、肘下解。同一个目标 (X, Y, φ) 有两组关节角——挑哪组是<em>工程问题</em>，常用「热启动」让数值法自然收敛到最近的一支。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1.5">② 奇异状态 (Singularity)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    把目标拖到外圈 (r ≈ {L_SUM}) 附近，机械臂趋于完全伸直，<strong>雅可比退化</strong>，detJJᵀ → 0。右上 HUD 会亮起 <code className="tw-text-orange-300">SINGULAR</code>。这时 λ 是救命稻草。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-rose-300 tw-text-sm tw-mb-1.5">③ 不可达 (Unreachable)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    把目标拖出甜甜圈外，画面会出现红色阴影。DLS 不会崩——它会<strong>贴着边缘最近点</strong>跟着目标走。这就是 DLS 相对纯伪逆最大的工程优势。
                  </p>
                </div>
              </div>
            </div>
          )}

          {/* === Page 5: 轨迹跟踪 === */}
          {currentPage === 4 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Target className="tw-mr-3 tw-text-emerald-400" size={24} />
                5. 末端轨迹跟踪
              </h2>

              <div className="tw-bg-emerald-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-emerald-900/60">
                <p className="tw-mb-2">
                  把第 3.7、3.8 节的两个实验搬进浏览器：让目标点沿一个参数化轨迹运动，DLS 每帧把末端拉过去——这就是 IK + 控制器最朴素的串联。
                </p>
                <div className="tw-font-mono tw-text-[12px] tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-emerald-200 tw-leading-loose tw-overflow-x-auto">
                  for t in time:<br />
                  &nbsp;&nbsp;p_target = traj(t)<br />
                  &nbsp;&nbsp;θ = ik_dls(θ, p_target)
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1.5">圆 vs 三角形</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    圆是连续可微的——IK 求解器没什么压力。三角形在三个角点处<strong>目标速度突变</strong>，你会看到末端实际轨迹的<em>角点会被「圆」掉</em>，因为 DLS 的 step 限制了单帧位移。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-cyan-300 tw-text-sm tw-mb-1.5">热启动的好处</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    每帧都把上一步的 θ 作为下一步 IK 的起点——目标只挪一点点，DLS 一两步就能跟上。这正是真机控制栈让 IK ≈ 200 Hz、PD ≈ 1 kHz 时仍然顺滑的原因。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700">
                💡 暂停后可以拖目标自己玩；点开始后回到轨迹跟踪。红色虚线是末端最近 220 帧的轨迹。
              </div>
            </div>
          )}
        </div>
      </div>

      {/* ===== 右侧:可视化画布 ===== */}
      <div className="tw-w-full md:tw-w-1/2 lg:tw-w-7/12 tw-bg-slate-900/60 tw-backdrop-blur-sm tw-flex tw-flex-col tw-gap-3 tw-p-4 md:tw-p-6 tw-relative tw-overflow-hidden tw-shadow-inner">

        <div
          className="tw-absolute tw-inset-0 tw-pointer-events-none tw-opacity-[0.12]"
          style={{
            backgroundImage: 'linear-gradient(#64748b 1px, transparent 1px), linear-gradient(90deg, #64748b 1px, transparent 1px)',
            backgroundSize: '40px 40px',
          }}
        />

        <div className="tw-flex tw-flex-col tw-gap-3 tw-z-10 tw-min-h-0 tw-overflow-y-auto">
          <ArmCanvas
            page={currentPage}
            svgRef={svgRef}
            theta={theta}
            target={target}
            fk={fk}
            errorMag={errorMag}
            reachable={reachable}
            analyticUp={analyticUp}
            analyticDown={analyticDown}
            analyticPhi={analyticPhi}
            isDragging={isDragging}
            onPointerDown={onPointerDown}
            onPointerMove={onPointerMove}
            onPointerUp={onPointerUp}
            trace={traceRef.current}
          />

          {/* 误差曲线 (page 3) */}
          {currentPage === 2 && (
            <div className="tw-flex-shrink-0">
              <div className="tw-text-xs tw-font-bold tw-text-slate-400 tw-mb-1 tw-flex tw-items-center">
                <Activity size={14} className="tw-mr-1.5" /> 末端误差 ‖e‖ 随迭代衰减
              </div>
              <svg
                viewBox={`0 0 ${ERR_CHART_W} ${ERR_CHART_H}`}
                preserveAspectRatio="none"
                className="tw-w-full tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700"
                style={{ height: '76px' }}
              >
                <rect width={ERR_CHART_W} height={ERR_CHART_H} fill="#020617" />
                <line x1="0" y1={ERR_CHART_H - 6} x2={ERR_CHART_W} y2={ERR_CHART_H - 6} stroke="#334155" strokeWidth="1" />
                {errorPoints && (
                  <polyline
                    points={errorPoints}
                    fill="none"
                    stroke="#22d3ee"
                    strokeWidth="2"
                    strokeLinejoin="round"
                  />
                )}
              </svg>
            </div>
          )}

          {/* 控制面板 */}
          <div className="tw-flex-shrink-0 tw-space-y-2">

            {/* page 2: 解析解 φ + elbow */}
            {currentPage === 1 && (
              <>
                <div className="tw-flex tw-flex-wrap tw-gap-2">
                  <button
                    type="button"
                    onClick={() => setElbowUp(true)}
                    className={`tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-lg tw-border tw-font-medium tw-transition ${
                      elbowUp
                        ? 'tw-border-cyan-500 tw-bg-cyan-950/70 tw-text-cyan-100'
                        : 'tw-border-slate-600 tw-bg-slate-800 tw-text-slate-300 hover:tw-border-cyan-400'
                    }`}
                  >
                    肘上 (elbow up)
                  </button>
                  <button
                    type="button"
                    onClick={() => setElbowUp(false)}
                    className={`tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-lg tw-border tw-font-medium tw-transition ${
                      !elbowUp
                        ? 'tw-border-purple-500 tw-bg-purple-950/70 tw-text-purple-100'
                        : 'tw-border-slate-600 tw-bg-slate-800 tw-text-slate-300 hover:tw-border-purple-400'
                    }`}
                  >
                    肘下 (elbow down)
                  </button>
                </div>
                <SliderCard
                  label="末端朝向 φ"
                  value={Number(rad2deg(analyticPhi).toFixed(0))}
                  min={-180}
                  max={180}
                  step={1}
                  onChange={(v) => setAnalyticPhi(deg2rad(v))}
                  accent="amber"
                  unit="°"
                />
              </>
            )}

            {/* page 3: λ + step */}
            {currentPage === 2 && (
              <div className="tw-grid tw-grid-cols-1 md:tw-grid-cols-2 tw-gap-2">
                <SliderCard
                  label="阻尼 λ"
                  value={Number(lambda.toFixed(3))}
                  min={0}
                  max={0.5}
                  step={0.005}
                  onChange={setLambda}
                  accent="cyan"
                />
                <SliderCard
                  label="步长 step"
                  value={Number(stepSize.toFixed(2))}
                  min={0.05}
                  max={1.0}
                  step={0.05}
                  onChange={setStepSize}
                  accent="emerald"
                />
              </div>
            )}

            {/* page 5: 轨迹模式 + 启停 */}
            {currentPage === 4 && (
              <div className="tw-flex tw-flex-wrap tw-items-center tw-gap-2">
                <button
                  type="button"
                  onClick={() => setTrajMode('circle')}
                  className={`tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-lg tw-border tw-font-medium tw-transition ${
                    trajMode === 'circle'
                      ? 'tw-border-emerald-500 tw-bg-emerald-950/70 tw-text-emerald-100'
                      : 'tw-border-slate-600 tw-bg-slate-800 tw-text-slate-300 hover:tw-border-emerald-400'
                  }`}
                >
                  画圆
                </button>
                <button
                  type="button"
                  onClick={() => setTrajMode('triangle')}
                  className={`tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-lg tw-border tw-font-medium tw-transition ${
                    trajMode === 'triangle'
                      ? 'tw-border-amber-500 tw-bg-amber-950/70 tw-text-amber-100'
                      : 'tw-border-slate-600 tw-bg-slate-800 tw-text-slate-300 hover:tw-border-amber-400'
                  }`}
                >
                  画三角形
                </button>
                <button
                  type="button"
                  onClick={() => setTrajRunning((v) => !v)}
                  className={`tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-lg tw-border tw-font-medium tw-transition ${
                    trajRunning
                      ? 'tw-border-rose-500 tw-bg-rose-950/70 tw-text-rose-100'
                      : 'tw-border-slate-600 tw-bg-slate-800 tw-text-slate-300 hover:tw-border-emerald-400'
                  }`}
                >
                  {trajRunning ? '暂停' : '开始'}
                </button>
                <button
                  type="button"
                  onClick={() => { traceRef.current = []; }}
                  className="tw-text-xs tw-bg-slate-800 tw-border tw-border-slate-600 tw-px-3 tw-py-1.5 tw-rounded-lg hover:tw-border-slate-400 tw-text-slate-300 tw-inline-flex tw-items-center tw-gap-1"
                >
                  <RotateCcw size={12} /> 清轨迹
                </button>
              </div>
            )}

            {/* 通用复位 */}
            {currentPage !== 1 && currentPage !== 4 && (
              <div className="tw-flex tw-justify-end">
                <button
                  type="button"
                  onClick={resetIK}
                  className="tw-text-xs tw-bg-slate-800 tw-border tw-border-slate-600 tw-px-3 tw-py-1.5 tw-rounded-lg hover:tw-border-rose-400 hover:tw-text-rose-300 hover:tw-bg-rose-950/60 tw-transition-all tw-text-slate-300 tw-inline-flex tw-items-center tw-gap-1"
                >
                  <RotateCcw size={12} /> 复位 IK
                </button>
              </div>
            )}
          </div>

          {/* 末端读数 */}
          <div className="tw-grid tw-grid-cols-2 md:tw-grid-cols-4 tw-gap-2 tw-flex-shrink-0">
            <ReadoutCard label="末端 x" value={fk.j3x.toFixed(1)} unit="px" accent="cyan" />
            <ReadoutCard label="末端 y" value={fk.j3y.toFixed(1)} unit="px" accent="sky" />
            <ReadoutCard label="目标距 r" value={targetDist.toFixed(1)} unit="px" accent={reachable ? 'emerald' : 'rose'} />
            <ReadoutCard label="‖e‖" value={errorMag.toFixed(2)} unit="px" accent={errorMag < 1.5 ? 'emerald' : 'amber'} />
          </div>
        </div>
      </div>
    </div>
  );
};

// ============================================================
// 子组件
// ============================================================

const ACCENTS = {
  cyan:    { ring: 'tw-text-cyan-300',    bg: 'tw-bg-cyan-900',    chipBg: 'tw-bg-cyan-950/60',    chipText: 'tw-text-cyan-200',    chipBorder: 'tw-border-cyan-800' },
  sky:     { ring: 'tw-text-sky-300',     bg: 'tw-bg-sky-900',     chipBg: 'tw-bg-sky-950/60',     chipText: 'tw-text-sky-200',     chipBorder: 'tw-border-sky-800' },
  emerald: { ring: 'tw-text-emerald-300', bg: 'tw-bg-emerald-900', chipBg: 'tw-bg-emerald-950/60', chipText: 'tw-text-emerald-200', chipBorder: 'tw-border-emerald-800' },
  amber:   { ring: 'tw-text-amber-300',   bg: 'tw-bg-amber-900',   chipBg: 'tw-bg-amber-950/60',   chipText: 'tw-text-amber-200',   chipBorder: 'tw-border-amber-800' },
  rose:    { ring: 'tw-text-rose-300',    bg: 'tw-bg-rose-900',    chipBg: 'tw-bg-rose-950/60',    chipText: 'tw-text-rose-200',    chipBorder: 'tw-border-rose-800' },
  purple:  { ring: 'tw-text-purple-300',  bg: 'tw-bg-purple-900',  chipBg: 'tw-bg-purple-950/60',  chipText: 'tw-text-purple-200',  chipBorder: 'tw-border-purple-800' },
};

const ACCENT_HEX = {
  cyan: '#06b6d4', sky: '#0ea5e9', emerald: '#10b981', amber: '#f59e0b', rose: '#f43f5e', purple: '#a855f7',
};

function SliderCard({ label, value, min, max, step = 1, onChange, accent = 'cyan', unit = '' }) {
  const a = ACCENTS[accent] || ACCENTS.cyan;
  return (
    <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm">
      <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
        <label className={`tw-font-bold tw-text-xs ${a.ring}`}>{label}</label>
        <span className={`tw-text-xs tw-font-mono tw-px-2 tw-py-0.5 tw-rounded tw-border ${a.chipBg} ${a.chipText} ${a.chipBorder}`}>
          {value}{unit}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className={`tw-w-full tw-h-2 tw-rounded-lg tw-appearance-none tw-cursor-pointer ${a.bg}`}
        style={{ accentColor: ACCENT_HEX[accent] }}
      />
    </div>
  );
}

function ReadoutCard({ label, value, unit, accent = 'cyan' }) {
  const a = ACCENTS[accent] || ACCENTS.cyan;
  return (
    <div className={`tw-p-2.5 tw-rounded-xl tw-border ${a.chipBorder} ${a.chipBg}`}>
      <div className={`tw-text-[11px] tw-mb-0.5 ${a.ring}`}>{label}</div>
      <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">
        {value}<span className="tw-text-slate-400 tw-text-xs tw-ml-1">{unit}</span>
      </div>
    </div>
  );
}

// 渲染一组幽灵机械臂 (页 2、4 的多解显示)
function GhostArm({ theta, color, opacity = 0.32 }) {
  if (!theta) return null;
  const fk = fkPlanar(theta[0], theta[1], theta[2]);
  return (
    <g opacity={opacity}>
      <line x1="0" y1="0" x2={fk.j1x} y2={fk.j1y} stroke={color} strokeWidth="10" strokeLinecap="round" />
      <line x1={fk.j1x} y1={fk.j1y} x2={fk.j2x} y2={fk.j2y} stroke={color} strokeWidth="8" strokeLinecap="round" />
      <line x1={fk.j2x} y1={fk.j2y} x2={fk.j3x} y2={fk.j3y} stroke={color} strokeWidth="6" strokeLinecap="round" />
      <circle cx={fk.j1x} cy={fk.j1y} r="6" fill={color} />
      <circle cx={fk.j2x} cy={fk.j2y} r="5" fill={color} />
      <circle cx={fk.j3x} cy={fk.j3y} r="5" fill={color} />
    </g>
  );
}

function ArmCanvas({
  page,
  svgRef,
  theta,
  target,
  fk,
  errorMag,
  reachable,
  analyticUp,
  analyticDown,
  analyticPhi,
  isDragging,
  onPointerDown,
  onPointerMove,
  onPointerUp,
  trace,
}) {
  const half = ARM_VIEW / 2;
  const { j1x, j1y, j2x, j2y, j3x, j3y } = fk;

  // 奇异判定:detJJT 接近 0 或目标距离接近 L_SUM
  const J = jacobianPlanar(theta[0], theta[1], theta[2]);
  const a = J[0][0] * J[0][0] + J[0][1] * J[0][1] + J[0][2] * J[0][2];
  const b = J[0][0] * J[1][0] + J[0][1] * J[1][1] + J[0][2] * J[1][2];
  const d = J[1][0] * J[1][0] + J[1][1] * J[1][1] + J[1][2] * J[1][2];
  const detJJT = a * d - b * b;
  const isSingular = detJJT < 5e4;

  const showGhosts = page === 1 || page === 3;

  return (
    <div className="tw-relative tw-flex-shrink-0">
      <svg
        ref={svgRef}
        viewBox={`-${half} -${half} ${ARM_VIEW} ${ARM_VIEW}`}
        preserveAspectRatio="xMidYMid meet"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        className={`tw-w-full tw-h-auto tw-max-w-[720px] tw-mx-auto tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm tw-touch-none ${
          isDragging ? 'tw-cursor-grabbing' : 'tw-cursor-crosshair'
        }`}
        style={{ aspectRatio: '1 / 1' }}
      >
        <g transform="scale(1, -1)">
          {/* 坐标轴 */}
          <line x1={-half} y1="0" x2={half} y2="0" stroke="#334155" strokeWidth="1.5" strokeDasharray="5,5" />
          <line x1="0" y1={-half} x2="0" y2={half} stroke="#334155" strokeWidth="1.5" strokeDasharray="5,5" />

          {/* 不可达红色阴影 (页 4) */}
          {page === 3 && !reachable && (
            <circle cx={target[0]} cy={target[1]} r="46" fill="rgba(244,63,94,0.18)" />
          )}

          {/* 工作空间外圈 + 内圈虚线 */}
          <circle cx="0" cy="0" r={L_SUM} stroke="#334155" strokeWidth="1.5" strokeDasharray="6,8" fill="none" opacity="0.6" />
          {L_INNER > 0 && (
            <circle cx="0" cy="0" r={L_INNER} stroke="#334155" strokeWidth="1" strokeDasharray="3,6" fill="none" opacity="0.45" />
          )}

          {/* 奇异线高亮 (页 4):紧贴外圈一圈 */}
          {page === 3 && (
            <circle cx="0" cy="0" r={L_SUM} stroke="#fb923c" strokeWidth="2.5" fill="none" opacity={isSingular ? 0.8 : 0.25} strokeDasharray="6,4" />
          )}

          {/* 末端轨迹 (页 5) */}
          {page === 4 && trace && trace.length > 1 && (
            <polyline
              points={trace.map((p) => `${p[0]},${p[1]}`).join(' ')}
              fill="none"
              stroke="#f43f5e"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              opacity="0.55"
              strokeDasharray="4,4"
            />
          )}

          {/* 双解幽灵 (页 2、4) */}
          {showGhosts && analyticUp.ok && (
            <GhostArm theta={analyticUp.theta} color="#22d3ee" opacity={0.32} />
          )}
          {showGhosts && analyticDown.ok && (
            <GhostArm theta={analyticDown.theta} color="#a855f7" opacity={0.32} />
          )}

          {/* 误差向量 (除页 2):从末端指向目标 */}
          {page !== 1 && (
            <line
              x1={j3x}
              y1={j3y}
              x2={target[0]}
              y2={target[1]}
              stroke="#fbbf24"
              strokeWidth="2"
              strokeDasharray="6,5"
              opacity="0.7"
            />
          )}

          {/* 当前机械臂三连杆 */}
          <line x1="0" y1="0" x2={j1x} y2={j1y} stroke="#3b82f6" strokeWidth="16" strokeLinecap="round" />
          <line x1={j1x} y1={j1y} x2={j2x} y2={j2y} stroke="#10b981" strokeWidth="13" strokeLinecap="round" />
          <line x1={j2x} y1={j2y} x2={j3x} y2={j3y} stroke="#a855f7" strokeWidth="10" strokeLinecap="round" />

          {/* 关节圆点 */}
          <circle cx="0" cy="0" r="14" fill="#e2e8f0" />
          <circle cx="0" cy="0" r="5" fill="#0f172a" />
          <circle cx={j1x} cy={j1y} r="12" fill="#e2e8f0" />
          <circle cx={j1x} cy={j1y} r="4.5" fill="#0f172a" />
          <circle cx={j2x} cy={j2y} r="11" fill="#e2e8f0" />
          <circle cx={j2x} cy={j2y} r="4" fill="#0f172a" />

          {/* 末端 (夹爪 + 圆) */}
          <g transform={`translate(${j3x}, ${j3y})`}>
            <path d="M 0,0 L 14,9 L 14,14" stroke="#94a3b8" strokeWidth="3" fill="none" strokeLinecap="round" />
            <path d="M 0,0 L 14,-9 L 14,-14" stroke="#94a3b8" strokeWidth="3" fill="none" strokeLinecap="round" />
            <circle cx="0" cy="0" r="7" fill="#0f172a" stroke="#cbd5e1" strokeWidth="2" />
          </g>

          {/* 目标十字 */}
          <g>
            <circle
              cx={target[0]}
              cy={target[1]}
              r="14"
              fill={reachable ? 'rgba(244,63,94,0.18)' : 'rgba(244,63,94,0.08)'}
              stroke={reachable ? '#f43f5e' : '#fda4af'}
              strokeWidth="2"
              strokeDasharray={reachable ? '0' : '4,3'}
            />
            <line x1={target[0] - 10} y1={target[1]} x2={target[0] + 10} y2={target[1]} stroke="#f43f5e" strokeWidth="2.5" />
            <line x1={target[0]} y1={target[1] - 10} x2={target[0]} y2={target[1] + 10} stroke="#f43f5e" strokeWidth="2.5" />
          </g>

          {/* 页 2:φ 朝向指示 */}
          {page === 1 && (
            <line
              x1={target[0]}
              y1={target[1]}
              x2={target[0] + 36 * Math.cos(analyticPhi)}
              y2={target[1] + 36 * Math.sin(analyticPhi)}
              stroke="#fbbf24"
              strokeWidth="3"
              strokeLinecap="round"
              opacity="0.95"
            />
          )}
        </g>

        {/* HUD */}
        <g className="tw-pointer-events-none">
          <rect x="14" y="14" width="220" height={page === 3 ? 78 : 56} rx="8" fill="rgba(15,23,42,0.86)" stroke="#334155" />
          <text x="26" y="38" fill="#67e8f9" fontSize="13" fontFamily="monospace" fontWeight="700">
            目标 ({target[0].toFixed(0)}, {target[1].toFixed(0)})
          </text>
          <text x="26" y="60" fill={errorMag < 1.5 ? '#34d399' : '#fbbf24'} fontSize="13" fontFamily="monospace" fontWeight="700">
            ‖e‖ = {errorMag.toFixed(2)} px
          </text>
          {page === 3 && (
            <text x="26" y="82" fill={isSingular ? '#fb923c' : '#94a3b8'} fontSize="12" fontFamily="monospace" fontWeight="700">
              {isSingular ? '⚠ SINGULAR' : 'detJJᵀ = ' + detJJT.toExponential(1)}
            </text>
          )}
        </g>

        {/* 状态徽章 */}
        <g className="tw-pointer-events-none">
          <rect
            x={ARM_VIEW - 178}
            y={ARM_VIEW - 48}
            width="160"
            height="30"
            rx="15"
            fill={!reachable ? '#f43f5e' : errorMag < 1.5 ? '#10b981' : '#0ea5e9'}
            opacity="0.95"
          />
          <text
            x={ARM_VIEW - 98}
            y={ARM_VIEW - 28}
            fill="#ffffff"
            fontSize="13"
            fontWeight="700"
            textAnchor="middle"
          >
            {!reachable ? '不可达 Unreachable' : errorMag < 1.5 ? '已收敛 Converged' : '迭代中 Solving'}
          </text>
        </g>
      </svg>

      {/* 提示 */}
      <div className="tw-absolute tw-top-3 tw-right-3 tw-bg-slate-800/90 tw-backdrop-blur-md tw-px-3 tw-py-1.5 tw-rounded-lg tw-shadow-md tw-border tw-border-slate-700 tw-pointer-events-none tw-flex tw-items-center tw-gap-1.5">
        <Crosshair size={12} className="tw-text-rose-300" />
        <span className="tw-text-[11px] tw-text-slate-300">拖动十字改目标</span>
      </div>
    </div>
  );
}

export default App;
