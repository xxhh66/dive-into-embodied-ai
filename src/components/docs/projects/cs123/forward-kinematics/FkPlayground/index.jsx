import React, { useEffect, useMemo, useRef, useState } from 'react';
import {
  Box,
  Calculator,
  ChevronDown,
  ChevronLeft,
  ChevronRight,
  Compass,
  Layers,
  PenTool,
  RotateCcw,
  Sigma,
} from 'lucide-react';

const PAGES = [
  { title: '位姿', icon: Compass, color: 'tw-text-cyan-400' },
  { title: '关节、连杆与末端', icon: Box, color: 'tw-text-blue-400' },
  { title: '运动链与齐次变换', icon: Layers, color: 'tw-text-amber-400' },
  { title: '工作空间散点图', icon: PenTool, color: 'tw-text-emerald-400' },
  { title: '雅可比矩阵', icon: Sigma, color: 'tw-text-rose-400' },
];

// ===== 机械臂连杆长度 (像素) =====
const L1 = 140;
const L2 = 110;
const L3 = 80;
const L_SUM = L1 + L2 + L3;

// ===== 视图常量 =====
const POSE_VIEW = 520;
const ARM_VIEW = 700;
const WS_SAMPLES = 4500;

const ARM_PRESETS = {
  home: { name: '初始姿态', t1: 45, t2: -30, t3: -45 },
  stretch: { name: '完全伸展', t1: 30, t2: 0, t3: 0 },
  zigzag: { name: 'Z 字折叠', t1: 120, t2: -120, t3: 120 },
  reach: { name: '抓取姿态', t1: 10, t2: 60, t3: -50 },
};

const deg2rad = (d) => (d * Math.PI) / 180;

// 输入度,返回从基座到三个关节末端的笛卡尔坐标 (画布坐标系下,后面 SVG 用 scale(1,-1) 翻 Y)
const computeArm = (theta1, theta2, theta3) => {
  const t1 = deg2rad(theta1);
  const t12 = deg2rad(theta1 + theta2);
  const t123 = deg2rad(theta1 + theta2 + theta3);
  const j1x = L1 * Math.cos(t1);
  const j1y = L1 * Math.sin(t1);
  const j2x = j1x + L2 * Math.cos(t12);
  const j2y = j1y + L2 * Math.sin(t12);
  const j3x = j2x + L3 * Math.cos(t123);
  const j3y = j2y + L3 * Math.sin(t123);
  return { j1x, j1y, j2x, j2y, j3x, j3y, t1, t12, t123 };
};

// 平面 3-DoF 雅可比 (位置 2x3)
const computeJacobian = (theta1, theta2, theta3) => {
  const t1 = deg2rad(theta1);
  const t12 = deg2rad(theta1 + theta2);
  const t123 = deg2rad(theta1 + theta2 + theta3);
  const s1 = Math.sin(t1);
  const c1 = Math.cos(t1);
  const s12 = Math.sin(t12);
  const c12 = Math.cos(t12);
  const s123 = Math.sin(t123);
  const c123 = Math.cos(t123);
  return [
    [-L1 * s1 - L2 * s12 - L3 * s123, -L2 * s12 - L3 * s123, -L3 * s123],
    [L1 * c1 + L2 * c12 + L3 * c123, L2 * c12 + L3 * c123, L3 * c123],
  ];
};

const App = () => {
  // 三个关节角 (页 2-5 共享)
  const [theta1, setTheta1] = useState(45);
  const [theta2, setTheta2] = useState(-30);
  const [theta3, setTheta3] = useState(-45);

  // 位姿演示状态 (页 1)
  const [poseX, setPoseX] = useState(120);
  const [poseY, setPoseY] = useState(80);
  const [poseTheta, setPoseTheta] = useState(35);

  // 末端轨迹 (页 4 工作空间显示)
  const [trajectory, setTrajectory] = useState([]);

  // 分页
  const [currentPage, setCurrentPage] = useState(0);
  const totalPages = PAGES.length;

  // 页面跳转下拉
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const dropdownRef = useRef(null);

  useEffect(() => {
    if (!dropdownOpen) return;
    const onDocClick = (e) => {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target)) {
        setDropdownOpen(false);
      }
    };
    const onEsc = (e) => {
      if (e.key === 'Escape') setDropdownOpen(false);
    };
    document.addEventListener('mousedown', onDocClick);
    document.addEventListener('keydown', onEsc);
    return () => {
      document.removeEventListener('mousedown', onDocClick);
      document.removeEventListener('keydown', onEsc);
    };
  }, [dropdownOpen]);

  // ===== 派生量 =====
  const arm = computeArm(theta1, theta2, theta3);
  const phi = theta1 + theta2 + theta3;
  const jacobian = useMemo(
    () => computeJacobian(theta1, theta2, theta3),
    [theta1, theta2, theta3],
  );

  // 工作空间点云 (Monte Carlo, 只算一次)
  const workspacePts = useMemo(() => {
    const pts = [];
    for (let i = 0; i < WS_SAMPLES; i++) {
      const a1 = Math.random() * 2 * Math.PI - Math.PI;
      const a2 = Math.random() * 2 * Math.PI - Math.PI;
      const a3 = Math.random() * 2 * Math.PI - Math.PI;
      const x =
        L1 * Math.cos(a1) +
        L2 * Math.cos(a1 + a2) +
        L3 * Math.cos(a1 + a2 + a3);
      const y =
        L1 * Math.sin(a1) +
        L2 * Math.sin(a1 + a2) +
        L3 * Math.sin(a1 + a2 + a3);
      pts.push([x, y]);
    }
    return pts;
  }, []);

  // 末端轨迹追踪
  useEffect(() => {
    setTrajectory((prev) => {
      const next = [...prev, [arm.j3x, arm.j3y]];
      return next.slice(-160);
    });
  }, [arm.j3x, arm.j3y]);

  // 位姿矩阵
  const poseRad = deg2rad(poseTheta);
  const poseCos = Math.cos(poseRad);
  const poseSin = Math.sin(poseRad);
  const poseAxisLen = 80;

  // ===== 操作 =====
  const applyPreset = (key) => {
    const p = ARM_PRESETS[key];
    if (!p) return;
    setTheta1(p.t1);
    setTheta2(p.t2);
    setTheta3(p.t3);
  };

  const resetPose = () => {
    setPoseX(120);
    setPoseY(80);
    setPoseTheta(35);
  };

  const clearTrajectory = () => setTrajectory([]);

  const nextPage = () => setCurrentPage((p) => Math.min(totalPages - 1, p + 1));
  const prevPage = () => setCurrentPage((p) => Math.max(0, p - 1));

  // ===== 通用控制面板 (3 个关节滑块,页 2-5 共用) =====
  const renderJointControls = (compact = false) => (
    <div className={compact ? 'tw-grid tw-grid-cols-1 md:tw-grid-cols-3 tw-gap-2' : 'tw-space-y-3'}>
      <SliderCard
        label="θ₁ · 基座关节"
        value={theta1}
        min={0}
        max={180}
        step={1}
        onChange={setTheta1}
        accent="blue"
        unit="°"
      />
      <SliderCard
        label="θ₂ · 中段关节"
        value={theta2}
        min={-180}
        max={180}
        step={1}
        onChange={setTheta2}
        accent="emerald"
        unit="°"
      />
      <SliderCard
        label="θ₃ · 末端关节"
        value={theta3}
        min={-180}
        max={180}
        step={1}
        onChange={setTheta3}
        accent="purple"
        unit="°"
      />
    </div>
  );

  const renderPresetButtons = () => (
    <div className="tw-flex tw-flex-wrap tw-gap-2">
      {Object.entries(ARM_PRESETS).map(([key, p]) => (
        <button
          key={key}
          onClick={() => applyPreset(key)}
          className="tw-text-xs tw-bg-slate-800 tw-border tw-border-slate-600 tw-px-3 tw-py-1.5 tw-rounded-lg hover:tw-border-blue-400 hover:tw-text-blue-300 hover:tw-bg-blue-950/60 tw-transition-all tw-font-medium tw-text-slate-300"
        >
          {p.name}
        </button>
      ))}
      <button
        onClick={clearTrajectory}
        className="tw-text-xs tw-bg-slate-800 tw-border tw-border-slate-600 tw-px-3 tw-py-1.5 tw-rounded-lg hover:tw-border-rose-400 hover:tw-text-rose-300 hover:tw-bg-rose-950/60 tw-transition-all tw-font-medium tw-text-slate-300 tw-inline-flex tw-items-center tw-gap-1"
        title="清除末端轨迹"
        type="button"
      >
        <RotateCcw size={12} /> 清除轨迹
      </button>
    </div>
  );

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
                            ? 'tw-bg-cyan-600 tw-text-white'
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
                : 'tw-text-white tw-bg-cyan-600 hover:tw-bg-cyan-500 active:tw-scale-95 tw-shadow-sm'
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

          {/* Page 1: 位姿 */}
          {currentPage === 0 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Compass className="tw-mr-3 tw-text-cyan-400" size={24} />
                1. 位姿
              </h2>

              <div className="tw-bg-cyan-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-cyan-900/60">
                <p className="tw-mb-2">
                  正运动学的输出不是「一个点」，而是「位姿」。位姿 = <strong>位置</strong> + <strong>姿态</strong>，统一用一个齐次变换矩阵描述：
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-cyan-200 tw-leading-loose">
                  T = [ R   p ; 0   1 ] ∈ SE(2 / 3)
                </div>
                <p className="tw-text-xs tw-text-slate-300 tw-mt-2">
                  右侧三个滑块直接控制一个 2D 末端坐标系 {'{E}'}：x、y 改变位置，θ 改变姿态。注意右上角的矩阵随之实时变化。
                </p>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-cyan-300 tw-text-sm tw-mb-1.5">位置 (Position)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    回答「末端原点在哪」。即矩阵最后一列 p = [x, y]ᵀ，对应右图橙色虚线箭头从 O_B 指向 O_E。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1.5">姿态 (Orientation)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    回答「末端坐标轴朝哪」。2D 中退化成一个角 θ；矩阵左上 2×2 就是旋转矩阵 R = [cosθ, −sinθ; sinθ, cosθ]。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1.5">为什么要合在一起</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    把位置和姿态压成一个 4×4（或 2D 下 3×3）矩阵的好处是：<strong>多个连杆的变换可以一路相乘</strong>。下一页就开始把它们串成机械臂。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700">
                💡 拖右侧滑块时，注意红/绿色末端坐标轴 {'{E}'} 是如何被「先平移到 (x, y)，再绕原点转 θ」的。
              </div>
            </div>
          )}

          {/* Page 2: 关节、连杆、末端 */}
          {currentPage === 1 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Box className="tw-mr-3 tw-text-blue-400" size={24} />
                2. 关节、连杆与末端执行器
              </h2>

              <div className="tw-bg-blue-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-blue-900/60">
                <p className="tw-mb-2">
                  机械臂由三类元素串成：<strong>关节 (Joint)</strong> 提供自由度，<strong>连杆 (Link)</strong> 是关节之间的刚体段，<strong>末端执行器 (End Effector)</strong> 就是最后一个连杆。
                </p>
                <p className="tw-text-xs tw-text-slate-300">
                  正运动学就是这样一个映射：FK : (θ₁, θ₂, …, θₙ) → 末端位姿。右侧 3-DoF 平面臂演示了这个映射。
                </p>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-blue-950/30 tw-border tw-border-blue-900/60 tw-rounded-xl tw-p-4">
                  <h3 className="tw-font-bold tw-text-blue-200 tw-text-sm tw-mb-1.5">关节 (Joint)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    本章默认讨论<strong>旋转关节</strong>，状态由关节角 θᵢ 描述。右图蓝、绿、紫三个圆点就是 3 个旋转关节。
                  </p>
                </div>
                <div className="tw-bg-emerald-950/30 tw-border tw-border-emerald-900/60 tw-rounded-xl tw-p-4">
                  <h3 className="tw-font-bold tw-text-emerald-200 tw-text-sm tw-mb-1.5">连杆 (Link)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    连杆是刚体，不会形变。它的位姿完全由两端关节决定。右图三段彩色线段就是 L₁、L₂、L₃。
                  </p>
                </div>
                <div className="tw-bg-rose-950/30 tw-border tw-border-rose-900/60 tw-rounded-xl tw-p-4">
                  <h3 className="tw-font-bold tw-text-rose-200 tw-text-sm tw-mb-1.5">末端执行器 (End Effector)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    本质上就是<strong>最后一个连杆</strong>，下游任务（抓取、装配、视觉对齐）只关心它的位姿。右图末端的红色点就是 {'{E}'} 原点。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700">
                💡 拖动 θ₁/θ₂/θ₃，注意末端的 (x, y, φ) 三个数字是怎么响应的——φ = θ₁+θ₂+θ₃ 就是平面臂的末端朝向。
              </div>
            </div>
          )}

          {/* Page 3: 运动链与齐次变换 */}
          {currentPage === 2 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Layers className="tw-mr-3 tw-text-amber-400" size={24} />
                3. 运动链与齐次变换
              </h2>

              <div className="tw-bg-amber-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-amber-900/60">
                <p className="tw-mb-2">
                  把基座、各连杆、末端按「父→子」串成树，就是<strong>运动链</strong>。每个关节贡献一个齐次变换，末端位姿就是这些变换的乘积：
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-amber-200 tw-leading-loose">
                  T₀³ = R<sub>z</sub>(θ₁)·Trans<sub>x</sub>(L₁)·R<sub>z</sub>(θ₂)·Trans<sub>x</sub>(L₂)·R<sub>z</sub>(θ₃)·Trans<sub>x</sub>(L₃)
                </div>
                <p className="tw-text-xs tw-text-slate-300 tw-mt-2">
                  这就是 URDF / MJCF 描述：每个 <code className="tw-text-amber-200">{'<joint>'}</code> 提供一个固定偏置 + 一个旋转，写代码时直接读 URDF 的参数即可。
                </p>
              </div>

              <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-2">展开三角投影</h3>
                <div className="tw-font-mono tw-text-[12px] tw-text-slate-300 tw-leading-loose tw-space-y-0.5">
                  <div>x = L₁cos(θ₁) + L₂cos(θ₁+θ₂) + L₃cos(θ₁+θ₂+θ₃)</div>
                  <div>y = L₁sin(θ₁) + L₂sin(θ₁+θ₂) + L₃sin(θ₁+θ₂+θ₃)</div>
                  <div>φ = θ₁ + θ₂ + θ₃</div>
                </div>
                <div className="tw-mt-3 tw-pt-3 tw-border-t tw-border-slate-700 tw-text-xs tw-text-slate-400 tw-leading-relaxed">
                  把当前角度代入：
                </div>
                <div className="tw-font-mono tw-text-[12px] tw-text-cyan-200 tw-leading-loose tw-mt-1">
                  <div>x = {(arm.j3x).toFixed(2)}</div>
                  <div>y = {(arm.j3y).toFixed(2)}</div>
                  <div>φ = {phi.toFixed(1)}°</div>
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-blue-300 tw-text-sm tw-mb-1">关键性质 1：可以一路乘下去</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">n 个关节的末端位姿就是 n 个齐次矩阵相乘，这一乘法正是 FK 的全部计算内容。</p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1">关键性质 2：可逆且正交</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">T⁻¹ = [Rᵀ, −Rᵀp; 0, 1]。把世界坐标系下的目标搬到末端坐标系时反复用到。</p>
                </div>
              </div>
            </div>
          )}

          {/* Page 4: 工作空间散点图 */}
          {currentPage === 3 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <PenTool className="tw-mr-3 tw-text-emerald-400" size={24} />
                4. 工作空间散点图
              </h2>

              <div className="tw-bg-emerald-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-emerald-900/60">
                <p className="tw-mb-2">
                  <strong>工作空间</strong>是末端能到达的所有点的集合。用 FK 做一次 Monte Carlo 就能把它画出来：均匀采样关节角，把每组角的末端位置画成一个点。
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-emerald-200 tw-leading-loose">
                  for _ in range(20000):<br />
                  &nbsp;&nbsp;q = uniform(−π, π, 3)<br />
                  &nbsp;&nbsp;pts.append( fk(q)[:2] )
                </div>
                <p className="tw-text-xs tw-text-slate-300 tw-mt-2">
                  右图淡绿色点云就是 {WS_SAMPLES} 次采样结果——形状像一个<strong>甜甜圈</strong>。
                </p>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1.5">外圈</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    最大可达半径 = L₁ + L₂ + L₃ = {L_SUM} px。三节连杆完全伸直时画出。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-rose-300 tw-text-sm tw-mb-1.5">为什么有用</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    第 3 章选 IK 目标时这张图会派上用场——<strong>目标点落在甜甜圈外</strong>就是典型的「无解」，再好的 IK 算法也救不了。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700">
                💡 拖动滑块让末端在工作空间里走一圈，红色折线是末端最近 160 帧的轨迹。
              </div>
            </div>
          )}

          {/* Page 5: 雅可比 */}
          {currentPage === 4 && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Sigma className="tw-mr-3 tw-text-rose-400" size={24} />
                5. 雅可比矩阵
              </h2>

              <div className="tw-bg-rose-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-rose-900/60">
                <p className="tw-mb-2">
                  FK 把关节角映射到末端位姿。对时间求导，就得到<strong>雅可比矩阵</strong>——它描述「关节角怎么动，末端怎么动」：
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-rose-200 tw-leading-loose">
                  ṗ = ∂f/∂θ · θ̇ = J(θ) · θ̇
                </div>
                <p className="tw-text-xs tw-text-slate-300 tw-mt-2">
                  对平面 3-DoF 臂，J 是 2×3。右图三条彩色箭头就是 J 的三列——<strong>各自表示「单独转某一个关节」时，末端会朝哪个方向走</strong>。
                </p>
              </div>

              <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                <h3 className="tw-font-bold tw-text-rose-300 tw-text-sm tw-mb-2 tw-flex tw-items-center">
                  <Calculator size={14} className="tw-mr-1.5" />
                  当前 J(θ)
                </h3>
                <div className="tw-font-mono tw-text-[12px] tw-text-slate-200 tw-leading-loose tw-overflow-x-auto">
                  <div>
                    [ <span className="tw-text-blue-300">{jacobian[0][0].toFixed(1).padStart(7, ' ')}</span>,
                    {' '}<span className="tw-text-emerald-300">{jacobian[0][1].toFixed(1).padStart(7, ' ')}</span>,
                    {' '}<span className="tw-text-purple-300">{jacobian[0][2].toFixed(1).padStart(7, ' ')}</span> ]
                  </div>
                  <div>
                    [ <span className="tw-text-blue-300">{jacobian[1][0].toFixed(1).padStart(7, ' ')}</span>,
                    {' '}<span className="tw-text-emerald-300">{jacobian[1][1].toFixed(1).padStart(7, ' ')}</span>,
                    {' '}<span className="tw-text-purple-300">{jacobian[1][2].toFixed(1).padStart(7, ' ')}</span> ]
                  </div>
                </div>
                <div className="tw-text-[11px] tw-text-slate-500 tw-mt-2 tw-leading-relaxed">
                  第 i 列对应「关节 i 转 1 rad」时末端的瞬时位移方向（单位 px）。
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1">几何雅可比</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    第 i 列的线速度部分 = ωᵢ × (p<sub>end</sub> − pᵢ)。这正是右图箭头的方向：从关节 i 到末端的向量旋转 90°。
                  </p>
                </div>
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1">为什么是桥</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    雅可比让<strong>速度、力、能量</strong>都能用同一套代数写出来。第 3 章数值 IK、第 5 章接触力、第 6 章 RL 奖励里都会反复见到它。
                  </p>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* ===== 右侧:可视化画布 ===== */}
      <div className="tw-w-full md:tw-w-1/2 lg:tw-w-7/12 tw-bg-slate-900/60 tw-backdrop-blur-sm tw-flex tw-flex-col tw-gap-3 tw-p-4 md:tw-p-6 tw-relative tw-overflow-hidden tw-shadow-inner">

        {/* 网格背景 */}
        <div
          className="tw-absolute tw-inset-0 tw-pointer-events-none tw-opacity-[0.12]"
          style={{
            backgroundImage: 'linear-gradient(#64748b 1px, transparent 1px), linear-gradient(90deg, #64748b 1px, transparent 1px)',
            backgroundSize: '40px 40px',
          }}
        />

        {/* === 页 1:位姿演示 === */}
        {currentPage === 0 && (
          <div className="tw-flex tw-flex-col tw-gap-3 tw-z-10 tw-min-h-0 tw-overflow-y-auto">
            <div className="tw-relative tw-flex-shrink-0">
              <svg
                viewBox={`-${POSE_VIEW / 2} -${POSE_VIEW / 2} ${POSE_VIEW} ${POSE_VIEW}`}
                className="tw-w-full tw-h-auto tw-max-w-[640px] tw-mx-auto tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                style={{ aspectRatio: '1 / 1' }}
              >
                <g transform="scale(1, -1)">
                  <line x1={-POSE_VIEW / 2} y1="0" x2={POSE_VIEW / 2} y2="0" stroke="#475569" strokeWidth="1.5" strokeDasharray="5,5" opacity="0.55" />
                  <line x1="0" y1={-POSE_VIEW / 2} x2="0" y2={POSE_VIEW / 2} stroke="#475569" strokeWidth="1.5" strokeDasharray="5,5" opacity="0.55" />

                  {/* 位置向量 p */}
                  <line x1="0" y1="0" x2={poseX} y2={poseY} stroke="#f59e0b" strokeWidth="3" strokeDasharray="8,6" strokeLinecap="round" />

                  {/* 基座坐标轴 */}
                  <line x1="0" y1="0" x2={poseAxisLen} y2="0" stroke="#fca5a5" strokeWidth="3" strokeLinecap="round" />
                  <line x1="0" y1="0" x2="0" y2={poseAxisLen} stroke="#86efac" strokeWidth="3" strokeLinecap="round" />

                  {/* 末端坐标轴 (绕 (poseX, poseY) 转 θ) */}
                  <line
                    x1={poseX}
                    y1={poseY}
                    x2={poseX + poseAxisLen * poseCos}
                    y2={poseY + poseAxisLen * poseSin}
                    stroke="#ef4444"
                    strokeWidth="4"
                    strokeLinecap="round"
                  />
                  <line
                    x1={poseX}
                    y1={poseY}
                    x2={poseX - poseAxisLen * poseSin}
                    y2={poseY + poseAxisLen * poseCos}
                    stroke="#22c55e"
                    strokeWidth="4"
                    strokeLinecap="round"
                  />

                  <circle cx="0" cy="0" r="9" fill="#e2e8f0" />
                  <circle cx="0" cy="0" r="3" fill="#0f172a" />
                  <circle cx={poseX} cy={poseY} r="10" fill="#ffffff" stroke="#10b981" strokeWidth="3" />
                  <circle cx={poseX} cy={poseY} r="3" fill="#0f172a" />
                </g>

                {/* 标签 (不翻转 Y) */}
                <g className="tw-pointer-events-none">
                  <text x={poseAxisLen + 6} y="-8" fill="#fca5a5" fontSize="13" fontWeight="700">x_B</text>
                  <text x="-22" y={-(poseAxisLen + 8)} fill="#86efac" fontSize="13" fontWeight="700">y_B</text>
                  <text x={poseX + poseAxisLen * poseCos + 8} y={-(poseY + poseAxisLen * poseSin) + 4} fill="#ef4444" fontSize="13" fontWeight="700">x_E</text>
                  <text x={poseX - poseAxisLen * poseSin + 8} y={-(poseY + poseAxisLen * poseCos) + 4} fill="#22c55e" fontSize="13" fontWeight="700">y_E</text>
                  <text x="-18" y="22" fill="#e2e8f0" fontSize="14" fontWeight="700">O_B</text>
                  <text x={poseX + 12} y={-poseY + 18} fill="#e2e8f0" fontSize="14" fontWeight="700">O_E</text>
                  <text x={poseX / 2 + 8} y={-poseY / 2 - 6} fill="#f59e0b" fontSize="15" fontWeight="700">p</text>
                </g>
              </svg>

              {/* 矩阵 HUD */}
              <div className="tw-absolute tw-top-3 tw-right-3 tw-bg-slate-800/95 tw-backdrop-blur-md tw-px-4 tw-py-2.5 tw-rounded-lg tw-shadow-lg tw-border tw-border-slate-700 tw-pointer-events-none">
                <div className="tw-text-[10px] tw-font-bold tw-text-slate-500 tw-mb-1 tw-border-b tw-border-slate-700 tw-pb-1">齐次变换矩阵 T</div>
                <div className="tw-font-mono tw-text-[11px] tw-leading-5">
                  <div className="tw-text-slate-200">[ {poseCos.toFixed(2).padStart(5)}, {(-poseSin).toFixed(2).padStart(5)}, <span className="tw-text-amber-300">{poseX.toFixed(0).padStart(4)}</span> ]</div>
                  <div className="tw-text-slate-200">[ {poseSin.toFixed(2).padStart(5)}, {poseCos.toFixed(2).padStart(5)}, <span className="tw-text-amber-300">{poseY.toFixed(0).padStart(4)}</span> ]</div>
                  <div className="tw-text-slate-200">[  0.00,  0.00,    1 ]</div>
                </div>
              </div>
            </div>

            {/* 滑块 */}
            <div className="tw-grid tw-grid-cols-1 md:tw-grid-cols-3 tw-gap-2 tw-flex-shrink-0">
              <SliderCard label="平移 x" value={poseX} min={-200} max={200} step={1} onChange={setPoseX} accent="cyan" />
              <SliderCard label="平移 y" value={poseY} min={-200} max={200} step={1} onChange={setPoseY} accent="sky" />
              <SliderCard label="旋转 θ" value={poseTheta} min={-180} max={180} step={1} onChange={setPoseTheta} accent="emerald" unit="°" />
            </div>

            <div className="tw-flex tw-justify-end">
              <button
                onClick={resetPose}
                type="button"
                className="tw-text-xs tw-bg-slate-800 tw-border tw-border-slate-600 tw-px-3 tw-py-1.5 tw-rounded-lg hover:tw-border-cyan-400 hover:tw-text-cyan-300 hover:tw-bg-cyan-950/60 tw-transition-all tw-font-medium tw-text-slate-300 tw-inline-flex tw-items-center tw-gap-1"
              >
                <RotateCcw size={12} /> 复位位姿
              </button>
            </div>
          </div>
        )}

        {/* === 页 2-5:3-DoF 平面臂演示 === */}
        {currentPage > 0 && (
          <div className="tw-flex tw-flex-col tw-gap-3 tw-z-10 tw-min-h-0 tw-overflow-y-auto">
            <ArmCanvas
              page={currentPage}
              arm={arm}
              phi={phi}
              jacobian={jacobian}
              workspacePts={workspacePts}
              trajectory={trajectory}
            />

            <div className="tw-flex-shrink-0 tw-space-y-2">
              {renderPresetButtons()}
              {renderJointControls(true)}
            </div>

            {/* 末端读数面板 */}
            <div className="tw-grid tw-grid-cols-3 tw-gap-2 tw-flex-shrink-0">
              <ReadoutCard label="末端 x" value={arm.j3x.toFixed(1)} unit="px" accent="cyan" />
              <ReadoutCard label="末端 y" value={arm.j3y.toFixed(1)} unit="px" accent="sky" />
              <ReadoutCard label="朝向 φ" value={phi.toFixed(1)} unit="°" accent="emerald" />
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

// ====== 子组件 ======

const ACCENTS = {
  cyan:    { ring: 'tw-text-cyan-300',    bg: 'tw-bg-cyan-900',    chipBg: 'tw-bg-cyan-950/60',    chipText: 'tw-text-cyan-200',    chipBorder: 'tw-border-cyan-800' },
  sky:     { ring: 'tw-text-sky-300',     bg: 'tw-bg-sky-900',     chipBg: 'tw-bg-sky-950/60',     chipText: 'tw-text-sky-200',     chipBorder: 'tw-border-sky-800' },
  emerald: { ring: 'tw-text-emerald-300', bg: 'tw-bg-emerald-900', chipBg: 'tw-bg-emerald-950/60', chipText: 'tw-text-emerald-200', chipBorder: 'tw-border-emerald-800' },
  blue:    { ring: 'tw-text-blue-300',    bg: 'tw-bg-blue-900',    chipBg: 'tw-bg-blue-950/60',    chipText: 'tw-text-blue-200',    chipBorder: 'tw-border-blue-800' },
  purple:  { ring: 'tw-text-purple-300',  bg: 'tw-bg-purple-900',  chipBg: 'tw-bg-purple-950/60',  chipText: 'tw-text-purple-200',  chipBorder: 'tw-border-purple-800' },
  rose:    { ring: 'tw-text-rose-300',    bg: 'tw-bg-rose-900',    chipBg: 'tw-bg-rose-950/60',    chipText: 'tw-text-rose-200',    chipBorder: 'tw-border-rose-800' },
};

const ACCENT_COLOR_HEX = {
  cyan: '#06b6d4',
  sky: '#0ea5e9',
  emerald: '#10b981',
  blue: '#3b82f6',
  purple: '#a855f7',
  rose: '#f43f5e',
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
        style={{ accentColor: ACCENT_COLOR_HEX[accent] }}
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

// 3-DoF 平面臂画布,根据 page 切换叠加层
function ArmCanvas({ page, arm, phi, jacobian, workspacePts, trajectory }) {
  const half = ARM_VIEW / 2;
  const { j1x, j1y, j2x, j2y, j3x, j3y, t1, t12 } = arm;

  // 雅可比箭头 (第 i 列代表关节 i 转动一弧度时末端的瞬时位移)
  // 缩放至画面可见
  const JACO_SCALE = 1.0;

  return (
    <div className="tw-relative tw-flex-shrink-0">
      <svg
        viewBox={`-${half} -${half} ${ARM_VIEW} ${ARM_VIEW}`}
        className="tw-w-full tw-h-auto tw-max-w-[720px] tw-mx-auto tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
        style={{ aspectRatio: '1 / 1' }}
      >
        {/* 反转 Y 让上为正 */}
        <g transform="scale(1, -1)">
          {/* 坐标轴 */}
          <line x1={-half} y1="0" x2={half} y2="0" stroke="#334155" strokeWidth="1.5" strokeDasharray="5,5" />
          <line x1="0" y1={-half} x2="0" y2={half} stroke="#334155" strokeWidth="1.5" strokeDasharray="5,5" />

          {/* 工作空间外圈 */}
          <circle cx="0" cy="0" r={L_SUM} stroke="#334155" strokeWidth="1.5" strokeDasharray="6,8" fill="none" opacity="0.6" />

          {/* === 页 4:工作空间点云 === */}
          {page === 3 && (
            <g opacity="0.55">
              {workspacePts.map((p, i) => (
                <circle key={i} cx={p[0]} cy={p[1]} r="1.4" fill="#34d399" />
              ))}
            </g>
          )}

          {/* === 页 4:末端轨迹 === */}
          {page === 3 && trajectory.length > 1 && (
            <polyline
              points={trajectory.map((p) => `${p[0]},${p[1]}`).join(' ')}
              fill="none"
              stroke="#f43f5e"
              strokeWidth="2.5"
              strokeLinecap="round"
              strokeLinejoin="round"
              opacity="0.7"
            />
          )}

          {/* === 页 3:中间帧坐标轴 === */}
          {page === 2 && (
            <g opacity="0.95">
              {/* {L1} 帧 */}
              <line x1={j1x} y1={j1y} x2={j1x + 30 * Math.cos(t1)} y2={j1y + 30 * Math.sin(t1)} stroke="#fca5a5" strokeWidth="2.5" strokeLinecap="round" />
              <line x1={j1x} y1={j1y} x2={j1x - 30 * Math.sin(t1)} y2={j1y + 30 * Math.cos(t1)} stroke="#86efac" strokeWidth="2.5" strokeLinecap="round" />
              {/* {L2} 帧 */}
              <line x1={j2x} y1={j2y} x2={j2x + 30 * Math.cos(t12)} y2={j2y + 30 * Math.sin(t12)} stroke="#fca5a5" strokeWidth="2.5" strokeLinecap="round" />
              <line x1={j2x} y1={j2y} x2={j2x - 30 * Math.sin(t12)} y2={j2y + 30 * Math.cos(t12)} stroke="#86efac" strokeWidth="2.5" strokeLinecap="round" />
            </g>
          )}

          {/* 三段连杆 */}
          <line x1="0" y1="0" x2={j1x} y2={j1y} stroke="#3b82f6" strokeWidth="16" strokeLinecap="round" />
          <line x1={j1x} y1={j1y} x2={j2x} y2={j2y} stroke="#10b981" strokeWidth="13" strokeLinecap="round" />
          <line x1={j2x} y1={j2y} x2={j3x} y2={j3y} stroke="#a855f7" strokeWidth="10" strokeLinecap="round" />

          {/* 关节 */}
          <circle cx="0" cy="0" r="14" fill="#e2e8f0" />
          <circle cx="0" cy="0" r="5" fill="#0f172a" />
          <circle cx="0" cy="0" r="2" fill="#3b82f6" />

          <circle cx={j1x} cy={j1y} r="12" fill="#e2e8f0" />
          <circle cx={j1x} cy={j1y} r="4.5" fill="#0f172a" />
          <circle cx={j1x} cy={j1y} r="2" fill="#10b981" />

          <circle cx={j2x} cy={j2y} r="11" fill="#e2e8f0" />
          <circle cx={j2x} cy={j2y} r="4" fill="#0f172a" />
          <circle cx={j2x} cy={j2y} r="2" fill="#a855f7" />

          {/* 末端 (夹爪 + 圆点) */}
          <g transform={`translate(${j3x}, ${j3y})`}>
            <path d="M 0,0 L 14,9 L 14,14" stroke="#94a3b8" strokeWidth="3" fill="none" strokeLinecap="round" />
            <path d="M 0,0 L 14,-9 L 14,-14" stroke="#94a3b8" strokeWidth="3" fill="none" strokeLinecap="round" />
            <circle cx="0" cy="0" r="7" fill="#f43f5e" />
            <circle cx="0" cy="0" r="2.5" fill="#ffffff" />
          </g>

          {/* === 页 5:雅可比三列箭头 === */}
          {page === 4 && (
            <g>
              {[0, 1, 2].map((col) => {
                const dx = jacobian[0][col] * JACO_SCALE;
                const dy = jacobian[1][col] * JACO_SCALE;
                const len = Math.hypot(dx, dy);
                if (len < 1) return null;
                const ux = dx / len;
                const uy = dy / len;
                const colors = ['#3b82f6', '#10b981', '#a855f7'];
                const ex = j3x + dx;
                const ey = j3y + dy;
                const headLen = 12;
                const headW = 6;
                return (
                  <g key={col}>
                    <line
                      x1={j3x}
                      y1={j3y}
                      x2={ex}
                      y2={ey}
                      stroke={colors[col]}
                      strokeWidth="3.5"
                      strokeLinecap="round"
                      opacity="0.9"
                    />
                    {/* 箭头三角 */}
                    <polygon
                      points={`${ex},${ey} ${ex - headLen * ux + headW * uy},${ey - headLen * uy - headW * ux} ${ex - headLen * ux - headW * uy},${ey - headLen * uy + headW * ux}`}
                      fill={colors[col]}
                      opacity="0.95"
                    />
                  </g>
                );
              })}
            </g>
          )}

          {/* 末端到原点连线 */}
          <line x1="0" y1="0" x2={j3x} y2={j3y} stroke="#f43f5e" strokeWidth="1" strokeDasharray="3,6" opacity="0.3" />
        </g>

        {/* HUD: 当前末端 */}
        <g className="tw-pointer-events-none">
          <rect x="14" y="14" width="178" height="56" rx="8" fill="rgba(15,23,42,0.86)" stroke="#334155" />
          <text x="26" y="38" fill="#67e8f9" fontSize="13" fontFamily="monospace" fontWeight="700">
            末端 ({arm.j3x.toFixed(0)}, {arm.j3y.toFixed(0)})
          </text>
          <text x="26" y="60" fill="#86efac" fontSize="13" fontFamily="monospace" fontWeight="700">
            φ = {phi.toFixed(1)}°
          </text>
        </g>

        {/* 页 2:连杆/关节标注 */}
        {page === 1 && (
          <g className="tw-pointer-events-none">
            <text x={(j1x) / 2} y={-(j1y) / 2 - 12} fill="#60a5fa" fontSize="13" fontWeight="700" textAnchor="middle">L₁</text>
            <text x={(j1x + j2x) / 2} y={-(j1y + j2y) / 2 - 12} fill="#34d399" fontSize="13" fontWeight="700" textAnchor="middle">L₂</text>
            <text x={(j2x + j3x) / 2} y={-(j2y + j3y) / 2 - 12} fill="#c084fc" fontSize="13" fontWeight="700" textAnchor="middle">L₃</text>
            <text x={j3x + 12} y={-j3y - 8} fill="#fda4af" fontSize="13" fontWeight="700">{'{E}'}</text>
            <text x="-22" y="-22" fill="#cbd5e1" fontSize="13" fontWeight="700">{'{B}'}</text>
          </g>
        )}

        {/* 页 3:中间帧标注 */}
        {page === 2 && (
          <g className="tw-pointer-events-none">
            <text x="-26" y="-22" fill="#cbd5e1" fontSize="12" fontWeight="700">{'{B}'}</text>
            <text x={j1x + 8} y={-j1y - 8} fill="#cbd5e1" fontSize="12" fontWeight="700">{'{L₁}'}</text>
            <text x={j2x + 8} y={-j2y - 8} fill="#cbd5e1" fontSize="12" fontWeight="700">{'{L₂}'}</text>
            <text x={j3x + 12} y={-j3y - 8} fill="#fda4af" fontSize="12" fontWeight="700">{'{E}'}</text>
          </g>
        )}

        {/* 页 5:Jacobian 列标签 */}
        {page === 4 && (
          <g className="tw-pointer-events-none">
            {[0, 1, 2].map((col) => {
              const dx = jacobian[0][col];
              const dy = jacobian[1][col];
              const len = Math.hypot(dx, dy);
              if (len < 1) return null;
              const labels = ['∂p/∂θ₁', '∂p/∂θ₂', '∂p/∂θ₃'];
              const colors = ['#60a5fa', '#34d399', '#c084fc'];
              return (
                <text
                  key={col}
                  x={j3x + dx + 6 * (dx > 0 ? 1 : -1)}
                  y={-(j3y + dy)}
                  fill={colors[col]}
                  fontSize="11"
                  fontWeight="700"
                >
                  {labels[col]}
                </text>
              );
            })}
          </g>
        )}
      </svg>

      {/* 右上角 HUD: 工作空间或参数 */}
      {page === 3 && (
        <div className="tw-absolute tw-top-3 tw-right-3 tw-bg-slate-800/95 tw-backdrop-blur-md tw-px-3 tw-py-2 tw-rounded-lg tw-shadow-lg tw-border tw-border-slate-700 tw-pointer-events-none">
          <div className="tw-text-[10px] tw-font-bold tw-text-slate-500 tw-mb-1">工作空间</div>
          <div className="tw-text-[11px] tw-font-mono tw-text-emerald-200">采样 {workspacePts.length}</div>
          <div className="tw-text-[11px] tw-font-mono tw-text-slate-300">外径 {L_SUM} px</div>
        </div>
      )}
    </div>
  );
}

export default App;
