import React, { useEffect, useRef, useState } from 'react';
import {
  Activity,
  AlertTriangle,
  BookOpen,
  Compass,
  Sliders,
  Calculator,
  Crosshair,
  ChevronLeft,
  ChevronRight,
  ChevronDown,
  Cog,
  Move,
  Pause,
  Play,
  RefreshCw,
  RotateCcw,
  Settings,
  Wind,
  Zap,
  ZapOff,
} from 'lucide-react';

const PAGE_KEYS = {
  ACTUATOR: 0,
  GEAR: 1,
  OPEN_LOOP: 2,
  CLOSED_LOOP: 3,
  BANG: 4,
  PID: 5,
  ARM: 6,
  WINDUP: 7,
};

const PAGES = [
  { title: '执行器', icon: BookOpen, color: 'tw-text-sky-400' },
  { title: '齿轮传动', icon: Cog, color: 'tw-text-amber-400' },
  { title: '开环控制', icon: Wind, color: 'tw-text-orange-400' },
  { title: '闭环控制', icon: Activity, color: 'tw-text-emerald-400' },
  { title: 'Bang-Bang 控制', icon: RotateCcw, color: 'tw-text-blue-400' },
  { title: 'PID 控制律', icon: Sliders, color: 'tw-text-sky-400' },
  { title: '机械臂关节控制', icon: Compass, color: 'tw-text-emerald-400' },
  { title: '积分饱和', icon: AlertTriangle, color: 'tw-text-rose-400' },
];

// ===== 齿轮渲染常量 & 辅助 =====
const GEAR_MODULE = 3;
const GEAR_VIEW_W = 700;
const GEAR_VIEW_H = 400;
const ACTUATOR_VIEW_W = 700;
const ACTUATOR_VIEW_H = 340;
const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const polarPoint = (cx, cy, radius, angleDeg) => {
  const rad = ((angleDeg - 90) * Math.PI) / 180;
  return {
    x: cx + Math.cos(rad) * radius,
    y: cy + Math.sin(rad) * radius,
  };
};
const jointPoint = (cx, cy, radius, angleDeg) => {
  const rad = (angleDeg * Math.PI) / 180;
  return {
    x: cx + Math.cos(rad) * radius,
    y: cy + Math.sin(rad) * radius,
  };
};
const verticalDialPoint = (cx, cy, radius, angleDeg) => {
  const rad = ((angleDeg - 90) * Math.PI) / 180;
  return {
    x: cx + Math.cos(rad) * radius,
    y: cy + Math.sin(rad) * radius,
  };
};
const ACTUATOR_TYPES = {
  servo: {
    label: '伺服电机',
    subtitle: '闭环位置控制',
    description: '编码器 + 控制器实时纠偏,适合精确到位和稳稳抱住目标姿态。',
    color: '#38bdf8',
    buttonClass: 'tw-border-sky-500 tw-bg-sky-950/70 tw-text-sky-100',
    chipClass: 'tw-border-sky-800 tw-bg-sky-950/60 tw-text-sky-200',
  },
  stepper: {
    label: '步进电机',
    subtitle: '离散步进',
    description: '每来一个脉冲转固定步距,结构简单,低速有力,但负载大时会失步。',
    color: '#f59e0b',
    buttonClass: 'tw-border-amber-500 tw-bg-amber-950/70 tw-text-amber-100',
    chipClass: 'tw-border-amber-800 tw-bg-amber-950/60 tw-text-amber-200',
  },
  bldc: {
    label: '直流无刷电机',
    subtitle: '高速高效',
    description: '功率密度高、转得快,通常配 ESC/FOC、编码器和减速器进入机器人关节。',
    color: '#34d399',
    buttonClass: 'tw-border-emerald-500 tw-bg-emerald-950/70 tw-text-emerald-100',
    chipClass: 'tw-border-emerald-800 tw-bg-emerald-950/60 tw-text-emerald-200',
  },
};

const renderGear = (cx, cy, teeth, angle, color) => {
  const innerR = teeth * GEAR_MODULE - 1.25 * GEAR_MODULE;
  const outerR = teeth * GEAR_MODULE + 1.2 * GEAR_MODULE;
  const holeR = teeth * GEAR_MODULE * 0.2;
  let d = '';
  for (let i = 0; i < teeth; i++) {
    const t1 = (i / teeth) * 2 * Math.PI;
    const t2 = ((i + 0.25) / teeth) * 2 * Math.PI;
    const t3 = ((i + 0.5) / teeth) * 2 * Math.PI;
    const t4 = ((i + 0.75) / teeth) * 2 * Math.PI;
    d += `${i === 0 ? 'M' : 'L'} ${(Math.cos(t1) * innerR).toFixed(2)} ${(Math.sin(t1) * innerR).toFixed(2)} `;
    d += `L ${(Math.cos(t2) * outerR).toFixed(2)} ${(Math.sin(t2) * outerR).toFixed(2)} `;
    d += `L ${(Math.cos(t3) * outerR).toFixed(2)} ${(Math.sin(t3) * outerR).toFixed(2)} `;
    d += `L ${(Math.cos(t4) * innerR).toFixed(2)} ${(Math.sin(t4) * innerR).toFixed(2)} `;
  }
  d += 'Z';
  const markLen = teeth * GEAR_MODULE * 0.7;
  const markH = teeth * GEAR_MODULE * 0.08;
  return (
    <g transform={`translate(${cx}, ${cy}) rotate(${(angle * 180 / Math.PI).toFixed(2)})`}>
      <path d={d} fill={color} stroke="#1e293b" strokeWidth="1.5" strokeLinejoin="round" />
      <circle cx="0" cy="0" r={holeR} fill="#0f172a" stroke="#1e293b" strokeWidth="1.5" />
      {/* 辐条标记,便于观察旋转方向 */}
      <rect x="0" y={-markH / 2} width={markLen} height={markH} fill="#0f172a" opacity="0.6" rx="1" />
    </g>
  );
};

const SIM_W = 600;
const SIM_H = 500;
const GRAPH_W = 600;
const GRAPH_H = 200;
const MAX_HISTORY = 300;

// ===== 开环 / 闭环风扇演示常量 =====
const FAN_SIM_W = 720;
const FAN_SIM_H = 420;
const FAN_CHART_W = 900;
const FAN_CHART_H = 160;
const FAN_HISTORY_MAX = 180;

// ===== Bang-Bang 控制演示常量 =====
const BANG_SIM_W = 560;
const BANG_SIM_H = 420;
const BANG_CHART_W = 900;
const BANG_CHART_H = 160;
const BANG_HISTORY_MAX = 180;
const BANG_DT_SCALE = 60;

// ===== 机械臂关节 PID 演示常量 =====
const ARM_SIM_W = 800;
const ARM_SIM_H = 720;
const ARM_CHART_W = 800;
const ARM_CHART_H = 128;
const ARM_LENGTH = 230;
const ARM_HISTORY_MAX = 220;
const ARM_DT = 0.016;
const ARM_GRAVITY = 9.81;
const ARM_FRICTION = 0.1;

// ===== Windup 演示常量 =====
const WINDUP_SIM_W = 700;
const WINDUP_SIM_H = 200;
const WINDUP_BLOCK = 30;
const WINDUP_TARGET_X = 620;
const WINDUP_START_X = 50;
const WINDUP_WALL_X = 340;
const WINDUP_WALL_W = 16;
const WINDUP_INTEGRAL_CAP = 10000;

const PRESETS = {
  stable: { name: '✅ 完美平稳', kp: 2.0, ki: 0.5, kd: 5.0 },
  oscillation: { name: '〰️ 剧烈震荡', kp: 8.0, ki: 0.0, kd: 0.5 },
  droop: { name: '📉 飞不到位', kp: 2.0, ki: 0.0, kd: 3.0 },
  slow: { name: '🐢 反应迟钝', kp: 0.5, ki: 0.2, kd: 1.0 },
};

const makeBangHistory = (target, hysteresis) => (
  Array.from({ length: BANG_HISTORY_MAX }, (_, idx) => ({
    time: idx,
    angle: 0,
    target,
    upperBound: target + hysteresis,
    lowerBound: target - hysteresis,
  }))
);

const makeFanHistory = (actual = 0, reference = 0, voltage = 0) => (
  Array.from({ length: FAN_HISTORY_MAX }, () => ({
    actual,
    reference,
    voltage,
  }))
);

const ARM_PRESETS = {
  industrial: { name: '工业级稳健', kp: 25, ki: 1.5, kd: 12 },
  loose: { name: '松散高弹', kp: 8, ki: 0.2, kd: 2 },
  noI: { name: '无 I 静差', kp: 15, ki: 0, kd: 8 },
  unstable: { name: '欠阻尼震荡', kp: 40, ki: 0.1, kd: 1 },
};

const App = () => {
  // ===== PID 参数 =====
  const [kp, setKp] = useState(2.0);
  const [ki, setKi] = useState(0.5);
  const [kd, setKd] = useState(3.0);

  // ===== 目标高度(显示用) =====
  const [targetY, setTargetY] = useState(150);

  // ===== 齿轮传动 =====
  const [tA, setTA] = useState(15);
  const [tB, setTB] = useState(45);
  const [rpm, setRpm] = useState(60);
  const tARef = useRef(15);
  const tBRef = useRef(45);
  const rpmRef = useRef(60);
  const gearAngleRef = useRef(0);
  const lastGearTimeRef = useRef(performance.now());
  useEffect(() => { tARef.current = tA; }, [tA]);
  useEffect(() => { tBRef.current = tB; }, [tB]);
  useEffect(() => { rpmRef.current = rpm; }, [rpm]);

  // ===== 电动执行器演示 =====
  const [actuatorType, setActuatorType] = useState('servo');
  const [actuatorCommand, setActuatorCommand] = useState(55);
  const [actuatorLoad, setActuatorLoad] = useState(25);
  const actuatorTypeRef = useRef('servo');
  const actuatorCommandRef = useRef(55);
  const actuatorLoadRef = useRef(25);
  const actuatorDemoRef = useRef({
    angle: 0,
    rpm: 0,
    targetAngle: 0,
    torque: 18,
    heat: 28,
    phase: 0,
    lostSteps: 0,
    stepTimer: 0,
  });
  useEffect(() => { actuatorTypeRef.current = actuatorType; }, [actuatorType]);
  useEffect(() => { actuatorCommandRef.current = actuatorCommand; }, [actuatorCommand]);
  useEffect(() => { actuatorLoadRef.current = actuatorLoad; }, [actuatorLoad]);
  useEffect(() => {
    actuatorDemoRef.current = {
      angle: 0,
      rpm: 0,
      targetAngle: 0,
      torque: 18,
      heat: 28,
      phase: 0,
      lostSteps: 0,
      stepTimer: 0,
    };
  }, [actuatorType]);

  // ===== 开环控制演示 =====
  const [openVoltage, setOpenVoltage] = useState(50);
  const [openResistance, setOpenResistance] = useState(0);
  const [openRunning, setOpenRunning] = useState(false);
  const openVoltageRef = useRef(50);
  const openResistanceRef = useRef(0);
  const openRunningRef = useRef(false);
  const openLoopPhysicsRef = useRef({
    speed: 0,
    bladeAngle: 0,
  });
  const openLoopHistoryRef = useRef(makeFanHistory(0, 100, 50));
  useEffect(() => { openVoltageRef.current = openVoltage; }, [openVoltage]);
  useEffect(() => { openResistanceRef.current = openResistance; }, [openResistance]);
  useEffect(() => { openRunningRef.current = openRunning; }, [openRunning]);

  // ===== 闭环控制演示 =====
  const [closedTargetSpeed, setClosedTargetSpeed] = useState(100);
  const [closedResistance, setClosedResistance] = useState(0);
  const [closedRunning, setClosedRunning] = useState(false);
  const closedTargetSpeedRef = useRef(100);
  const closedResistanceRef = useRef(0);
  const closedRunningRef = useRef(false);
  const closedLoopPhysicsRef = useRef({
    speed: 0,
    voltage: 0,
    bladeAngle: 0,
  });
  const closedLoopHistoryRef = useRef(makeFanHistory(0, 100, 0));
  useEffect(() => { closedTargetSpeedRef.current = closedTargetSpeed; }, [closedTargetSpeed]);
  useEffect(() => { closedResistanceRef.current = closedResistance; }, [closedResistance]);
  useEffect(() => { closedRunningRef.current = closedRunning; }, [closedRunning]);

  // ===== Bang-Bang 控制演示 =====
  const [bangTargetAngle, setBangTargetAngle] = useState(90);
  const [bangHysteresis, setBangHysteresis] = useState(2.0);
  const [bangTorque, setBangTorque] = useState(0.8);
  const [bangFriction, setBangFriction] = useState(0.05);
  const [bangRunning, setBangRunning] = useState(true);
  const bangTargetAngleRef = useRef(90);
  const bangHysteresisRef = useRef(2.0);
  const bangTorqueRef = useRef(0.8);
  const bangFrictionRef = useRef(0.05);
  const bangRunningRef = useRef(true);
  const bangHistoryRef = useRef(makeBangHistory(90, 2.0));
  const bangPhysicsRef = useRef({
    angle: 0,
    velocity: 0,
    motorState: 0,
    frame: BANG_HISTORY_MAX,
  });
  useEffect(() => { bangTargetAngleRef.current = bangTargetAngle; }, [bangTargetAngle]);
  useEffect(() => { bangHysteresisRef.current = bangHysteresis; }, [bangHysteresis]);
  useEffect(() => { bangTorqueRef.current = bangTorque; }, [bangTorque]);
  useEffect(() => { bangFrictionRef.current = bangFriction; }, [bangFriction]);
  useEffect(() => { bangRunningRef.current = bangRunning; }, [bangRunning]);

  // ===== 机械臂关节 PID 演示 =====
  const [armKp, setArmKp] = useState(15.0);
  const [armKi, setArmKi] = useState(0.5);
  const [armKd, setArmKd] = useState(8.0);
  const [armMass, setArmMass] = useState(1.0);
  const [armUseGravity, setArmUseGravity] = useState(true);
  const [armTargetAngle, setArmTargetAngle] = useState(0);
  const armKpRef = useRef(15.0);
  const armKiRef = useRef(0.5);
  const armKdRef = useRef(8.0);
  const armMassRef = useRef(1.0);
  const armUseGravityRef = useRef(true);
  const armTargetAngleRef = useRef(0);
  const armSvgRef = useRef(null);
  const armHistoryRef = useRef([]);
  const armPhysicsRef = useRef({
    angle: -90,
    velocity: 0,
    integral: 0,
    lastError: 0,
    pTerm: 0,
    iTerm: 0,
    dTerm: 0,
    pidTorque: 0,
    gravityTorque: 0,
    frictionTorque: 0,
    totalTorque: 0,
  });
  useEffect(() => { armKpRef.current = armKp; }, [armKp]);
  useEffect(() => {
    armKiRef.current = armKi;
    if (armKi === 0) armPhysicsRef.current.integral = 0;
  }, [armKi]);
  useEffect(() => { armKdRef.current = armKd; }, [armKd]);
  useEffect(() => { armMassRef.current = armMass; }, [armMass]);
  useEffect(() => { armUseGravityRef.current = armUseGravity; }, [armUseGravity]);
  useEffect(() => { armTargetAngleRef.current = armTargetAngle; }, [armTargetAngle]);

  // ===== 积分饱和 (Windup) 演示 =====
  const [windupKp, setWindupKp] = useState(2.0);
  const [windupKi, setWindupKi] = useState(0.5);
  const [windupKd, setWindupKd] = useState(10.0);
  const [windupMaxF, setWindupMaxF] = useState(50);
  const [wallActive, setWallActive] = useState(true);
  const windupKpRef = useRef(2.0);
  const windupKiRef = useRef(0.5);
  const windupKdRef = useRef(10.0);
  const windupMaxFRef = useRef(50);
  const wallActiveRef = useRef(true);
  const windupPhysicsRef = useRef({
    blockX: WINDUP_START_X,
    velocity: 0,
    integralSum: 0,
    lastError: WINDUP_TARGET_X - WINDUP_START_X,
    calcForce: 0,
    actualForce: 0,
  });
  useEffect(() => { windupKpRef.current = windupKp; }, [windupKp]);
  useEffect(() => {
    windupKiRef.current = windupKi;
    if (windupKi === 0) windupPhysicsRef.current.integralSum = 0;
  }, [windupKi]);
  useEffect(() => { windupKdRef.current = windupKd; }, [windupKd]);
  useEffect(() => { windupMaxFRef.current = windupMaxF; }, [windupMaxF]);
  useEffect(() => { wallActiveRef.current = wallActive; }, [wallActive]);

  const resetWindup = () => {
    windupPhysicsRef.current = {
      blockX: WINDUP_START_X,
      velocity: 0,
      integralSum: 0,
      lastError: WINDUP_TARGET_X - WINDUP_START_X,
      calcForce: 0,
      actualForce: 0,
    };
    setWallActive(true);
  };

  // ===== 分页 =====
  const [currentPage, setCurrentPage] = useState(0);
  const totalPages = PAGES.length;
  const currentPageRef = useRef(0);
  useEffect(() => { currentPageRef.current = currentPage; }, [currentPage]);

  // 进入 Windup 页时自动重置演示
  useEffect(() => {
    if (currentPage === PAGE_KEYS.WINDUP) {
      windupPhysicsRef.current = {
        blockX: WINDUP_START_X,
        velocity: 0,
        integralSum: 0,
        lastError: WINDUP_TARGET_X - WINDUP_START_X,
        calcForce: 0,
        actualForce: 0,
      };
    }
  }, [currentPage]);

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

  // ===== 物理状态(refs,避免每帧触发 re-render) =====
  const physicsRef = useRef({ currentY: 250, velocity: 0, integral: 0, lastError: 0 });
  const targetRef = useRef(150);
  const historyRef = useRef([]);
  const kpRef = useRef(kp);
  const kiRef = useRef(ki);
  const kdRef = useRef(kd);
  const svgRef = useRef(null);

  // 把 state 同步到 ref,rAF 循环里读 ref 拿到最新值
  useEffect(() => { kpRef.current = kp; }, [kp]);
  useEffect(() => { kiRef.current = ki; }, [ki]);
  useEffect(() => { kdRef.current = kd; }, [kd]);
  useEffect(() => { targetRef.current = targetY; }, [targetY]);

  // 用一个 tick 驱动重绘
  const [, setTick] = useState(0);

  // ===== 物理循环 =====
  useEffect(() => {
    let raf;
    const dt = 0.1;
    const gravity = 0.5;
    const mass = 1.0;

    const step = () => {
      const s = physicsRef.current;
      const target = targetRef.current;
      const error = s.currentY - target; // > 0 需要向上飞

      // PID 三项
      const pOut = kpRef.current * error;
      s.integral += error * dt;
      // 积分限幅(anti-windup)
      if (Math.abs(s.integral) > 1000) s.integral = 1000 * Math.sign(s.integral);
      const iOut = kiRef.current * s.integral;
      const derivative = (error - s.lastError) / dt;
      const dOut = kdRef.current * derivative;
      s.lastError = error;

      const outputForce = pOut + iOut + dOut;
      // Canvas y 向下为正,向上推力取负;重力系数放大 50 让 I 项效果明显
      const netForce = -outputForce + gravity * 50;
      const acceleration = netForce / mass;
      s.velocity += acceleration * dt;
      s.currentY += s.velocity * dt;

      // 边界碰撞
      if (s.currentY > SIM_H - 20) {
        s.currentY = SIM_H - 20;
        s.velocity *= -0.5;
      }
      if (s.currentY < 20) {
        s.currentY = 20;
        s.velocity = 0;
      }

      historyRef.current.push(s.currentY);
      if (historyRef.current.length > MAX_HISTORY) historyRef.current.shift();

      // 齿轮角度 —— 用实时 dt(PID 物理用的是固定 dt=0.1,两者独立)
      const nowT = performance.now();
      const gDt = Math.min((nowT - lastGearTimeRef.current) / 1000, 0.1);
      lastGearTimeRef.current = nowT;
      gearAngleRef.current += (rpmRef.current / 60) * 2 * Math.PI * gDt;

      // 电动执行器动画
      const actuator = actuatorDemoRef.current;
      const cmd = actuatorCommandRef.current;
      const load = actuatorLoadRef.current;
      const loadRatio = load / 100;
      actuator.phase += gDt;

      if (actuatorTypeRef.current === 'servo') {
        const targetAngle = (cmd - 50) * 2.4;
        const disturbance = Math.sin(actuator.phase * 2.1) * loadRatio * 18;
        const angleError = targetAngle - actuator.angle;
        const control = angleError * 5.8 - actuator.rpm * 0.16;
        actuator.rpm += (control - disturbance) * gDt * 10;
        actuator.rpm *= 0.92;
        actuator.angle = clamp(actuator.angle + actuator.rpm * gDt, -130, 130);
        actuator.targetAngle = targetAngle;
        actuator.torque = clamp(Math.abs(angleError) * 0.55 + load * 0.48, 8, 100);
        actuator.heat += (30 + actuator.torque * 0.18 - actuator.heat) * 0.08;
        actuator.lostSteps *= 0.9;
      } else if (actuatorTypeRef.current === 'stepper') {
        const stepAngle = 15;
        const targetStep = Math.round((cmd / 100) * 16) - 8;
        const targetAngle = targetStep * stepAngle;
        const effectiveStep = stepAngle * (load > 72 ? clamp(1 - (load - 72) / 45, 0.35, 1) : 1);
        actuator.targetAngle = targetAngle;
        actuator.stepTimer += gDt * (2 + cmd / 11);

        while (actuator.stepTimer >= 1 && Math.abs(targetAngle - actuator.angle) > 0.1) {
          actuator.stepTimer -= 1;
          const diffAngle = targetAngle - actuator.angle;
          const stepDir = Math.sign(diffAngle);
          const stepSize = Math.abs(diffAngle) < stepAngle ? Math.abs(diffAngle) : effectiveStep;
          actuator.angle = clamp(actuator.angle + stepDir * stepSize, -130, 130);
          if (effectiveStep < stepAngle) {
            actuator.lostSteps = clamp(
              actuator.lostSteps + ((stepAngle - effectiveStep) / stepAngle) * 0.45,
              0,
              6,
            );
          }
        }

        actuator.rpm += (
          ((2 + cmd / 11) * 14 * Math.sign(targetAngle - actuator.angle) * Math.max(0.35, 1 - loadRatio * 0.7))
          - actuator.rpm
        ) * 0.15;
        actuator.torque = clamp(24 + load * 0.55 + actuator.lostSteps * 12, 12, 100);
        actuator.heat += (32 + actuator.torque * 0.16 - actuator.heat) * 0.08;
        actuator.lostSteps *= load > 70 ? 0.995 : 0.94;
      } else {
        const targetRpm = 400 + cmd * 48;
        actuator.rpm += (targetRpm - actuator.rpm) * clamp(gDt * (3.5 - loadRatio * 1.3), 0, 1);
        actuator.rpm *= 1 - loadRatio * 0.015;
        actuator.angle = (actuator.angle + actuator.rpm * 6 * gDt) % 360;
        actuator.targetAngle = 0;
        actuator.torque = clamp(18 + load * 0.62 + Math.abs(targetRpm - actuator.rpm) * 0.02, 10, 100);
        actuator.heat += (26 + load * 0.12 + actuator.rpm / 220 - actuator.heat) * 0.06;
        actuator.lostSteps = 0;
        actuator.stepTimer = 0;
      }

      // ===== 开环风扇控制物理 =====
      if (currentPageRef.current === PAGE_KEYS.OPEN_LOOP) {
        const fan = openLoopPhysicsRef.current;
        const voltage = openVoltageRef.current;
        const resistance = openResistanceRef.current;
        const running = openRunningRef.current;
        const noLoadSpeed = voltage * 2;
        const loadedSpeed = Math.max(0, noLoadSpeed - resistance * 1.5);
        const targetSpeed = running ? loadedSpeed : 0;

        fan.speed += (targetSpeed - fan.speed) * clamp(gDt * (running ? 3.2 : 5.5), 0, 1);
        fan.bladeAngle = (fan.bladeAngle + fan.speed * 5.8 * gDt) % 360;

        openLoopHistoryRef.current.push({
          actual: fan.speed,
          reference: running ? noLoadSpeed : 0,
          voltage,
        });
        if (openLoopHistoryRef.current.length > FAN_HISTORY_MAX) openLoopHistoryRef.current.shift();
      }

      // ===== 闭环风扇控制物理 =====
      if (currentPageRef.current === PAGE_KEYS.CLOSED_LOOP) {
        const fan = closedLoopPhysicsRef.current;
        const targetSpeed = closedTargetSpeedRef.current;
        const resistance = closedResistanceRef.current;
        const running = closedRunningRef.current;

        if (running) {
          const speedError = targetSpeed - fan.speed;
          fan.voltage = clamp(fan.voltage + speedError * 0.45 * gDt * 2.2, 0, 120);
          const physicalLimit = Math.max(0, fan.voltage * 2 - resistance * 1.5);
          fan.speed += (physicalLimit - fan.speed) * clamp(gDt * 2.5, 0, 1);
        } else {
          fan.voltage += (0 - fan.voltage) * clamp(gDt * 5.5, 0, 1);
          fan.speed += (0 - fan.speed) * clamp(gDt * 5.5, 0, 1);
        }

        fan.bladeAngle = (fan.bladeAngle + fan.speed * 5.8 * gDt) % 360;
        closedLoopHistoryRef.current.push({
          actual: fan.speed,
          reference: running ? targetSpeed : 0,
          voltage: fan.voltage,
        });
        if (closedLoopHistoryRef.current.length > FAN_HISTORY_MAX) closedLoopHistoryRef.current.shift();
      }

      // ===== Bang-Bang 位置控制物理 =====
      if (currentPageRef.current === PAGE_KEYS.BANG && bangRunningRef.current) {
        const bang = bangPhysicsRef.current;
        const targetAngle = bangTargetAngleRef.current;
        const hysteresis = bangHysteresisRef.current;
        let nextMotorState = bang.motorState;

        if (bang.angle < targetAngle - hysteresis) {
          nextMotorState = 1;
        } else if (bang.angle > targetAngle + hysteresis) {
          nextMotorState = -1;
        }

        const scaledDt = gDt * BANG_DT_SCALE;
        const acceleration = (nextMotorState * bangTorqueRef.current) - (bang.velocity * bangFrictionRef.current);
        bang.velocity += acceleration * scaledDt;
        bang.angle += bang.velocity * scaledDt;
        bang.motorState = nextMotorState;
        bang.frame += 1;

        bangHistoryRef.current.push({
          time: bang.frame,
          angle: bang.angle,
          target: targetAngle,
          upperBound: targetAngle + hysteresis,
          lowerBound: targetAngle - hysteresis,
        });
        if (bangHistoryRef.current.length > BANG_HISTORY_MAX) bangHistoryRef.current.shift();
      }

      // ===== 机械臂单关节 PID 物理 =====
      if (currentPageRef.current === PAGE_KEYS.ARM) {
        const arm = armPhysicsRef.current;
        const targetAngle = armTargetAngleRef.current;
        const massValue = armMassRef.current;
        const errorAngle = targetAngle - arm.angle;

        const pTermArm = armKpRef.current * errorAngle;
        arm.integral += errorAngle * ARM_DT;
        arm.integral = clamp(arm.integral, -100, 100);
        const iTermArm = armKiRef.current * arm.integral;
        const derivativeArm = (errorAngle - arm.lastError) / ARM_DT;
        const dTermArm = armKdRef.current * derivativeArm;
        const pidTorque = pTermArm + iTermArm + dTermArm;

        const thetaRad = (arm.angle * Math.PI) / 180;
        const gravityTorque = armUseGravityRef.current
          ? massValue * ARM_GRAVITY * 20 * Math.cos(thetaRad)
          : 0;
        const frictionTorque = arm.velocity * ARM_FRICTION * 50;
        const totalTorque = pidTorque - gravityTorque - frictionTorque;
        const inertia = massValue * 10;
        const acceleration = totalTorque / inertia;

        arm.velocity += acceleration * ARM_DT;
        arm.angle += arm.velocity * ARM_DT;
        arm.lastError = errorAngle;
        arm.pTerm = pTermArm;
        arm.iTerm = iTermArm;
        arm.dTerm = dTermArm;
        arm.pidTorque = pidTorque;
        arm.gravityTorque = gravityTorque;
        arm.frictionTorque = frictionTorque;
        arm.totalTorque = totalTorque;

        armHistoryRef.current.push({ target: targetAngle, current: arm.angle });
        if (armHistoryRef.current.length > ARM_HISTORY_MAX) armHistoryRef.current.shift();
      }

      // ===== 积分饱和 (Windup) 物理 =====
      if (currentPageRef.current === PAGE_KEYS.WINDUP) {
        const wPhys = windupPhysicsRef.current;
        const wKp = windupKpRef.current;
        const wKi = windupKiRef.current;
        const wKd = windupKdRef.current;
        const wMaxF = windupMaxFRef.current;
        const wallOn = wallActiveRef.current;
        const wdt = 0.05;

        const error = WINDUP_TARGET_X - wPhys.blockX;
        const pTerm = wKp * error;

        if (wKi > 0) {
          wPhys.integralSum += error * wdt;
          // 防止积分数值溢出到 NaN,但留足够上限让 windup 看得见
          wPhys.integralSum = clamp(wPhys.integralSum, -WINDUP_INTEGRAL_CAP, WINDUP_INTEGRAL_CAP);
        } else {
          wPhys.integralSum = 0;
        }
        const iTerm = wKi * wPhys.integralSum;
        const errorRate = (error - wPhys.lastError) / wdt;
        const dTermW = wKd * errorRate;
        wPhys.lastError = error;

        const calcForce = pTerm + iTerm + dTermW;
        const actualForce = clamp(calcForce, -wMaxF, wMaxF);

        wPhys.velocity += (actualForce / 1) * wdt;
        wPhys.velocity *= 0.98;
        wPhys.blockX += wPhys.velocity * wdt;

        // 刚性墙碰撞:卡死,但误差继续存在,积分还在累!
        if (wallOn && wPhys.blockX + WINDUP_BLOCK / 2 > WINDUP_WALL_X && error > 0) {
          wPhys.blockX = WINDUP_WALL_X - WINDUP_BLOCK / 2;
          wPhys.velocity = 0;
        }
        // 画布边界
        wPhys.blockX = clamp(wPhys.blockX, 20, WINDUP_SIM_W - 20);

        wPhys.calcForce = calcForce;
        wPhys.actualForce = actualForce;
      }

      setTick((t) => (t + 1) % 1_000_000);
      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, []);

  // ===== 点击画布改目标高度 =====
  const handleSimPointerDown = (e) => {
    const svg = svgRef.current;
    if (!svg) return;
    const pt = svg.createSVGPoint();
    pt.x = e.clientX;
    pt.y = e.clientY;
    const ctm = svg.getScreenCTM();
    if (!ctm) return;
    const p = pt.matrixTransform(ctm.inverse());
    const y = Math.max(20, Math.min(SIM_H - 20, p.y));
    setTargetY(y);
    physicsRef.current.integral = 0;
  };

  const resetOpenLoopFan = () => {
    openLoopPhysicsRef.current = {
      speed: 0,
      bladeAngle: 0,
    };
    openLoopHistoryRef.current = makeFanHistory(0, openVoltageRef.current * 2, openVoltageRef.current);
    setOpenRunning(false);
    setOpenResistance(0);
  };

  const resetClosedLoopFan = () => {
    closedLoopPhysicsRef.current = {
      speed: 0,
      voltage: 0,
      bladeAngle: 0,
    };
    closedLoopHistoryRef.current = makeFanHistory(0, closedTargetSpeedRef.current, 0);
    setClosedRunning(false);
    setClosedResistance(0);
  };

  const resetBangBang = () => {
    bangPhysicsRef.current = {
      angle: 0,
      velocity: 0,
      motorState: 0,
      frame: BANG_HISTORY_MAX,
    };
    bangHistoryRef.current = makeBangHistory(bangTargetAngleRef.current, bangHysteresisRef.current);
  };

  // ===== 机械臂画布交互 =====
  const handleArmPointer = (e) => {
    const svg = armSvgRef.current;
    if (!svg) return;
    const pt = svg.createSVGPoint();
    pt.x = e.clientX;
    pt.y = e.clientY;
    const ctm = svg.getScreenCTM();
    if (!ctm) return;
    const p = pt.matrixTransform(ctm.inverse());
    const angle = (Math.atan2(p.y - ARM_SIM_H / 2, p.x - ARM_SIM_W / 2) * 180) / Math.PI;
    setArmTargetAngle(angle);
    armTargetAngleRef.current = angle;
    armPhysicsRef.current.integral = 0;
    armPhysicsRef.current.lastError = angle - armPhysicsRef.current.angle;
  };

  const resetArmJoint = () => {
    armPhysicsRef.current = {
      angle: -90,
      velocity: 0,
      integral: 0,
      lastError: armTargetAngleRef.current + 90,
      pTerm: 0,
      iTerm: 0,
      dTerm: 0,
      pidTorque: 0,
      gravityTorque: 0,
      frictionTorque: 0,
      totalTorque: 0,
    };
    armHistoryRef.current = [];
  };

  const applyArmPreset = (key) => {
    const p = ARM_PRESETS[key];
    if (!p) return;
    setArmKp(p.kp);
    setArmKi(p.ki);
    setArmKd(p.kd);
    armPhysicsRef.current.integral = 0;
    armPhysicsRef.current.lastError = armTargetAngleRef.current - armPhysicsRef.current.angle;
  };

  // ===== 预设 =====
  const applyPreset = (key) => {
    const p = PRESETS[key];
    if (!p) return;
    setKp(p.kp);
    setKi(p.ki);
    setKd(p.kd);
    physicsRef.current.integral = 0;
    physicsRef.current.currentY = targetRef.current + 100;
    physicsRef.current.velocity = 0;
    physicsRef.current.lastError = 0;
  };

  // ===== 分页导航 =====
  const nextPage = () => setCurrentPage((p) => Math.min(totalPages - 1, p + 1));
  const prevPage = () => setCurrentPage((p) => Math.max(0, p - 1));

  // ===== 当前物理读数 =====
  const phys = physicsRef.current;
  const currentHeight = Math.round(SIM_H - phys.currentY);
  const targetHeight = Math.round(SIM_H - targetY);
  const error = phys.currentY - targetY;
  const pTerm = kp * error;
  const iTerm = ki * phys.integral;
  const dTerm = kd * ((error - phys.lastError) / 0.1);
  const output = pTerm + iTerm + dTerm;

  // ===== 螺旋桨动画相位 =====
  const propPhase = (Date.now() / 50) % 10;

  // ===== 历史折线点串 =====
  const historyPoints = historyRef.current
    .map((y, i) => `${(i / MAX_HISTORY) * GRAPH_W},${(y / SIM_H) * GRAPH_H}`)
    .join(' ');

  // ===== 推力火焰可视化 =====
  const thrust = Math.max(0, phys.currentY - targetY + phys.integral * 0.1);

  // ===== Bang-Bang 读数与曲线 =====
  const bangPhys = bangPhysicsRef.current;
  const bangCenterX = BANG_SIM_W / 2;
  const bangCenterY = BANG_SIM_H / 2;
  const bangDialRadius = 148;
  const bangArmLength = 132;
  const bangTargetPoint = verticalDialPoint(bangCenterX, bangCenterY, bangDialRadius, bangTargetAngle);
  const bangError = bangTargetAngle - bangPhys.angle;
  const bangUpperBound = bangTargetAngle + bangHysteresis;
  const bangLowerBound = bangTargetAngle - bangHysteresis;
  const bangMotorMeta = bangPhys.motorState > 0
    ? { label: '正转', color: '#22c55e', icon: Zap }
    : bangPhys.motorState < 0
      ? { label: '反转', color: '#ef4444', icon: Zap }
      : { label: '停止', color: '#64748b', icon: ZapOff };
  const BangMotorIcon = bangMotorMeta.icon;
  const bangChartY = (angleValue) => (
    clamp(BANG_CHART_H - ((angleValue + 30) / 240) * BANG_CHART_H, 8, BANG_CHART_H - 8)
  );
  const bangHistory = bangHistoryRef.current;
  const bangAnglePoints = bangHistory
    .map((h, i) => `${(i / Math.max(1, BANG_HISTORY_MAX - 1)) * BANG_CHART_W},${bangChartY(h.angle)}`)
    .join(' ');
  const bangTargetPoints = bangHistory
    .map((h, i) => `${(i / Math.max(1, BANG_HISTORY_MAX - 1)) * BANG_CHART_W},${bangChartY(h.target)}`)
    .join(' ');
  const bangUpperPoints = bangHistory
    .map((h, i) => `${(i / Math.max(1, BANG_HISTORY_MAX - 1)) * BANG_CHART_W},${bangChartY(h.upperBound)}`)
    .join(' ');
  const bangLowerPoints = bangHistory
    .map((h, i) => `${(i / Math.max(1, BANG_HISTORY_MAX - 1)) * BANG_CHART_W},${bangChartY(h.lowerBound)}`)
    .join(' ');

  // ===== Windup 读数 =====
  const wPhys = windupPhysicsRef.current;
  const windupError = WINDUP_TARGET_X - wPhys.blockX;
  const windupITerm = windupKi * wPhys.integralSum;
  const windupGlowSize = Math.min(110, Math.abs(wPhys.integralSum) / 50);
  const showWindupGlow = windupKi > 0 && Math.abs(wPhys.integralSum) > 400;

  // ===== 开环 / 闭环风扇读数 =====
  const fanChartY = (speedValue) => (
    clamp(FAN_CHART_H - (speedValue / 220) * FAN_CHART_H, 8, FAN_CHART_H - 8)
  );
  const fanChartX = (idx) => (idx / Math.max(1, FAN_HISTORY_MAX - 1)) * FAN_CHART_W;
  const openFan = openLoopPhysicsRef.current;
  const openNoLoadSpeed = openVoltage * 2;
  const openLoadedSpeed = Math.max(0, openNoLoadSpeed - openResistance * 1.5);
  const openSpeedDrop = Math.max(0, openNoLoadSpeed - openFan.speed);
  const openHistory = openLoopHistoryRef.current;
  const openActualPoints = openHistory
    .map((h, i) => `${fanChartX(i)},${fanChartY(h.actual)}`)
    .join(' ');
  const openReferencePoints = openHistory
    .map((h, i) => `${fanChartX(i)},${fanChartY(h.reference)}`)
    .join(' ');
  const closedFan = closedLoopPhysicsRef.current;
  const closedError = closedTargetSpeed - closedFan.speed;
  const closedWithinBand = Math.abs(closedError) < 4 && closedRunning;
  const closedHistory = closedLoopHistoryRef.current;
  const closedActualPoints = closedHistory
    .map((h, i) => `${fanChartX(i)},${fanChartY(h.actual)}`)
    .join(' ');
  const closedTargetPoints = closedHistory
    .map((h, i) => `${fanChartX(i)},${fanChartY(h.reference)}`)
    .join(' ');
  const closedVoltagePoints = closedHistory
    .map((h, i) => `${fanChartX(i)},${FAN_CHART_H - (h.voltage / 120) * (FAN_CHART_H - 16) - 8}`)
    .join(' ');

  // ===== 齿轮传动读数 =====
  const gearRatio = tB / tA;
  const rpmOut = rpm / gearRatio;
  const gearAngleA = gearAngleRef.current;
  // 加小偏移 Math.PI / tB 让两齿轮视觉上刚好错开啮合
  const gearAngleB = -gearAngleA * (tA / tB) + Math.PI / tB;
  const actuatorMeta = ACTUATOR_TYPES[actuatorType];
  const actuatorDemo = actuatorDemoRef.current;
  const actuatorCenterX = 240;
  const actuatorCenterY = 170;
  const actuatorRadius = 84;
  const actuatorActualPoint = polarPoint(actuatorCenterX, actuatorCenterY, actuatorRadius, actuatorDemo.angle);
  const actuatorTargetPoint = polarPoint(
    actuatorCenterX,
    actuatorCenterY,
    actuatorRadius + 14,
    actuatorDemo.targetAngle,
  );
  const actuatorLoadTrackY = 94;
  const actuatorLoadTrackHeight = 128;
  const actuatorLoadHeight = actuatorLoadTrackHeight * (actuatorLoad / 100);
  const actuatorEfficiency = clamp(
    actuatorType === 'bldc' ? 94 - actuatorLoad * 0.18 : 91 - actuatorLoad * 0.28,
    55,
    96,
  );
  const actuatorTorquePct = Math.round(actuatorDemo.torque);
  const actuatorTemp = actuatorDemo.heat.toFixed(0);
  const actuatorStepAngles = Array.from({ length: 17 }, (_, idx) => (idx - 8) * 15);
  let actuatorCommandLabel = '目标角度';
  let actuatorCommandValue = `${Math.round((actuatorCommand - 50) * 2.4)}°`;
  let actuatorActualLabel = '输出角度';
  let actuatorActualValue = `${Math.round(actuatorDemo.angle)}°`;
  let actuatorSecondaryLabel = '闭环误差';
  let actuatorSecondaryValue = `${Math.abs(actuatorDemo.targetAngle - actuatorDemo.angle).toFixed(1)}°`;
  let actuatorHint = '闭环反馈会主动对抗外界负载,更适合稳定保持关节姿态。';

  if (actuatorType === 'stepper') {
    actuatorCommandLabel = '目标步位';
    actuatorCommandValue = `${Math.round(actuatorDemo.targetAngle / 15)} steps`;
    actuatorActualLabel = '当前角度';
    actuatorActualValue = `${Math.round(actuatorDemo.angle)}°`;
    actuatorSecondaryLabel = '丢步指数';
    actuatorSecondaryValue = actuatorDemo.lostSteps.toFixed(1);
    actuatorHint = '步进电机靠脉冲逐格推进,负载大或节拍太快时会逐步偏离目标。';
  } else if (actuatorType === 'bldc') {
    actuatorCommandLabel = '目标转速';
    actuatorCommandValue = `${Math.round(400 + actuatorCommand * 48)} RPM`;
    actuatorActualLabel = '当前转速';
    actuatorActualValue = `${Math.round(actuatorDemo.rpm)} RPM`;
    actuatorSecondaryLabel = '效率感知';
    actuatorSecondaryValue = `${Math.round(actuatorEfficiency)}%`;
    actuatorHint = 'BLDC 天生擅长高速旋转,想做高精度关节控制通常还要再接编码器与减速器。';
  }

  // ===== 机械臂关节读数 =====
  const armPhys = armPhysicsRef.current;
  const armCenterX = ARM_SIM_W / 2;
  const armCenterY = ARM_SIM_H / 2;
  const armEnd = jointPoint(armCenterX, armCenterY, ARM_LENGTH, armPhys.angle);
  const armTargetEnd = jointPoint(armCenterX, armCenterY, ARM_LENGTH, armTargetAngle);
  const armError = armTargetAngle - armPhys.angle;
  const armStable = Math.abs(armError) < 1 && Math.abs(armPhys.velocity) < 1.2;
  const armMassRadius = 8 + armMass * 4;
  const armCurrentAngleLabel = `${Math.round(armPhys.angle)}°`;
  const armTargetAngleLabel = `${Math.round(armTargetAngle)}°`;
  const armHistoryTargetPoints = armHistoryRef.current
    .map((h, i) => {
      const x = (i / Math.max(1, ARM_HISTORY_MAX - 1)) * ARM_CHART_W;
      const y = clamp(ARM_CHART_H / 2 - h.target * 0.35, 6, ARM_CHART_H - 6);
      return `${x},${y}`;
    })
    .join(' ');
  const armHistoryCurrentPoints = armHistoryRef.current
    .map((h, i) => {
      const x = (i / Math.max(1, ARM_HISTORY_MAX - 1)) * ARM_CHART_W;
      const y = clamp(ARM_CHART_H / 2 - h.current * 0.35, 6, ARM_CHART_H - 6);
      return `${x},${y}`;
    })
    .join(' ');

  return (
    <div
      style={{ minHeight: 'calc(100vh - var(--ifm-navbar-height, 60px) - 48px)' }}
      className="tw-relative tw-flex tw-flex-col md:tw-flex-row md:tw-h-[calc(100vh-var(--ifm-navbar-height,60px)-48px)] md:tw-overflow-hidden tw-font-sans tw-text-slate-100"
    >
      {/* 左侧:课程内容 */}
      <div className="tw-w-full md:tw-w-1/2 lg:tw-w-5/12 tw-bg-slate-800/75 tw-backdrop-blur-sm tw-border-r tw-border-slate-700 tw-shadow-sm tw-z-10 tw-flex tw-flex-col">

        {/* 顶部分页导航栏 */}
        <div className="tw-relative tw-flex tw-items-center tw-gap-2 tw-px-4 md:tw-px-6 tw-py-3 tw-border-b tw-border-slate-700 tw-bg-slate-800/90 tw-flex-shrink-0 tw-z-20">
          <button
            type="button"
            onClick={prevPage}
            disabled={currentPage === PAGE_KEYS.ACTUATOR}
            aria-label="上一页"
            className={`tw-inline-flex tw-items-center tw-gap-1 tw-px-3 tw-py-1.5 tw-rounded-md tw-text-sm tw-font-medium tw-transition ${
              currentPage === PAGE_KEYS.ACTUATOR
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
                className="tw-absolute tw-top-full tw-left-1/2 -tw-translate-x-1/2 tw-mt-2 tw-z-50 tw-min-w-[240px] tw-max-w-[360px] tw-rounded-lg tw-border tw-border-slate-700 tw-bg-slate-900 tw-shadow-xl tw-p-1 tw-list-none tw-m-0"
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
                            ? 'tw-bg-sky-600 tw-text-white'
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
                : 'tw-text-white tw-bg-sky-600 hover:tw-bg-sky-500 active:tw-scale-95 tw-shadow-sm'
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

          {/* Page 1:执行器 */}
          {currentPage === PAGE_KEYS.ACTUATOR && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500">
              <h2 className="tw-text-xl md:tw-text-2xl tw-font-semibold tw-mb-4 tw-flex tw-items-center tw-text-slate-100">
                <BookOpen className="tw-mr-3 tw-text-sky-400" size={24} />
                1. 执行器
              </h2>
              <div className="tw-bg-sky-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-sky-900/60 tw-mb-4">
                <p className="tw-mb-3">
                  <strong>执行器</strong>负责把电能变成旋转、位移和力矩。后面的 PID 并不是悬空工作的，它最终调的就是这些真实的运动响应。
                </p>
                <div className="tw-flex tw-items-start tw-bg-slate-800 tw-p-3 tw-rounded-lg tw-border tw-border-sky-800">
                  <Compass className="tw-text-sky-400 tw-mr-3 tw-flex-shrink-0 tw-mt-0.5" size={20} />
                  <p className="tw-text-xs tw-text-slate-300 tw-font-medium tw-italic">
                    右侧可以切换三种常见电动执行器，并观察命令与负载变化后它们在到位、掉速和发热上的差别。
                  </p>
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-sky-950/30 tw-border tw-border-sky-900/60 tw-rounded-xl tw-p-4">
                  <h3 className="tw-font-bold tw-text-sky-200 tw-text-sm tw-mb-1.5">伺服电机</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    带编码器做<strong>闭环纠偏</strong>，擅长精确到位与稳定保持。
                  </p>
                </div>

                <div className="tw-bg-amber-950/30 tw-border tw-border-amber-900/60 tw-rounded-xl tw-p-4">
                  <h3 className="tw-font-bold tw-text-amber-200 tw-text-sm tw-mb-1.5">步进电机</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    按脉冲<strong>逐格前进</strong>，结构简单、低速有力，但负载大时可能失步。
                  </p>
                </div>

                <div className="tw-bg-emerald-950/30 tw-border tw-border-emerald-900/60 tw-rounded-xl tw-p-4">
                  <h3 className="tw-font-bold tw-text-emerald-200 tw-text-sm tw-mb-1.5">直流无刷电机 (BLDC)</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    <strong>高速高效</strong>、功率密度高，通常配合驱动器和减速器进入机器人关节。
                  </p>
                </div>
              </div>

              <div className="tw-mt-3 tw-text-xs tw-text-slate-400 tw-leading-relaxed">
                控制器能调到什么程度，最终取决于执行器本身的带宽、惯量和负载能力。
              </div>
            </div>
          )}

          {/* Page 3:开环控制 */}
          {currentPage === PAGE_KEYS.OPEN_LOOP && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <Wind className="tw-mr-3 tw-text-orange-400" size={24} />
                3. 开环控制
              </h2>

              <div className="tw-bg-orange-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-border tw-border-orange-900/60">
                <p className="tw-mb-2">
                  <strong>开环控制</strong>只根据输入命令直接驱动执行器，不读取输出结果。右侧风扇里，控制器只知道“给多少电压”，不知道实际转速有没有达到预期。
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-orange-200 tw-leading-loose">
                  u = V<sub>cmd</sub><br />
                  speed ≈ 2V<sub>cmd</sub> - disturbance<br />
                  控制器没有测量 speed
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-orange-300 tw-text-sm tw-mb-1.5">优点：简单、便宜、反应直接</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    只要环境比较稳定，输入和输出关系也比较确定，开环控制就能用很少的传感器和计算完成任务。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1.5">弱点：看不见扰动</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    一旦负载、摩擦或外部风阻改变，实际输出会偏离预期；控制器不会自动补偿，因为它没有反馈信号。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-sky-300 tw-text-sm tw-mb-1.5">机器人里的例子</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    预设电压、固定 PWM、按固定步数发脉冲，都带有开环味道。它们不一定错误，但需要知道“没有反馈”意味着什么。
                  </p>
                </div>
              </div>

              <div className="tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700 tw-text-xs tw-text-slate-400 tw-leading-relaxed">
                右侧玩法：启动系统后调电压，再施加阻力。观察橙色“按电压预测”的速度和白色实际速度如何分开。
              </div>
            </div>
          )}

          {/* Page 4:闭环控制 */}
          {currentPage === PAGE_KEYS.CLOSED_LOOP && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <div className="tw-flex tw-justify-between tw-items-center tw-gap-3">
                <h2 className="tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                  <Activity className="tw-mr-3 tw-text-emerald-400" size={24} />
                  4. 闭环控制
                </h2>
                <div
                  className={`tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-md tw-flex tw-items-center ${
                    closedWithinBand
                      ? 'tw-bg-emerald-950/60 tw-text-emerald-300'
                      : 'tw-bg-sky-950/60 tw-text-sky-300'
                  }`}
                >
                  <Crosshair size={12} className="tw-mr-1" />
                  {closedWithinBand ? '接近目标' : '反馈调节中'}
                </div>
              </div>

              <div className="tw-bg-emerald-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-border tw-border-emerald-900/60">
                <p className="tw-mb-2">
                  <strong>闭环控制</strong>会测量实际输出，把它和目标值比较，再根据误差修正输入。这里用一个最小的 P 控制器自动调电压，让风扇尽量保持目标转速。
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-emerald-200 tw-leading-loose">
                  e = speed<sub>target</sub> - speed<sub>actual</sub><br />
                  V ← V + K<sub>p</sub>e<br />
                  speed<sub>actual</sub> 被传感器反馈回来
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1.5">反馈回路</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    传感器提供当前转速，控制器计算误差，执行器改变电压，新的转速再回到传感器。这条闭合链路就是“闭环”。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1.5">自动补偿扰动</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    当阻力突然变大，实际转速下降，误差变正；控制器会升高电压，把转速拉回目标附近。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-blue-300 tw-text-sm tw-mb-1.5">通往 PID</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    这页只用了比例修正。后面的 Bang-Bang 和 PID 会继续讨论：控制律太粗会震荡，控制律更细则能兼顾速度、精度和稳定性。
                  </p>
                </div>
              </div>

              <div className="tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700 tw-text-xs tw-text-slate-400 tw-leading-relaxed">
                右侧玩法：设置目标转速并启动闭环，再增加阻力。注意绿色电压曲线会主动抬升，这就是反馈在补偿扰动。
              </div>
            </div>
          )}

          {/* Page 5:Bang-Bang 控制 */}
          {currentPage === PAGE_KEYS.BANG && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <h2 className="tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                <RotateCcw className="tw-mr-3 tw-text-blue-400" size={24} />
                5. Bang-Bang 控制
              </h2>

              <div className="tw-bg-blue-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-border tw-border-blue-900/60">
                <p className="tw-mb-2">
                  <strong>Bang-Bang 控制</strong>是最直接的反馈控制：只看误差在哪一侧，然后让电机全力正转或全力反转。
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-blue-200 tw-leading-loose">
                  e = q<sub>d</sub> - q<br />
                  u = +τ<sub>max</sub>, 当 q &lt; q<sub>d</sub> - δ<br />
                  u = -τ<sub>max</sub>, 当 q &gt; q<sub>d</sub> + δ
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-blue-300 tw-text-sm tw-mb-1.5">为什么会形成极限环</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    机械臂有惯性。即使控制器在目标附近立刻换向，关节也会带着动能继续冲过头，于是在目标两侧来回切换。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-amber-300 tw-text-sm tw-mb-1.5">死区 / 滞后 δ 的作用</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    δ 让控制器不要在目标点附近疯狂翻转。它能减少高频抖动，但会牺牲定位精度，目标附近会保留一个允许误差带。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1.5">为什么要升级到 PID</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    Bang-Bang 只有"满功率"和"反向满功率"，无法在接近目标时柔和减速。下一页的 PID 会把输出变成连续力矩。
                  </p>
                </div>
              </div>

              <div className="tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700 tw-text-xs tw-text-slate-400 tw-leading-relaxed">
                右侧可以改目标角、死区、电机扭矩和摩擦，观察白色实际轨迹如何围绕蓝色目标线震荡。
              </div>
            </div>
          )}

          {/* Page 6:三项各司其职 */}
          {currentPage === PAGE_KEYS.PID && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-6">
              <div className="tw-flex tw-justify-between tw-items-center">
                <h2 className="tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                  <Sliders className="tw-mr-3 tw-text-sky-400" size={24} />
                  6. PID 控制律
                </h2>
                <div className="tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-md tw-bg-sky-950/60 tw-text-sky-300 tw-flex tw-items-center">
                  <Crosshair size={12} className="tw-mr-1" />
                  误差 {error.toFixed(1)}
                </div>
              </div>

              {/* P/I/D 各司其职 */}
              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-rose-950/40 tw-p-4 tw-rounded-xl tw-border tw-border-rose-900/60">
                  <h4 className="tw-font-bold tw-text-rose-200 tw-text-sm tw-mb-1.5">P — Proportional (比例)</h4>
                  <p className="tw-text-xs tw-text-rose-300/90 tw-leading-relaxed">
                    "误差越大,力量越大"。把当前误差乘 K<sub>p</sub> 作反馈,决定系统对偏差的<strong>反应速度</strong>——太小反应迟钝,太大容易冲过头并震荡。
                  </p>
                </div>
                <div className="tw-bg-emerald-950/40 tw-p-4 tw-rounded-xl tw-border tw-border-emerald-900/60">
                  <h4 className="tw-font-bold tw-text-emerald-200 tw-text-sm tw-mb-1.5">I — Integral (积分)</h4>
                  <p className="tw-text-xs tw-text-emerald-300/90 tw-leading-relaxed">
                    累积历史误差,负责<strong>消除稳态偏差</strong>——无人机受重力影响时,仅靠 P 会差一点点到不了目标,I 项长期累加能补上"最后一厘米"。代价是容易<em>积分饱和 (windup)</em>——见后面的 Windup 页。
                  </p>
                </div>
                <div className="tw-bg-purple-950/40 tw-p-4 tw-rounded-xl tw-border tw-border-purple-900/60">
                  <h4 className="tw-font-bold tw-text-purple-200 tw-text-sm tw-mb-1.5">D — Derivative (微分)</h4>
                  <p className="tw-text-xs tw-text-purple-300/90 tw-leading-relaxed">
                    "冲得越快,越提前刹车"。对误差变化率施加反向力,提供<strong>阻尼</strong>,抑制过冲与震荡,但对测量噪声敏感,常配低通滤波。
                  </p>
                </div>
              </div>

              <div className="tw-text-xs tw-text-slate-400 tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700 tw-leading-relaxed">
                💡 右侧可以直接拖滑块、选预设、点击画布改目标高度,公式会实时代入并给出合力 u。
              </div>
            </div>
          )}

          {/* Page 7:机械臂关节控制 */}
          {currentPage === PAGE_KEYS.ARM && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500 tw-space-y-5">
              <div className="tw-flex tw-justify-between tw-items-center tw-gap-3">
                <h2 className="tw-text-2xl tw-font-semibold tw-flex tw-items-center tw-text-slate-100">
                  <Compass className="tw-mr-3 tw-text-emerald-400" size={24} />
                  7. 机械臂关节控制
                </h2>
                <div
                  className={`tw-text-xs tw-px-3 tw-py-1.5 tw-rounded-md tw-flex tw-items-center ${
                    armStable
                      ? 'tw-bg-emerald-950/60 tw-text-emerald-300'
                      : 'tw-bg-sky-950/60 tw-text-sky-300'
                  }`}
                >
                  <Crosshair size={12} className="tw-mr-1" />
                  {armStable ? '已稳定' : '调节中'}
                </div>
              </div>

              <div className="tw-bg-emerald-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-border tw-border-emerald-900/60">
                <p className="tw-mb-2">
                  这页把同一套 PID 放进一个<strong>单自由度机械臂关节</strong>里。目标角是 q<sub>d</sub>，当前角是 q，控制器输出的是关节力矩 τ。
                </p>
                <div className="tw-font-mono tw-text-xs tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-text-emerald-200 tw-leading-loose">
                  e = q<sub>d</sub> - q<br />
                  τ<sub>pid</sub> = K<sub>p</sub>e + K<sub>i</sub>∫e dt + K<sub>d</sub>de/dt<br />
                  τ<sub>total</sub> = τ<sub>pid</sub> - τ<sub>gravity</sub> - τ<sub>friction</sub>
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-sky-300 tw-text-sm tw-mb-1.5">为什么机械臂比无人机更像真实关节</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    关节有转动惯量、摩擦、末端负载和重力力矩。即使目标角不变，手臂水平伸出时也要持续输出力矩才能抗住重力。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1.5">观察 I 项的作用</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    关闭 I 项时，P 项在目标附近变小，可能留下一个抗重力不够的静差；打开 I 项后，历史误差会慢慢补足这部分保持力矩。
                  </p>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-purple-300 tw-text-sm tw-mb-1.5">D 项像关节阻尼器</h3>
                  <p className="tw-text-xs tw-text-slate-300 tw-leading-relaxed">
                    K<sub>d</sub> 越大，越会惩罚高速冲向目标的动作。阻尼太低会来回摆，阻尼太高则响应变钝。
                  </p>
                </div>
              </div>

              <div className="tw-bg-slate-700/70 tw-p-2.5 tw-rounded-lg tw-border tw-border-slate-700 tw-text-xs tw-text-slate-400 tw-leading-relaxed">
                右侧可以点击或拖动机械臂画布改变目标角，再调质量、重力开关和 PID 参数观察响应曲线。
              </div>
            </div>
          )}

          {/* Page 8:积分饱和 (Windup) */}
          {currentPage === PAGE_KEYS.WINDUP && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500">
              <h2 className="tw-text-2xl tw-font-semibold tw-mb-5 tw-flex tw-items-center tw-text-slate-100">
                <AlertTriangle className="tw-mr-3 tw-text-rose-400" size={24} />
                8. 积分饱和 (Windup)
              </h2>

              <div className="tw-bg-rose-950/40 tw-rounded-xl tw-p-4 tw-text-sm tw-leading-relaxed tw-text-slate-200 tw-border tw-border-rose-900/60 tw-mb-4">
                <p className="tw-mb-2">
                  前面看到的 I 项可以消除稳态偏差，但在真实硬件上它藏着一个陷阱——<strong className="tw-text-rose-300">积分饱和 (Integral Windup)</strong>。
                </p>
                <p className="tw-text-xs tw-text-slate-300">
                  当误差长时间无法被消除（比如撞墙、电机力矩饱和），积分项会像滚雪球一样越攒越大；等约束解除的一瞬间，这笔"积分账单"会一股脑打出去，造成剧烈过冲。
                </p>
              </div>

              <div className="tw-grid tw-grid-cols-1 tw-gap-3 tw-mb-4">
                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-rose-300 tw-text-sm tw-mb-1.5">① 问题发生的条件</h3>
                  <ul className="tw-text-xs tw-text-slate-300 tw-leading-relaxed tw-list-disc tw-pl-4 tw-space-y-0.5">
                    <li>K<sub>i</sub> &gt; 0，积分一直在累</li>
                    <li>有物理约束让误差无法消除（墙、关节限位、最大电流）</li>
                    <li>执行器饱和：<em>算法想输出的力</em> 大于 <em>硬件能给的力</em></li>
                  </ul>
                </div>

                <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                  <h3 className="tw-font-bold tw-text-emerald-300 tw-text-sm tw-mb-1.5">② 常见 anti-windup 对策</h3>
                  <ul className="tw-text-xs tw-text-slate-300 tw-leading-relaxed tw-list-disc tw-pl-4 tw-space-y-0.5">
                    <li><strong>积分限幅</strong>：把 ∫e dτ 夹到 [-I<sub>max</sub>, I<sub>max</sub>]</li>
                    <li><strong>条件积分</strong>：执行器一旦饱和就暂停累加</li>
                    <li><strong>反算 (back-calculation)</strong>：用"实际输出 - 算法输出"的差值反向修正积分</li>
                  </ul>
                </div>
              </div>

              <div className="tw-bg-amber-950/40 tw-rounded-xl tw-p-3 tw-border tw-border-amber-900/60 tw-text-xs tw-text-amber-100 tw-leading-relaxed">
                💡 右侧玩法：把 K<sub>i</sub> 拉到 1 以上，等"积分账单"红光亮起，再点 <strong>移除障碍物</strong>，观察方块是如何"复仇式"冲过目标线的。
              </div>
            </div>
          )}

          {/* Page 2:齿轮传动 */}
          {currentPage === PAGE_KEYS.GEAR && (
            <div className="tw-animate-in tw-fade-in tw-slide-in-from-right-4 tw-duration-500">
              <h2 className="tw-text-2xl tw-font-semibold tw-mb-6 tw-flex tw-items-center tw-text-slate-100">
                <Cog className="tw-mr-3 tw-text-amber-400" size={24} />
                2. 齿轮传动 (Gear Ratio)
              </h2>

              <div className="tw-bg-amber-950/40 tw-rounded-xl tw-p-6 tw-text-base tw-leading-relaxed tw-text-slate-200 tw-shadow-inner tw-border tw-border-amber-900/60 tw-mb-6">
                <p className="tw-mb-3">
                  BLDC 电机往往"转得快、力矩小",而机器人关节要的是"慢而有力"。<strong>齿轮箱</strong>负责这个换算:
                </p>
                <div className="tw-font-mono tw-text-sm tw-bg-slate-900 tw-p-3 tw-rounded-lg tw-my-2 tw-text-amber-300 tw-leading-loose">
                  τ<sub>out</sub> = N · τ<sub>motor</sub><br />
                  ω<sub>out</sub> = ω<sub>motor</sub> / N<br />
                  I<sub>reflected</sub> = N² · I<sub>motor</sub>
                </div>
                <p className="tw-text-sm tw-text-slate-400 tw-mt-3">
                  传动比 N = 从动轮齿数 / 主动轮齿数。N &gt; 1 减速增扭(机器人常用),N &lt; 1 增速减扭。
                </p>
              </div>

              <div className="tw-text-xs tw-text-slate-500 tw-leading-relaxed tw-p-3 tw-bg-slate-900/50 tw-rounded-lg tw-border tw-border-slate-800">
                💡 QDD 路线(Unitree / MIT Cheetah)取 N≈6-10,保留反向可驱动;协作臂(UR / Franka)的谐波减速器 N≈50-160,换来高精度。右侧拖动滑块即可实时改变齿轮。
              </div>
            </div>
          )}

        </div>

      </div>

      {/* 右侧:画布 + 曲线 */}
      <div className="tw-w-full md:tw-w-1/2 lg:tw-w-7/12 tw-bg-slate-900/60 tw-backdrop-blur-sm tw-flex tw-flex-col tw-gap-4 tw-p-4 md:tw-p-6 tw-relative tw-overflow-hidden tw-shadow-inner">

        {currentPage === PAGE_KEYS.ACTUATOR ? (
          <div className="tw-flex tw-flex-col tw-gap-3 tw-pr-1">
            <div className="tw-grid tw-grid-cols-1 md:tw-grid-cols-3 tw-gap-2">
              {Object.entries(ACTUATOR_TYPES).map(([key, meta]) => {
                const active = key === actuatorType;
                return (
                  <button
                    key={key}
                    type="button"
                    onClick={() => setActuatorType(key)}
                    className={`tw-text-left tw-rounded-xl tw-border tw-p-3 tw-transition-all tw-shadow-sm ${
                      active
                        ? meta.buttonClass
                        : 'tw-border-slate-700 tw-bg-slate-800/70 tw-text-slate-300 hover:tw-border-slate-500'
                    }`}
                  >
                    <div className="tw-flex tw-items-center tw-justify-between tw-gap-3">
                      <span className="tw-font-semibold tw-text-sm">{meta.label}</span>
                      <span
                        className={`tw-text-[11px] tw-border tw-rounded-full tw-px-2 tw-py-0.5 ${active ? meta.chipClass : 'tw-border-slate-700 tw-text-slate-400'}`}
                      >
                        {meta.subtitle}
                      </span>
                    </div>
                  </button>
                );
              })}
            </div>

            <div className="tw-bg-slate-900/70 tw-border tw-border-slate-700 tw-rounded-xl tw-p-3 tw-text-xs tw-text-slate-400 tw-leading-relaxed">
              {actuatorMeta.description}
            </div>

            <div className="tw-relative tw-flex-shrink-0">
              <svg
                viewBox={`0 0 ${ACTUATOR_VIEW_W} ${ACTUATOR_VIEW_H}`}
                className="tw-w-full tw-h-auto tw-bg-slate-800 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                style={{ aspectRatio: `${ACTUATOR_VIEW_W} / ${ACTUATOR_VIEW_H}` }}
              >
                <defs>
                  <pattern id="actuatorGrid" width="36" height="36" patternUnits="userSpaceOnUse">
                    <path d="M 36 0 L 0 0 0 36" fill="none" stroke="#334155" strokeWidth="0.6" opacity="0.32" />
                  </pattern>
                </defs>
                <rect width={ACTUATOR_VIEW_W} height={ACTUATOR_VIEW_H} fill="#020617" />
                <rect width={ACTUATOR_VIEW_W} height={ACTUATOR_VIEW_H} fill="url(#actuatorGrid)" />

                <rect x="18" y="18" width="266" height="56" rx="10" fill="rgba(15,23,42,0.92)" stroke="#334155" />
                <text x="32" y="42" fill={actuatorMeta.color} fontSize="16" fontWeight="700">{actuatorMeta.label}</text>
                <text x="32" y="61" fill="#cbd5e1" fontSize="12">{actuatorMeta.subtitle} · 命令 {actuatorCommand}% · 负载 {actuatorLoad}%</text>

                <circle cx={actuatorCenterX} cy={actuatorCenterY} r="118" fill="#0f172a" stroke="#334155" strokeWidth="2" />
                <circle cx={actuatorCenterX} cy={actuatorCenterY} r="98" fill="#111827" stroke="#475569" strokeWidth="2" />

                {actuatorType === 'stepper' && actuatorStepAngles.map((angle) => {
                  const point = polarPoint(actuatorCenterX, actuatorCenterY, 110, angle);
                  const isTarget = Math.abs(angle - actuatorDemo.targetAngle) < 0.1;
                  return (
                    <circle
                      key={angle}
                      cx={point.x}
                      cy={point.y}
                      r={isTarget ? 6 : 4}
                      fill={isTarget ? '#f59e0b' : '#334155'}
                      opacity={isTarget ? 1 : 0.75}
                    />
                  );
                })}

                {actuatorType !== 'bldc' && (
                  <>
                    <line
                      x1={actuatorCenterX}
                      y1={actuatorCenterY}
                      x2={actuatorTargetPoint.x}
                      y2={actuatorTargetPoint.y}
                      stroke="#f87171"
                      strokeWidth="3"
                      strokeDasharray="6,6"
                    />
                    <circle cx={actuatorTargetPoint.x} cy={actuatorTargetPoint.y} r="8" fill="#f87171" />
                  </>
                )}

                {actuatorType === 'bldc' ? (
                  <>
                    {[0, 120, 240].map((angle) => {
                      const point = polarPoint(actuatorCenterX, actuatorCenterY, 78, angle);
                      return (
                        <circle
                          key={angle}
                          cx={point.x}
                          cy={point.y}
                          r="13"
                          fill="rgba(52,211,153,0.16)"
                          stroke="#34d399"
                          strokeWidth="2"
                        />
                      );
                    })}

                    <circle
                      cx={actuatorCenterX}
                      cy={actuatorCenterY}
                      r="72"
                      fill="none"
                      stroke={actuatorMeta.color}
                      strokeWidth="2"
                      strokeDasharray="10,8"
                      opacity="0.35"
                    />

                    <g transform={`translate(${actuatorCenterX}, ${actuatorCenterY}) rotate(${actuatorDemo.angle.toFixed(1)})`}>
                      {[0, 120, 240].map((angle) => (
                        <g key={angle} transform={`rotate(${angle})`}>
                          <path
                            d="M 0 -12 L 14 -66 Q 22 -80 36 -82 L 14 -22 Z"
                            fill={actuatorMeta.color}
                            opacity="0.9"
                          />
                        </g>
                      ))}
                    </g>
                  </>
                ) : (
                  <>
                    <line
                      x1={actuatorCenterX}
                      y1={actuatorCenterY}
                      x2={actuatorActualPoint.x}
                      y2={actuatorActualPoint.y}
                      stroke={actuatorMeta.color}
                      strokeWidth="10"
                      strokeLinecap="round"
                    />
                    <circle cx={actuatorActualPoint.x} cy={actuatorActualPoint.y} r="12" fill={actuatorMeta.color} />
                    <line
                      x1={actuatorActualPoint.x}
                      y1={actuatorActualPoint.y}
                      x2="512"
                      y2={actuatorLoadTrackY + (actuatorLoadTrackHeight - actuatorLoadHeight)}
                      stroke="#64748b"
                      strokeWidth="2"
                      strokeDasharray="5,5"
                      opacity="0.85"
                    />
                  </>
                )}

                <circle cx={actuatorCenterX} cy={actuatorCenterY} r="36" fill="#020617" stroke="#94a3b8" strokeWidth="2" />
                <circle cx={actuatorCenterX} cy={actuatorCenterY} r="10" fill="#e2e8f0" />

                <rect x="518" y="84" width="48" height="150" rx="10" fill="#0f172a" stroke="#475569" strokeWidth="2" />
                <rect
                  x="527"
                  y={actuatorLoadTrackY + (actuatorLoadTrackHeight - actuatorLoadHeight)}
                  width="30"
                  height={actuatorLoadHeight}
                  rx="7"
                  fill="#f97316"
                  opacity="0.9"
                />
                <text x="542" y="258" fill="#cbd5e1" fontSize="11" fontWeight="700" textAnchor="middle">负载</text>

                <rect x="448" y="22" width="118" height="48" rx="10" fill="rgba(15,23,42,0.92)" stroke="#334155" />
                <text x="464" y="42" fill="#f8fafc" fontSize="12" fontFamily="monospace" fontWeight="700">
                  扭矩 {actuatorTorquePct}%
                </text>
                <text x="464" y="59" fill="#94a3b8" fontSize="12" fontFamily="monospace" fontWeight="700">
                  热估计 {actuatorTemp}°C
                </text>
              </svg>
            </div>

            <div className="tw-grid tw-grid-cols-1 md:tw-grid-cols-2 tw-gap-2">
              <div className="tw-bg-slate-800/80 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm">
                <div className="tw-flex tw-items-center tw-justify-between tw-mb-2">
                  <label className="tw-font-bold tw-text-slate-100 tw-text-xs">命令输入</label>
                  <span className={`tw-text-xs tw-border tw-rounded-full tw-px-2 tw-py-1 ${actuatorMeta.chipClass}`}>
                    {actuatorType === 'bldc' ? '油门 / 速度指令' : '目标位置 / 步位'}
                  </span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="100"
                  step="1"
                  value={actuatorCommand}
                  onChange={(e) => setActuatorCommand(Number(e.target.value))}
                  className="tw-w-full tw-h-2.5 tw-rounded-lg tw-appearance-none tw-cursor-pointer"
                  style={{ accentColor: actuatorMeta.color }}
                />
                <p className="tw-mt-1.5 tw-text-[11px] tw-text-slate-400">
                  当前 {actuatorCommandLabel}: <span className="tw-text-slate-200 tw-font-mono">{actuatorCommandValue}</span>
                </p>
              </div>

              <div className="tw-bg-slate-800/80 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm">
                <div className="tw-flex tw-items-center tw-justify-between tw-mb-2">
                  <label className="tw-font-bold tw-text-slate-100 tw-text-xs">外部负载</label>
                  <span className="tw-text-xs tw-border tw-border-orange-800 tw-bg-orange-950/60 tw-text-orange-200 tw-rounded-full tw-px-2 tw-py-1">
                    阻力 / 惯量 / 摩擦
                  </span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="100"
                  step="1"
                  value={actuatorLoad}
                  onChange={(e) => setActuatorLoad(Number(e.target.value))}
                  className="tw-w-full tw-h-2.5 tw-rounded-lg tw-appearance-none tw-cursor-pointer tw-accent-orange-500"
                />
                <p className="tw-mt-1.5 tw-text-[11px] tw-text-slate-400">
                  负载越大，电机越容易出现掉速、发热或保持误差。
                </p>
              </div>
            </div>

            <div className="tw-grid tw-grid-cols-2 md:tw-grid-cols-4 tw-gap-2">
              <div className="tw-bg-sky-950/30 tw-border tw-border-sky-900 tw-p-3 tw-rounded-xl">
                <div className="tw-text-[11px] tw-text-sky-400 tw-mb-1">{actuatorCommandLabel}</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-sky-100">{actuatorCommandValue}</div>
              </div>
              <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-3 tw-rounded-xl">
                <div className="tw-text-[11px] tw-text-slate-400 tw-mb-1">{actuatorActualLabel}</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{actuatorActualValue}</div>
              </div>
              <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-3 tw-rounded-xl">
                <div className="tw-text-[11px] tw-text-slate-400 tw-mb-1">{actuatorSecondaryLabel}</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{actuatorSecondaryValue}</div>
              </div>
              <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-3 tw-rounded-xl">
                <div className="tw-text-[11px] tw-text-slate-400 tw-mb-1">负载 / 扭矩</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">
                  {actuatorLoad}% / {actuatorTorquePct}%
                </div>
              </div>
            </div>
          </div>
        ) : currentPage === PAGE_KEYS.GEAR ? (
          /* 齿轮传动可视化 + 交互 */
          <div className="tw-flex tw-flex-col tw-gap-4 tw-overflow-y-auto tw-pr-1">
            <div className="tw-relative tw-flex-shrink-0">
              <svg
                viewBox={`0 0 ${GEAR_VIEW_W} ${GEAR_VIEW_H}`}
                className="tw-w-full tw-h-auto tw-bg-slate-800 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                style={{ aspectRatio: `${GEAR_VIEW_W} / ${GEAR_VIEW_H}` }}
              >
                {/* 背景网格 */}
                <defs>
                  <pattern id="gearGrid" width="40" height="40" patternUnits="userSpaceOnUse">
                    <path d="M 40 0 L 0 0 0 40" fill="none" stroke="#334155" strokeWidth="0.5" opacity="0.3" />
                  </pattern>
                </defs>
                <rect width={GEAR_VIEW_W} height={GEAR_VIEW_H} fill="url(#gearGrid)" />

                {(() => {
                  const rA = tA * GEAR_MODULE;
                  const rB = tB * GEAR_MODULE;
                  const distance = rA + rB;
                  const centerX = GEAR_VIEW_W / 2;
                  const cy = GEAR_VIEW_H / 2;
                  const xA = centerX - distance / 2;
                  const xB = centerX + distance / 2;
                  return (
                    <>
                      {/* 轴心连线 */}
                      <line x1={xA} y1={cy} x2={xB} y2={cy} stroke="#475569" strokeWidth="1" strokeDasharray="4,4" opacity="0.6" />
                      {/* 齿轮 */}
                      {renderGear(xA, cy, tA, gearAngleA, '#fbbf24')}
                      {renderGear(xB, cy, tB, gearAngleB, '#34d399')}
                      {/* 底部标签 */}
                      <text x={xA} y={cy + rA + 28} fill="#cbd5e1" fontSize="14" textAnchor="middle" fontWeight="600">输入 · 电机</text>
                      <text x={xB} y={cy + rB + 28} fill="#cbd5e1" fontSize="14" textAnchor="middle" fontWeight="600">输出 · 关节</text>
                      {/* 顶部 RPM 标注 */}
                      <text x={xA} y={cy - rA - 14} fill="#fbbf24" fontSize="13" textAnchor="middle" fontFamily="monospace" fontWeight="700">
                        {rpm} RPM
                      </text>
                      <text x={xB} y={cy - rB - 14} fill="#34d399" fontSize="13" textAnchor="middle" fontFamily="monospace" fontWeight="700">
                        {rpmOut.toFixed(1)} RPM
                      </text>
                    </>
                  );
                })()}

                {/* HUD */}
                <rect x="14" y="14" width="180" height="60" rx="6" fill="rgba(15,23,42,0.85)" stroke="#475569" />
                <text x="24" y="36" fill="#fbbf24" fontSize="13" fontFamily="monospace" fontWeight="700">⚙️  齿数 A={tA}  B={tB}</text>
                <text x="24" y="58" fill="#38bdf8" fontSize="13" fontFamily="monospace" fontWeight="700">📐 传动比 = {gearRatio.toFixed(2)}</text>
              </svg>
            </div>

            {/* 滑块 */}
            <div className="tw-grid tw-grid-cols-1 md:tw-grid-cols-3 tw-gap-3">
              <div className="tw-bg-slate-800/80 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                  <label className="tw-font-bold tw-text-amber-300 tw-flex tw-items-center tw-text-xs">
                    <span className="tw-w-3 tw-h-3 tw-rounded-full tw-bg-amber-500 tw-mr-1.5 tw-shadow-[0_0_8px_rgba(245,158,11,0.6)] tw-inline-block"></span>
                    主动轮齿数
                  </label>
                  <span className="tw-text-xs tw-font-mono tw-bg-amber-950/60 tw-text-amber-200 tw-px-1.5 tw-py-0.5 tw-rounded tw-border tw-border-amber-800">{tA}</span>
                </div>
                <input
                  type="range"
                  min="10"
                  max="40"
                  value={tA}
                  onChange={(e) => setTA(Number(e.target.value))}
                  className="tw-w-full tw-h-2 tw-bg-amber-900 tw-rounded-lg tw-appearance-none tw-cursor-pointer tw-accent-amber-500"
                />
              </div>

              <div className="tw-bg-slate-800/80 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                  <label className="tw-font-bold tw-text-emerald-300 tw-flex tw-items-center tw-text-xs">
                    <span className="tw-w-3 tw-h-3 tw-rounded-full tw-bg-emerald-500 tw-mr-1.5 tw-shadow-[0_0_8px_rgba(16,185,129,0.6)] tw-inline-block"></span>
                    从动轮齿数
                  </label>
                  <span className="tw-text-xs tw-font-mono tw-bg-emerald-950/60 tw-text-emerald-200 tw-px-1.5 tw-py-0.5 tw-rounded tw-border tw-border-emerald-800">{tB}</span>
                </div>
                <input
                  type="range"
                  min="10"
                  max="60"
                  value={tB}
                  onChange={(e) => setTB(Number(e.target.value))}
                  className="tw-w-full tw-h-2 tw-bg-emerald-900 tw-rounded-lg tw-appearance-none tw-cursor-pointer tw-accent-emerald-500"
                />
              </div>

              <div className="tw-bg-slate-800/80 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                  <label className="tw-font-bold tw-text-sky-300 tw-flex tw-items-center tw-text-xs">
                    <span className="tw-w-3 tw-h-3 tw-rounded-full tw-bg-sky-500 tw-mr-1.5 tw-shadow-[0_0_8px_rgba(14,165,233,0.6)] tw-inline-block"></span>
                    输入转速 (RPM)
                  </label>
                  <span className="tw-text-xs tw-font-mono tw-bg-sky-950/60 tw-text-sky-200 tw-px-1.5 tw-py-0.5 tw-rounded tw-border tw-border-sky-800">{rpm}</span>
                </div>
                <input
                  type="range"
                  min="10"
                  max="150"
                  value={rpm}
                  onChange={(e) => setRpm(Number(e.target.value))}
                  className="tw-w-full tw-h-2 tw-bg-sky-900 tw-rounded-lg tw-appearance-none tw-cursor-pointer tw-accent-sky-500"
                />
              </div>
            </div>

            {/* 仪表盘 */}
            <div className="tw-grid tw-grid-cols-3 tw-gap-3">
              <div className="tw-bg-sky-950/40 tw-border tw-border-sky-900 tw-p-3 tw-rounded-xl tw-text-center">
                <div className="tw-text-[11px] tw-text-sky-400 tw-mb-1 tw-font-medium">传动比 N</div>
                <div className="tw-text-lg tw-font-mono tw-text-sky-200 tw-font-bold">{gearRatio.toFixed(2)}:1</div>
              </div>
              <div className="tw-bg-sky-950/40 tw-border tw-border-sky-900 tw-p-3 tw-rounded-xl tw-text-center">
                <div className="tw-text-[11px] tw-text-sky-400 tw-mb-1 tw-font-medium">输出转速</div>
                <div className="tw-text-lg tw-font-mono tw-text-sky-200 tw-font-bold">{rpmOut.toFixed(1)}</div>
              </div>
              <div
                className={`tw-p-3 tw-rounded-xl tw-text-center tw-border ${
                  gearRatio > 1
                    ? 'tw-bg-rose-950/40 tw-border-rose-900'
                    : gearRatio < 1
                      ? 'tw-bg-emerald-950/40 tw-border-emerald-900'
                      : 'tw-bg-sky-950/40 tw-border-sky-900'
                }`}
              >
                <div
                  className={`tw-text-[11px] tw-mb-1 tw-font-medium ${
                    gearRatio > 1 ? 'tw-text-rose-400' : gearRatio < 1 ? 'tw-text-emerald-400' : 'tw-text-sky-400'
                  }`}
                >
                  状态
                </div>
                <div
                  className={`tw-text-sm tw-font-bold ${
                    gearRatio > 1 ? 'tw-text-rose-200' : gearRatio < 1 ? 'tw-text-emerald-200' : 'tw-text-sky-200'
                  }`}
                >
                  {gearRatio > 1 ? '减速增扭' : gearRatio < 1 ? '增速减扭' : '1:1 传递'}
                </div>
              </div>
            </div>
          </div>
        ) : currentPage === PAGE_KEYS.OPEN_LOOP ? (
          /* 开环控制风扇演示 */
          <div className="tw-grid tw-grid-cols-1 xl:tw-grid-cols-[minmax(0,1fr)_300px] xl:tw-grid-rows-[minmax(0,1fr)_136px] tw-gap-3 tw-flex-1 tw-min-h-0 tw-overflow-y-auto xl:tw-overflow-hidden tw-pr-1">
            <div className="tw-relative tw-min-h-0 tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-overflow-hidden">
              <svg
                viewBox={`0 0 ${FAN_SIM_W} ${FAN_SIM_H}`}
                preserveAspectRatio="xMidYMid meet"
                className="tw-w-full tw-h-full tw-bg-slate-950"
              >
                <defs>
                  <pattern id="openFanGrid" width="40" height="40" patternUnits="userSpaceOnUse">
                    <path d="M 40 0 L 0 0 0 40" fill="none" stroke="#334155" strokeWidth="0.55" opacity="0.25" />
                  </pattern>
                  <radialGradient id="openFanGlow" cx="50%" cy="50%" r="50%">
                    <stop offset="0%" stopColor="#fb923c" stopOpacity="0.32" />
                    <stop offset="100%" stopColor="#fb923c" stopOpacity="0" />
                  </radialGradient>
                </defs>
                <rect width={FAN_SIM_W} height={FAN_SIM_H} fill="#020617" />
                <rect width={FAN_SIM_W} height={FAN_SIM_H} fill="url(#openFanGrid)" />
                <circle cx="310" cy="205" r="166" fill="url(#openFanGlow)" />

                {openResistance > 0 && (
                  <g opacity="0.9">
                    <rect x="536" y="88" width="72" height="234" rx="14" fill="rgba(251,146,60,0.15)" stroke="#fb923c" strokeWidth="2" />
                    {Array.from({ length: 6 }).map((_, idx) => (
                      <path
                        key={idx}
                        d={`M 620 ${112 + idx * 34} C 576 ${104 + idx * 34}, 566 ${138 + idx * 34}, 526 ${126 + idx * 34}`}
                        fill="none"
                        stroke="#fdba74"
                        strokeWidth="3"
                        strokeLinecap="round"
                        opacity="0.52"
                      />
                    ))}
                    <text x="572" y="344" fill="#fdba74" fontSize="14" fontWeight="700" textAnchor="middle">外部阻力</text>
                  </g>
                )}

                <line x1="310" y1="238" x2="310" y2="334" stroke="#475569" strokeWidth="18" strokeLinecap="round" />
                <path d="M 252 352 L 368 352 L 392 388 L 228 388 Z" fill="#1e293b" stroke="#64748b" strokeWidth="2" />
                <circle cx="310" cy="190" r="124" fill="#0f172a" stroke="#64748b" strokeWidth="5" />
                <circle cx="310" cy="190" r="104" fill="none" stroke="#334155" strokeWidth="2" strokeDasharray="6,8" />

                <g transform={`translate(310, 190) rotate(${openFan.bladeAngle.toFixed(1)})`}>
                  {[0, 90, 180, 270].map((angle) => (
                    <g key={angle} transform={`rotate(${angle})`}>
                      <path
                        d="M -10 -12 C 22 -28, 84 -18, 96 3 C 76 18, 30 24, 8 13 Z"
                        fill="#fb923c"
                        opacity="0.9"
                      />
                    </g>
                  ))}
                </g>
                <circle cx="310" cy="190" r="26" fill="#020617" stroke="#e2e8f0" strokeWidth="4" />
                <circle cx="310" cy="190" r="8" fill="#f97316" />

                {openFan.speed > 8 && Array.from({ length: 4 }).map((_, idx) => (
                  <line
                    key={idx}
                    x1={430 + idx * 18}
                    y1={126 + idx * 38}
                    x2={430 + Math.min(146, openFan.speed * 0.8) + idx * 18}
                    y2={126 + idx * 38}
                    stroke="#fed7aa"
                    strokeWidth="3"
                    strokeLinecap="round"
                    opacity={Math.min(0.72, openFan.speed / 190)}
                  />
                ))}

                <rect x="24" y="24" width="208" height="88" rx="10" fill="rgba(15,23,42,0.88)" stroke="#334155" />
                <text x="40" y="52" fill="#fb923c" fontSize="15" fontFamily="monospace" fontWeight="700">
                  输入电压 {openVoltage} V
                </text>
                <text x="40" y="78" fill="#f8fafc" fontSize="15" fontFamily="monospace" fontWeight="700">
                  实际 {Math.round(openFan.speed)} RPM
                </text>
                <text x="40" y="98" fill="#fdba74" fontSize="12" fontFamily="monospace" fontWeight="700">
                  控制器未读取反馈
                </text>

                <rect x={FAN_SIM_W - 190} y="24" width="164" height="42" rx="21" fill="rgba(251,146,60,0.16)" stroke="#fb923c" />
                <text x={FAN_SIM_W - 108} y="51" fill="#fdba74" fontSize="14" fontWeight="700" textAnchor="middle">
                  Open Loop
                </text>
              </svg>
            </div>

            <div className="tw-flex tw-flex-col tw-gap-2 tw-min-h-0">
              <div className="tw-grid tw-grid-cols-2 tw-gap-2">
                <button
                  type="button"
                  onClick={() => setOpenRunning((v) => !v)}
                  className={`tw-flex tw-items-center tw-justify-center tw-gap-1.5 tw-rounded-lg tw-px-3 tw-py-2 tw-text-xs tw-font-bold tw-transition ${
                    openRunning
                      ? 'tw-bg-red-950/50 tw-text-red-300 tw-border tw-border-red-800 hover:tw-bg-red-950'
                      : 'tw-bg-orange-600 tw-text-white tw-border tw-border-orange-500 hover:tw-bg-orange-500'
                  }`}
                >
                  {openRunning ? <Pause size={14} /> : <Play size={14} />}
                  {openRunning ? '暂停' : '启动'}
                </button>
                <button
                  type="button"
                  onClick={resetOpenLoopFan}
                  className="tw-flex tw-items-center tw-justify-center tw-gap-1.5 tw-rounded-lg tw-px-3 tw-py-2 tw-text-xs tw-font-bold tw-bg-slate-700 tw-text-slate-100 tw-border tw-border-slate-600 hover:tw-bg-slate-600 tw-transition"
                >
                  <RefreshCw size={14} />
                  重置
                </button>
              </div>

              <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                  <label className="tw-text-xs tw-font-bold tw-text-orange-300 tw-flex tw-items-center">
                    <Settings size={13} className="tw-mr-1" />
                    设定电压
                  </label>
                  <span className="tw-text-xs tw-font-mono tw-text-orange-200 tw-bg-orange-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-orange-800">{openVoltage} V</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="100"
                  step="1"
                  value={openVoltage}
                  onChange={(e) => setOpenVoltage(Number(e.target.value))}
                  className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-orange-500 tw-cursor-pointer"
                />
              </div>

              <button
                type="button"
                onClick={() => setOpenResistance((v) => (v > 0 ? 0 : 40))}
                className={`tw-rounded-xl tw-border tw-p-3 tw-text-left tw-transition ${
                  openResistance > 0
                    ? 'tw-bg-amber-600/20 tw-border-amber-500 tw-text-amber-100'
                    : 'tw-bg-slate-900 tw-border-slate-700 tw-text-slate-300 hover:tw-border-amber-500'
                }`}
              >
                <div className="tw-flex tw-items-center tw-justify-between tw-gap-2">
                  <span className="tw-text-xs tw-font-bold tw-flex tw-items-center">
                    <AlertTriangle size={14} className="tw-mr-1.5 tw-text-amber-400" />
                    外部阻力
                  </span>
                  <span className="tw-text-xs tw-font-mono">{openResistance > 0 ? `${openResistance}%` : '关闭'}</span>
                </div>
                <p className="tw-mt-1 tw-text-[11px] tw-text-slate-400">
                  阻力只影响物理对象，不会改变控制器输出。
                </p>
              </button>

              <div className="tw-grid tw-grid-cols-2 tw-gap-2 tw-flex-1 tw-min-h-0">
                <div className="tw-bg-orange-950/30 tw-border tw-border-orange-900 tw-p-2.5 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-orange-300 tw-mb-0.5">无负载预测</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-orange-100">{Math.round(openNoLoadSpeed)} RPM</div>
                </div>
                <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2.5 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">实际输出</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{Math.round(openFan.speed)} RPM</div>
                </div>
                <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2.5 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">扰动后上限</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{Math.round(openLoadedSpeed)} RPM</div>
                </div>
                <div className="tw-bg-amber-950/30 tw-border tw-border-amber-900 tw-p-2.5 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-amber-300 tw-mb-0.5">偏离量</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-amber-100">{Math.round(openSpeedDrop)} RPM</div>
                </div>
              </div>
            </div>

            <div className="tw-flex-shrink-0 xl:tw-col-span-2">
              <div className="tw-flex tw-items-center tw-justify-between tw-mb-1">
                <div className="tw-text-xs tw-font-bold tw-text-slate-400 tw-flex tw-items-center">
                  <Activity size={14} className="tw-mr-1.5" /> 开环输出响应
                </div>
                <div className="tw-flex tw-gap-3 tw-text-[10px] tw-text-slate-400">
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-orange-400"></span>预测</span>
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-white"></span>实际</span>
                </div>
              </div>
              <svg
                viewBox={`0 0 ${FAN_CHART_W} ${FAN_CHART_H}`}
                preserveAspectRatio="none"
                className="tw-w-full tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                style={{ height: '108px' }}
              >
                <rect width={FAN_CHART_W} height={FAN_CHART_H} fill="#020617" />
                {Array.from({ length: 5 }).map((_, idx) => (
                  <line
                    key={idx}
                    x1="0"
                    y1={(idx / 4) * FAN_CHART_H}
                    x2={FAN_CHART_W}
                    y2={(idx / 4) * FAN_CHART_H}
                    stroke="#334155"
                    strokeWidth="0.8"
                    opacity="0.55"
                  />
                ))}
                <polyline points={openReferencePoints} fill="none" stroke="#fb923c" strokeWidth="1.7" strokeDasharray="6,5" opacity="0.9" />
                <polyline points={openActualPoints} fill="none" stroke="#f8fafc" strokeWidth="2.3" strokeLinejoin="round" strokeLinecap="round" />
              </svg>
            </div>
          </div>
        ) : currentPage === PAGE_KEYS.CLOSED_LOOP ? (
          /* 闭环控制风扇演示 */
          <div className="tw-grid tw-grid-cols-1 xl:tw-grid-cols-[minmax(0,1fr)_300px] xl:tw-grid-rows-[minmax(0,1fr)_136px] tw-gap-3 tw-flex-1 tw-min-h-0 tw-overflow-y-auto xl:tw-overflow-hidden tw-pr-1">
            <div className="tw-relative tw-min-h-0 tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-overflow-hidden">
              <svg
                viewBox={`0 0 ${FAN_SIM_W} ${FAN_SIM_H}`}
                preserveAspectRatio="xMidYMid meet"
                className="tw-w-full tw-h-full tw-bg-slate-950"
              >
                <defs>
                  <pattern id="closedFanGrid" width="40" height="40" patternUnits="userSpaceOnUse">
                    <path d="M 40 0 L 0 0 0 40" fill="none" stroke="#334155" strokeWidth="0.55" opacity="0.25" />
                  </pattern>
                  <radialGradient id="closedFanGlow" cx="50%" cy="50%" r="50%">
                    <stop offset="0%" stopColor="#34d399" stopOpacity="0.3" />
                    <stop offset="100%" stopColor="#34d399" stopOpacity="0" />
                  </radialGradient>
                </defs>
                <rect width={FAN_SIM_W} height={FAN_SIM_H} fill="#020617" />
                <rect width={FAN_SIM_W} height={FAN_SIM_H} fill="url(#closedFanGrid)" />
                <circle cx="310" cy="205" r="166" fill="url(#closedFanGlow)" />

                {closedResistance > 0 && (
                  <g opacity="0.9">
                    <rect x="536" y="88" width="72" height="234" rx="14" fill="rgba(251,191,36,0.13)" stroke="#f59e0b" strokeWidth="2" />
                    {Array.from({ length: 6 }).map((_, idx) => (
                      <path
                        key={idx}
                        d={`M 620 ${112 + idx * 34} C 576 ${104 + idx * 34}, 566 ${138 + idx * 34}, 526 ${126 + idx * 34}`}
                        fill="none"
                        stroke="#fbbf24"
                        strokeWidth="3"
                        strokeLinecap="round"
                        opacity="0.52"
                      />
                    ))}
                    <text x="572" y="344" fill="#fbbf24" fontSize="14" fontWeight="700" textAnchor="middle">外部阻力</text>
                  </g>
                )}

                <path
                  d="M 92 94 C 60 94, 60 286, 92 286 M 92 190 L 162 190"
                  fill="none"
                  stroke="#22c55e"
                  strokeWidth="4"
                  strokeLinecap="round"
                  strokeDasharray="7,8"
                  opacity="0.65"
                />
                <text x="74" y="84" fill="#86efac" fontSize="13" fontWeight="700">反馈</text>

                <line x1="310" y1="238" x2="310" y2="334" stroke="#475569" strokeWidth="18" strokeLinecap="round" />
                <path d="M 252 352 L 368 352 L 392 388 L 228 388 Z" fill="#1e293b" stroke="#64748b" strokeWidth="2" />
                <circle cx="310" cy="190" r="124" fill="#0f172a" stroke="#64748b" strokeWidth="5" />
                <circle cx="310" cy="190" r="104" fill="none" stroke="#334155" strokeWidth="2" strokeDasharray="6,8" />

                <g transform={`translate(310, 190) rotate(${closedFan.bladeAngle.toFixed(1)})`}>
                  {[0, 90, 180, 270].map((angle) => (
                    <g key={angle} transform={`rotate(${angle})`}>
                      <path
                        d="M -10 -12 C 22 -28, 84 -18, 96 3 C 76 18, 30 24, 8 13 Z"
                        fill="#34d399"
                        opacity="0.9"
                      />
                    </g>
                  ))}
                </g>
                <circle cx="310" cy="190" r="26" fill="#020617" stroke="#e2e8f0" strokeWidth="4" />
                <circle cx="310" cy="190" r="8" fill="#10b981" />

                {closedFan.speed > 8 && Array.from({ length: 4 }).map((_, idx) => (
                  <line
                    key={idx}
                    x1={430 + idx * 18}
                    y1={126 + idx * 38}
                    x2={430 + Math.min(146, closedFan.speed * 0.8) + idx * 18}
                    y2={126 + idx * 38}
                    stroke="#bbf7d0"
                    strokeWidth="3"
                    strokeLinecap="round"
                    opacity={Math.min(0.72, closedFan.speed / 190)}
                  />
                ))}

                <rect x="24" y="24" width="222" height="96" rx="10" fill="rgba(15,23,42,0.88)" stroke="#334155" />
                <text x="40" y="52" fill="#34d399" fontSize="15" fontFamily="monospace" fontWeight="700">
                  目标 {closedTargetSpeed} RPM
                </text>
                <text x="40" y="78" fill="#f8fafc" fontSize="15" fontFamily="monospace" fontWeight="700">
                  实际 {Math.round(closedFan.speed)} RPM
                </text>
                <text x="40" y="102" fill="#fbbf24" fontSize="13" fontFamily="monospace" fontWeight="700">
                  电压 {closedFan.voltage.toFixed(1)} V
                </text>

                <rect
                  x={FAN_SIM_W - 190}
                  y="24"
                  width="164"
                  height="42"
                  rx="21"
                  fill={closedWithinBand ? 'rgba(16,185,129,0.22)' : 'rgba(37,99,235,0.18)'}
                  stroke={closedWithinBand ? '#34d399' : '#60a5fa'}
                />
                <text
                  x={FAN_SIM_W - 108}
                  y="51"
                  fill={closedWithinBand ? '#86efac' : '#bfdbfe'}
                  fontSize="14"
                  fontWeight="700"
                  textAnchor="middle"
                >
                  {closedWithinBand ? 'Stable' : 'Feedback'}
                </text>
              </svg>
            </div>

            <div className="tw-flex tw-flex-col tw-gap-2 tw-min-h-0">
              <div className="tw-grid tw-grid-cols-2 tw-gap-2">
                <button
                  type="button"
                  onClick={() => setClosedRunning((v) => !v)}
                  className={`tw-flex tw-items-center tw-justify-center tw-gap-1.5 tw-rounded-lg tw-px-3 tw-py-2 tw-text-xs tw-font-bold tw-transition ${
                    closedRunning
                      ? 'tw-bg-red-950/50 tw-text-red-300 tw-border tw-border-red-800 hover:tw-bg-red-950'
                      : 'tw-bg-emerald-600 tw-text-white tw-border tw-border-emerald-500 hover:tw-bg-emerald-500'
                  }`}
                >
                  {closedRunning ? <Pause size={14} /> : <Play size={14} />}
                  {closedRunning ? '暂停' : '启动'}
                </button>
                <button
                  type="button"
                  onClick={resetClosedLoopFan}
                  className="tw-flex tw-items-center tw-justify-center tw-gap-1.5 tw-rounded-lg tw-px-3 tw-py-2 tw-text-xs tw-font-bold tw-bg-slate-700 tw-text-slate-100 tw-border tw-border-slate-600 hover:tw-bg-slate-600 tw-transition"
                >
                  <RefreshCw size={14} />
                  重置
                </button>
              </div>

              <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                  <label className="tw-text-xs tw-font-bold tw-text-emerald-300 tw-flex tw-items-center">
                    <Crosshair size={13} className="tw-mr-1" />
                    目标转速
                  </label>
                  <span className="tw-text-xs tw-font-mono tw-text-emerald-200 tw-bg-emerald-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-emerald-800">{closedTargetSpeed} RPM</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="200"
                  step="1"
                  value={closedTargetSpeed}
                  onChange={(e) => setClosedTargetSpeed(Number(e.target.value))}
                  className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-emerald-500 tw-cursor-pointer"
                />
              </div>

              <button
                type="button"
                onClick={() => setClosedResistance((v) => (v > 0 ? 0 : 60))}
                className={`tw-rounded-xl tw-border tw-p-3 tw-text-left tw-transition ${
                  closedResistance > 0
                    ? 'tw-bg-amber-600/20 tw-border-amber-500 tw-text-amber-100'
                    : 'tw-bg-slate-900 tw-border-slate-700 tw-text-slate-300 hover:tw-border-amber-500'
                }`}
              >
                <div className="tw-flex tw-items-center tw-justify-between tw-gap-2">
                  <span className="tw-text-xs tw-font-bold tw-flex tw-items-center">
                    <AlertTriangle size={14} className="tw-mr-1.5 tw-text-amber-400" />
                    外部阻力
                  </span>
                  <span className="tw-text-xs tw-font-mono">{closedResistance > 0 ? `${closedResistance}%` : '关闭'}</span>
                </div>
                <p className="tw-mt-1 tw-text-[11px] tw-text-slate-400">
                  阻力会造成误差，反馈控制器会提高电压补偿。
                </p>
              </button>

              <div className="tw-grid tw-grid-cols-2 tw-gap-2 tw-flex-1 tw-min-h-0">
                <div className="tw-bg-emerald-950/30 tw-border tw-border-emerald-900 tw-p-2.5 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-emerald-300 tw-mb-0.5">目标速度</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-emerald-100">{closedTargetSpeed} RPM</div>
                </div>
                <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2.5 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">实际速度</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{Math.round(closedFan.speed)} RPM</div>
                </div>
                <div
                  className={`tw-border tw-p-2.5 tw-rounded-lg ${
                    closedWithinBand ? 'tw-bg-emerald-950/40 tw-border-emerald-900' : 'tw-bg-slate-800/80 tw-border-slate-700'
                  }`}
                >
                  <div className={`tw-text-[10px] tw-mb-0.5 ${closedWithinBand ? 'tw-text-emerald-300' : 'tw-text-slate-400'}`}>误差 e</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{closedError.toFixed(1)} RPM</div>
                </div>
                <div className="tw-bg-amber-950/30 tw-border tw-border-amber-900 tw-p-2.5 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-amber-300 tw-mb-0.5">控制电压</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-amber-100">{closedFan.voltage.toFixed(1)} V</div>
                </div>
              </div>
            </div>

            <div className="tw-flex-shrink-0 xl:tw-col-span-2">
              <div className="tw-flex tw-items-center tw-justify-between tw-mb-1">
                <div className="tw-text-xs tw-font-bold tw-text-slate-400 tw-flex tw-items-center">
                  <Activity size={14} className="tw-mr-1.5" /> 闭环响应与控制输出
                </div>
                <div className="tw-flex tw-gap-3 tw-text-[10px] tw-text-slate-400">
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-emerald-400"></span>目标</span>
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-white"></span>实际</span>
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-amber-400"></span>电压</span>
                </div>
              </div>
              <svg
                viewBox={`0 0 ${FAN_CHART_W} ${FAN_CHART_H}`}
                preserveAspectRatio="none"
                className="tw-w-full tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                style={{ height: '108px' }}
              >
                <rect width={FAN_CHART_W} height={FAN_CHART_H} fill="#020617" />
                {Array.from({ length: 5 }).map((_, idx) => (
                  <line
                    key={idx}
                    x1="0"
                    y1={(idx / 4) * FAN_CHART_H}
                    x2={FAN_CHART_W}
                    y2={(idx / 4) * FAN_CHART_H}
                    stroke="#334155"
                    strokeWidth="0.8"
                    opacity="0.55"
                  />
                ))}
                <polyline points={closedTargetPoints} fill="none" stroke="#34d399" strokeWidth="1.7" strokeDasharray="6,5" opacity="0.9" />
                <polyline points={closedVoltagePoints} fill="none" stroke="#fbbf24" strokeWidth="1.5" opacity="0.72" />
                <polyline points={closedActualPoints} fill="none" stroke="#f8fafc" strokeWidth="2.3" strokeLinejoin="round" strokeLinecap="round" />
              </svg>
            </div>
          </div>
        ) : currentPage === PAGE_KEYS.BANG ? (
          /* Bang-Bang 控制可视化 */
          <div className="tw-grid tw-grid-cols-1 xl:tw-grid-cols-[minmax(0,1fr)_310px] xl:tw-grid-rows-[minmax(0,1fr)_140px] tw-gap-3 tw-flex-1 tw-min-h-0 tw-overflow-y-auto xl:tw-overflow-hidden tw-pr-1">
            <div className="tw-relative tw-min-h-0 tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-overflow-hidden">
              <svg
                viewBox={`0 0 ${BANG_SIM_W} ${BANG_SIM_H}`}
                preserveAspectRatio="xMidYMid meet"
                className="tw-w-full tw-h-full tw-bg-slate-950"
              >
                <defs>
                  <pattern id="bangGrid" width="36" height="36" patternUnits="userSpaceOnUse">
                    <path d="M 36 0 L 0 0 0 36" fill="none" stroke="#334155" strokeWidth="0.55" opacity="0.25" />
                  </pattern>
                  <linearGradient id="bangArmGradient" x1="0%" x2="0%" y1="0%" y2="100%">
                    <stop offset="0%" stopColor="#60a5fa" />
                    <stop offset="100%" stopColor="#2563eb" />
                  </linearGradient>
                </defs>
                <rect width={BANG_SIM_W} height={BANG_SIM_H} fill="#020617" />
                <rect width={BANG_SIM_W} height={BANG_SIM_H} fill="url(#bangGrid)" />

                <circle cx={bangCenterX} cy={bangCenterY} r="174" fill="none" stroke="#1e293b" strokeWidth="2" />
                <circle cx={bangCenterX} cy={bangCenterY} r={bangDialRadius} fill="none" stroke="#334155" strokeWidth="2" strokeDasharray="6,8" />
                <path
                  d={`M ${verticalDialPoint(bangCenterX, bangCenterY, bangDialRadius, bangLowerBound).x} ${verticalDialPoint(bangCenterX, bangCenterY, bangDialRadius, bangLowerBound).y}
                    A ${bangDialRadius} ${bangDialRadius} 0 ${bangHysteresis > 90 ? 1 : 0} 1 ${verticalDialPoint(bangCenterX, bangCenterY, bangDialRadius, bangUpperBound).x} ${verticalDialPoint(bangCenterX, bangCenterY, bangDialRadius, bangUpperBound).y}`}
                  fill="none"
                  stroke="#f59e0b"
                  strokeWidth="8"
                  strokeLinecap="round"
                  opacity="0.45"
                />
                <line
                  x1={bangCenterX}
                  y1={bangCenterY}
                  x2={bangTargetPoint.x}
                  y2={bangTargetPoint.y}
                  stroke="#3b82f6"
                  strokeWidth="7"
                  strokeLinecap="round"
                  opacity="0.35"
                />
                <circle cx={bangTargetPoint.x} cy={bangTargetPoint.y} r="9" fill="#3b82f6" />

                <g transform={`rotate(${bangPhys.angle.toFixed(2)}, ${bangCenterX}, ${bangCenterY})`}>
                  <rect
                    x={bangCenterX - 9}
                    y={bangCenterY - bangArmLength}
                    width="18"
                    height={bangArmLength}
                    rx="9"
                    fill="url(#bangArmGradient)"
                    stroke="#93c5fd"
                    strokeWidth="1"
                  />
                  <circle cx={bangCenterX} cy={bangCenterY - bangArmLength} r="12" fill="#60a5fa" stroke="#bfdbfe" strokeWidth="3" />
                </g>

                <circle cx={bangCenterX} cy={bangCenterY} r="34" fill="#020617" stroke="#64748b" strokeWidth="3" />
                <circle cx={bangCenterX} cy={bangCenterY} r="8" fill="#3b82f6" />

                <rect x="22" y="22" width="154" height="78" rx="10" fill="rgba(15,23,42,0.86)" stroke="#334155" />
                <text x="38" y="48" fill="#60a5fa" fontSize="15" fontFamily="monospace" fontWeight="700">
                  目标 {Math.round(bangTargetAngle)}°
                </text>
                <text x="38" y="72" fill="#f8fafc" fontSize="15" fontFamily="monospace" fontWeight="700">
                  当前 {Math.round(bangPhys.angle)}°
                </text>
                <text x="38" y="92" fill="#fbbf24" fontSize="12" fontFamily="monospace" fontWeight="700">
                  死区 ±{bangHysteresis.toFixed(1)}°
                </text>

                <rect
                  x={BANG_SIM_W - 144}
                  y="22"
                  width="120"
                  height="32"
                  rx="16"
                  fill={`${bangMotorMeta.color}26`}
                  stroke={bangMotorMeta.color}
                />
                <text
                  x={BANG_SIM_W - 84}
                  y="43"
                  fill={bangMotorMeta.color}
                  fontSize="13"
                  fontWeight="700"
                  textAnchor="middle"
                >
                  电机{bangMotorMeta.label}
                </text>
              </svg>
            </div>

            <div className="tw-flex tw-flex-col tw-gap-2 tw-min-h-0">
              <div className="tw-grid tw-grid-cols-2 tw-gap-2">
                <button
                  type="button"
                  onClick={() => setBangRunning((v) => !v)}
                  className={`tw-flex tw-items-center tw-justify-center tw-gap-1.5 tw-rounded-lg tw-px-3 tw-py-2 tw-text-xs tw-font-bold tw-transition ${
                    bangRunning
                      ? 'tw-bg-red-950/50 tw-text-red-300 tw-border tw-border-red-800 hover:tw-bg-red-950'
                      : 'tw-bg-emerald-600 tw-text-white tw-border tw-border-emerald-500 hover:tw-bg-emerald-500'
                  }`}
                >
                  {bangRunning ? <Pause size={14} /> : <Play size={14} />}
                  {bangRunning ? '暂停' : '运行'}
                </button>
                <button
                  type="button"
                  onClick={resetBangBang}
                  className="tw-flex tw-items-center tw-justify-center tw-gap-1.5 tw-rounded-lg tw-px-3 tw-py-2 tw-text-xs tw-font-bold tw-bg-slate-700 tw-text-slate-100 tw-border tw-border-slate-600 hover:tw-bg-slate-600 tw-transition"
                >
                  <RefreshCw size={14} />
                  归零
                </button>
              </div>

              <div className="tw-space-y-2">
                <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-blue-300">目标位置</label>
                    <span className="tw-text-xs tw-font-mono tw-text-blue-200 tw-bg-blue-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-blue-800">{Math.round(bangTargetAngle)}°</span>
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="180"
                    step="1"
                    value={bangTargetAngle}
                    onChange={(e) => setBangTargetAngle(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-blue-500 tw-cursor-pointer"
                  />
                </div>

                <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-amber-300">死区 δ</label>
                    <span className="tw-text-xs tw-font-mono tw-text-amber-200 tw-bg-amber-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-amber-800">±{bangHysteresis.toFixed(1)}°</span>
                  </div>
                  <input
                    type="range"
                    min="0.5"
                    max="15"
                    step="0.5"
                    value={bangHysteresis}
                    onChange={(e) => setBangHysteresis(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-amber-500 tw-cursor-pointer"
                  />
                </div>

                <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-emerald-300">电机扭矩</label>
                    <span className="tw-text-xs tw-font-mono tw-text-emerald-200 tw-bg-emerald-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-emerald-800">{Math.round(bangTorque * 100)}%</span>
                  </div>
                  <input
                    type="range"
                    min="0.1"
                    max="2"
                    step="0.1"
                    value={bangTorque}
                    onChange={(e) => setBangTorque(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-emerald-500 tw-cursor-pointer"
                  />
                </div>

                <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-purple-300">阻尼 / 摩擦</label>
                    <span className="tw-text-xs tw-font-mono tw-text-purple-200 tw-bg-purple-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-purple-800">{bangFriction.toFixed(2)}</span>
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="0.2"
                    step="0.01"
                    value={bangFriction}
                    onChange={(e) => setBangFriction(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-purple-500 tw-cursor-pointer"
                  />
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-2 tw-gap-2 tw-flex-1 tw-min-h-0">
                <div className="tw-bg-blue-950/30 tw-border tw-border-blue-900 tw-p-2 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-blue-300 tw-mb-0.5">误差 e</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-blue-100">{bangError.toFixed(1)}°</div>
                </div>
                <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">角速度</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{bangPhys.velocity.toFixed(1)}°/frame</div>
                </div>
                <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">上边界</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{bangUpperBound.toFixed(1)}°</div>
                </div>
                <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2 tw-rounded-lg">
                  <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">下边界</div>
                  <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{bangLowerBound.toFixed(1)}°</div>
                </div>
              </div>
            </div>

            <div className="tw-flex-shrink-0 xl:tw-col-span-2">
              <div className="tw-flex tw-items-center tw-justify-between tw-mb-1">
                <div className="tw-text-xs tw-font-bold tw-text-slate-400 tw-flex tw-items-center">
                  <Move size={14} className="tw-mr-1.5" /> 运动轨迹实时分析
                </div>
                <div className="tw-flex tw-gap-3 tw-text-[10px] tw-text-slate-400">
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-blue-500"></span>目标</span>
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-white"></span>实际</span>
                  <span className="tw-inline-flex tw-items-center tw-gap-1"><span className="tw-w-3 tw-h-0.5 tw-bg-red-400"></span>死区</span>
                </div>
              </div>
              <svg
                viewBox={`0 0 ${BANG_CHART_W} ${BANG_CHART_H}`}
                preserveAspectRatio="none"
                className="tw-w-full tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                style={{ height: '112px' }}
              >
                <rect width={BANG_CHART_W} height={BANG_CHART_H} fill="#020617" />
                {Array.from({ length: 5 }).map((_, idx) => (
                  <line
                    key={idx}
                    x1="0"
                    y1={(idx / 4) * BANG_CHART_H}
                    x2={BANG_CHART_W}
                    y2={(idx / 4) * BANG_CHART_H}
                    stroke="#334155"
                    strokeWidth="0.8"
                    opacity="0.55"
                  />
                ))}
                <polyline points={bangUpperPoints} fill="none" stroke="#ef4444" strokeWidth="1.2" strokeDasharray="4,4" opacity="0.65" />
                <polyline points={bangLowerPoints} fill="none" stroke="#ef4444" strokeWidth="1.2" strokeDasharray="4,4" opacity="0.65" />
                <polyline points={bangTargetPoints} fill="none" stroke="#3b82f6" strokeWidth="1.5" strokeDasharray="6,5" opacity="0.85" />
                <polyline points={bangAnglePoints} fill="none" stroke="#f8fafc" strokeWidth="2.2" strokeLinejoin="round" strokeLinecap="round" />
              </svg>
            </div>
          </div>
        ) : currentPage === PAGE_KEYS.ARM ? (
          /* 机械臂关节 PID 可视化 */
          <div className="tw-grid tw-grid-cols-1 xl:tw-grid-cols-[minmax(0,1fr)_300px] xl:tw-grid-rows-[minmax(0,1fr)_auto] tw-gap-2 tw-flex-1 tw-min-h-0 tw-overflow-y-auto xl:tw-overflow-hidden tw-pr-1">
            <div className="tw-relative tw-min-h-0 tw-flex-shrink-0 xl:tw-flex-shrink xl:tw-col-start-1 xl:tw-row-start-1">
              <svg
                ref={armSvgRef}
                viewBox={`0 0 ${ARM_SIM_W} ${ARM_SIM_H}`}
                preserveAspectRatio="xMidYMid meet"
                onPointerDown={(e) => {
                  e.currentTarget.setPointerCapture?.(e.pointerId);
                  handleArmPointer(e);
                }}
                onPointerMove={(e) => {
                  if (e.buttons === 1) handleArmPointer(e);
                }}
                className="tw-w-full tw-h-auto xl:tw-h-full tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm tw-cursor-crosshair tw-touch-none"
                style={{ aspectRatio: `${ARM_SIM_W} / ${ARM_SIM_H}`, backgroundColor: '#020617' }}
              >
                <defs>
                  <pattern id="armJointGrid" width="40" height="40" patternUnits="userSpaceOnUse">
                    <path d="M 40 0 L 0 0 0 40" fill="none" stroke="#334155" strokeWidth="0.6" opacity="0.28" />
                  </pattern>
                  <linearGradient id="armBodyGradient" x1="0%" x2="0%" y1="0%" y2="100%">
                    <stop offset="0%" stopColor="#94a3b8" />
                    <stop offset="48%" stopColor="#475569" />
                    <stop offset="100%" stopColor="#1e293b" />
                  </linearGradient>
                </defs>
                <rect width={ARM_SIM_W} height={ARM_SIM_H} fill="#020617" />
                <rect width={ARM_SIM_W} height={ARM_SIM_H} fill="url(#armJointGrid)" />

                <circle cx={armCenterX} cy={armCenterY} r="270" fill="none" stroke="#1e293b" strokeWidth="2" />
                <line x1={armCenterX - 300} y1={armCenterY} x2={armCenterX + 300} y2={armCenterY} stroke="#334155" strokeWidth="1.5" strokeDasharray="6,8" />
                <line x1={armCenterX} y1={armCenterY - 300} x2={armCenterX} y2={armCenterY + 300} stroke="#334155" strokeWidth="1.5" strokeDasharray="6,8" />

                <line
                  x1={armCenterX}
                  y1={armCenterY}
                  x2={armTargetEnd.x}
                  y2={armTargetEnd.y}
                  stroke="#60a5fa"
                  strokeWidth="16"
                  strokeLinecap="round"
                  strokeDasharray="10,12"
                  opacity="0.26"
                />
                <circle cx={armTargetEnd.x} cy={armTargetEnd.y} r="10" fill="#3b82f6" opacity="0.9" />

                <rect x={armCenterX - 46} y={armCenterY - 10} width="92" height="64" rx="8" fill="#1e293b" stroke="#475569" strokeWidth="2" />

                {armUseGravity && (
                  <g>
                    <line
                      x1={armEnd.x}
                      y1={armEnd.y + armMassRadius + 8}
                      x2={armEnd.x}
                      y2={armEnd.y + armMassRadius + 48}
                      stroke="#f97316"
                      strokeWidth="3"
                      strokeLinecap="round"
                    />
                    <polygon
                      points={`${armEnd.x - 7},${armEnd.y + armMassRadius + 42} ${armEnd.x + 7},${armEnd.y + armMassRadius + 42} ${armEnd.x},${armEnd.y + armMassRadius + 56}`}
                      fill="#f97316"
                    />
                    <text x={armEnd.x + 12} y={armEnd.y + armMassRadius + 38} fill="#fdba74" fontSize="12" fontWeight="700">
                      重力
                    </text>
                  </g>
                )}

                <g transform={`translate(${armCenterX}, ${armCenterY}) rotate(${armPhys.angle.toFixed(2)})`}>
                  <rect
                    x="-10"
                    y="-12"
                    width={ARM_LENGTH + 20}
                    height="24"
                    rx="12"
                    fill="url(#armBodyGradient)"
                    stroke="#cbd5e1"
                    strokeWidth="1"
                    opacity="0.96"
                  />
                  <circle cx={ARM_LENGTH} cy="0" r={armMassRadius} fill="#ef4444" stroke="#fecaca" strokeWidth="2" />
                  <circle cx={ARM_LENGTH - 4} cy="-4" r="4" fill="rgba(255,255,255,0.35)" />
                </g>

                <circle cx={armCenterX} cy={armCenterY} r="38" fill="#020617" stroke="#94a3b8" strokeWidth="2" />
                <circle cx={armCenterX} cy={armCenterY} r="11" fill="#e2e8f0" />

                <rect x="24" y="24" width="154" height="58" rx="9" fill="rgba(15,23,42,0.86)" stroke="#334155" />
                <text x="38" y="48" fill="#60a5fa" fontSize="16" fontFamily="monospace" fontWeight="700">
                  目标 {armTargetAngleLabel}
                </text>
                <text x="38" y="71" fill="#34d399" fontSize="16" fontFamily="monospace" fontWeight="700">
                  当前 {armCurrentAngleLabel}
                </text>

                <rect
                  x={ARM_SIM_W - 166}
                  y={ARM_SIM_H - 48}
                  width="138"
                  height="30"
                  rx="15"
                  fill={armStable ? '#10b981' : '#2563eb'}
                  opacity="0.96"
                />
                <text
                  x={ARM_SIM_W - 97}
                  y={ARM_SIM_H - 29}
                  fill="#ffffff"
                  fontSize="13"
                  fontWeight="700"
                  textAnchor="middle"
                >
                  {armStable ? '已就绪 Stable' : '调节中 Active'}
                </text>
              </svg>
            </div>

            <div className="tw-flex-shrink-0 xl:tw-col-span-2 xl:tw-col-start-1 xl:tw-row-start-2">
              <div className="tw-text-xs tw-font-bold tw-text-slate-400 tw-mb-1 tw-flex tw-items-center">
                <Activity size={14} className="tw-mr-1.5" /> 关节角响应曲线
              </div>
              <svg
                viewBox={`0 0 ${ARM_CHART_W} ${ARM_CHART_H}`}
                preserveAspectRatio="none"
                className="tw-w-full tw-bg-slate-950 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                style={{ height: '76px' }}
              >
                <rect width={ARM_CHART_W} height={ARM_CHART_H} fill="#020617" />
                <line x1="0" y1={ARM_CHART_H / 2} x2={ARM_CHART_W} y2={ARM_CHART_H / 2} stroke="#334155" strokeWidth="1" />
                <text x="12" y={ARM_CHART_H / 2 - 6} fill="#64748b" fontSize="11" fontFamily="monospace">0°</text>
                {armHistoryRef.current.length > 1 && (
                  <>
                    <polyline
                      points={armHistoryTargetPoints}
                      fill="none"
                      stroke="#ef4444"
                      strokeWidth="1.5"
                      strokeDasharray="5,5"
                      strokeLinejoin="round"
                      strokeLinecap="round"
                      opacity="0.85"
                    />
                    <polyline
                      points={armHistoryCurrentPoints}
                      fill="none"
                      stroke="#3b82f6"
                      strokeWidth="2.2"
                      strokeLinejoin="round"
                      strokeLinecap="round"
                    />
                  </>
                )}
              </svg>
            </div>

            <div className="tw-flex tw-flex-col tw-gap-2 tw-flex-shrink-0 xl:tw-col-start-2 xl:tw-row-start-1 xl:tw-h-full xl:tw-self-stretch xl:tw-overflow-hidden">
              <div className="tw-grid tw-grid-cols-1 md:tw-grid-cols-3 xl:tw-grid-cols-1 tw-gap-2">
                <div className="tw-bg-slate-900 tw-p-2.5 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-blue-300">P · 比例</label>
                    <span className="tw-text-xs tw-font-mono tw-text-blue-200 tw-bg-blue-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-blue-800">{armKp.toFixed(1)}</span>
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="50"
                    step="0.5"
                    value={armKp}
                    onChange={(e) => setArmKp(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-blue-500 tw-cursor-pointer"
                  />
                </div>

                <div className="tw-bg-slate-900 tw-p-2.5 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-emerald-300">I · 积分</label>
                    <span className="tw-text-xs tw-font-mono tw-text-emerald-200 tw-bg-emerald-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-emerald-800">{armKi.toFixed(2)}</span>
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="5"
                    step="0.05"
                    value={armKi}
                    onChange={(e) => setArmKi(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-emerald-500 tw-cursor-pointer"
                  />
                </div>

                <div className="tw-bg-slate-900 tw-p-2.5 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-justify-between tw-items-center tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-amber-300">D · 微分</label>
                    <span className="tw-text-xs tw-font-mono tw-text-amber-200 tw-bg-amber-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-amber-800">{armKd.toFixed(1)}</span>
                  </div>
                  <input
                    type="range"
                    min="0"
                    max="20"
                    step="0.2"
                    value={armKd}
                    onChange={(e) => setArmKd(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-amber-500 tw-cursor-pointer"
                  />
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-1 md:tw-grid-cols-2 xl:tw-grid-cols-1 tw-gap-2">
                <div className="tw-bg-slate-900 tw-p-2.5 tw-rounded-xl tw-border tw-border-slate-700">
                  <div className="tw-flex tw-items-center tw-justify-between tw-gap-3 tw-mb-1.5">
                    <label className="tw-text-xs tw-font-bold tw-text-slate-200">重力环境</label>
                    <input
                      type="checkbox"
                      checked={armUseGravity}
                      onChange={(e) => setArmUseGravity(e.target.checked)}
                      className="tw-w-4 tw-h-4 tw-rounded tw-accent-blue-500"
                    />
                  </div>
                  <div className="tw-flex tw-justify-between tw-text-[11px] tw-text-slate-400 tw-mb-1">
                    <span>末端负载 Mass</span>
                    <span className="tw-font-mono">{armMass.toFixed(1)}</span>
                  </div>
                  <input
                    type="range"
                    min="0.1"
                    max="5"
                    step="0.1"
                    value={armMass}
                    onChange={(e) => setArmMass(Number(e.target.value))}
                    className="tw-w-full tw-h-2 tw-rounded-lg tw-accent-orange-500 tw-cursor-pointer"
                  />
                </div>

                <div className="tw-grid tw-grid-cols-2 tw-gap-2">
                  {Object.entries(ARM_PRESETS).map(([key, p]) => (
                    <button
                      key={key}
                      type="button"
                      onClick={() => applyArmPreset(key)}
                      className="tw-text-xs tw-bg-slate-800 tw-border tw-border-slate-600 tw-px-2.5 tw-py-1.5 tw-rounded-lg hover:tw-border-emerald-400 hover:tw-text-emerald-300 hover:tw-bg-emerald-950/60 tw-transition-all tw-font-medium tw-text-slate-300 tw-shadow-sm tw-text-left tw-truncate"
                    >
                      {p.name}
                    </button>
                  ))}
                  <button
                    type="button"
                    onClick={resetArmJoint}
                    className="tw-col-span-2 tw-text-xs tw-bg-slate-700 tw-border tw-border-slate-600 tw-px-2.5 tw-py-1.5 tw-rounded-lg hover:tw-bg-slate-600 tw-transition-all tw-font-bold tw-text-slate-100"
                  >
                    重置关节
                  </button>
                </div>
              </div>

              <div className="tw-grid tw-grid-cols-2 tw-auto-rows-fr tw-gap-2 tw-flex-1 tw-min-h-0">
              <div className="tw-bg-sky-950/40 tw-border tw-border-sky-900 tw-p-2 tw-rounded-lg">
                <div className="tw-text-[10px] tw-text-sky-300 tw-mb-0.5">目标 qd</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-sky-100">{armTargetAngleLabel}</div>
              </div>
              <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2 tw-rounded-lg">
                <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">当前 q</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{armCurrentAngleLabel}</div>
              </div>
              <div
                className={`tw-border tw-p-2 tw-rounded-lg ${
                  armStable ? 'tw-bg-emerald-950/40 tw-border-emerald-900' : 'tw-bg-slate-800/80 tw-border-slate-700'
                }`}
              >
                <div className={`tw-text-[10px] tw-mb-0.5 ${armStable ? 'tw-text-emerald-300' : 'tw-text-slate-400'}`}>
                  误差 e
                </div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{armError.toFixed(1)}°</div>
              </div>
              <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2 tw-rounded-lg">
                <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">角速度</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{armPhys.velocity.toFixed(1)}°/s</div>
              </div>
              <div className="tw-bg-blue-950/30 tw-border tw-border-blue-900 tw-p-2 tw-rounded-lg">
                <div className="tw-text-[10px] tw-text-blue-300 tw-mb-0.5">P 项</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-blue-100">{armPhys.pTerm.toFixed(1)}</div>
              </div>
              <div className="tw-bg-emerald-950/30 tw-border tw-border-emerald-900 tw-p-2 tw-rounded-lg">
                <div className="tw-text-[10px] tw-text-emerald-300 tw-mb-0.5">I 项</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-emerald-100">{armPhys.iTerm.toFixed(1)}</div>
              </div>
              <div className="tw-bg-amber-950/30 tw-border tw-border-amber-900 tw-p-2 tw-rounded-lg">
                <div className="tw-text-[10px] tw-text-amber-300 tw-mb-0.5">D 项</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-amber-100">{armPhys.dTerm.toFixed(1)}</div>
              </div>
              <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-2 tw-rounded-lg">
                <div className="tw-text-[10px] tw-text-slate-400 tw-mb-0.5">合力矩 τ</div>
                <div className="tw-text-sm tw-font-mono tw-font-bold tw-text-slate-100">{armPhys.totalTorque.toFixed(1)}</div>
              </div>
              </div>
            </div>
          </div>
        ) : currentPage === PAGE_KEYS.WINDUP ? (
          /* 积分饱和 (Windup) 可视化 */
          <div className="tw-flex tw-flex-col tw-gap-3 tw-flex-1 tw-min-h-0">
            <div className="tw-relative tw-flex-1 tw-min-h-0">
              <svg
                viewBox={`0 0 ${WINDUP_SIM_W} ${WINDUP_SIM_H}`}
                preserveAspectRatio="xMidYMid meet"
                className="tw-w-full tw-h-full tw-bg-slate-800 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
              >
                <defs>
                  <pattern id="windupGrid" width="40" height="40" patternUnits="userSpaceOnUse">
                    <path d="M 40 0 L 0 0 0 40" fill="none" stroke="#334155" strokeWidth="0.5" opacity="0.3" />
                  </pattern>
                  <radialGradient id="windupGlow" cx="50%" cy="50%" r="50%">
                    <stop offset="0%" stopColor="#fa5252" stopOpacity="0.85" />
                    <stop offset="100%" stopColor="#fa5252" stopOpacity="0" />
                  </radialGradient>
                </defs>
                <rect width={WINDUP_SIM_W} height={WINDUP_SIM_H} fill="url(#windupGrid)" />

                {/* 目标区 */}
                <rect
                  x={WINDUP_TARGET_X - 20}
                  y="0"
                  width="40"
                  height={WINDUP_SIM_H}
                  fill="rgba(55,178,77,0.18)"
                />
                <line
                  x1={WINDUP_TARGET_X}
                  y1="0"
                  x2={WINDUP_TARGET_X}
                  y2={WINDUP_SIM_H}
                  stroke="#37b24d"
                  strokeWidth="2"
                  strokeDasharray="4,4"
                />
                <text x={WINDUP_TARGET_X - 22} y="26" fill="#2b8a3e" fontSize="18" fontWeight="700">Target</text>

                {/* 障碍物 */}
                {wallActive && (
                  <g>
                    <rect x={WINDUP_WALL_X} y="0" width={WINDUP_WALL_W} height={WINDUP_SIM_H} fill="#495057" />
                    {Array.from({ length: Math.ceil(WINDUP_SIM_H / 20) }).map((_, i) => (
                      <rect
                        key={i}
                        x={WINDUP_WALL_X}
                        y={i * 20}
                        width={WINDUP_WALL_W}
                        height="10"
                        fill="#868e96"
                      />
                    ))}
                  </g>
                )}

                {/* Windup 红色光晕 */}
                {showWindupGlow && (
                  <>
                    <circle
                      cx={wPhys.blockX}
                      cy={WINDUP_SIM_H / 2}
                      r={WINDUP_BLOCK / 2 + windupGlowSize}
                      fill="url(#windupGlow)"
                    />
                    <text
                      x={wPhys.blockX}
                      y={WINDUP_SIM_H / 2 - 46}
                      fill="#fa5252"
                      fontSize="22"
                      fontWeight="700"
                      textAnchor="middle"
                    >
                      WINDUP 积攒中!
                    </text>
                  </>
                )}

                {/* 方块 */}
                <rect
                  x={wPhys.blockX - WINDUP_BLOCK / 2}
                  y={WINDUP_SIM_H / 2 - WINDUP_BLOCK / 2}
                  width={WINDUP_BLOCK}
                  height={WINDUP_BLOCK}
                  fill="#228be6"
                  stroke="#1c7ed6"
                  strokeWidth="2"
                  rx="3"
                />

                {/* HUD */}
                <rect x="14" y="14" width="220" height="44" rx="8" fill="rgba(15,23,42,0.82)" stroke="#475569" />
                <text x="26" y="44" fill="#e2e8f0" fontSize="20" fontFamily="monospace" fontWeight="700">
                  误差 e = {windupError.toFixed(1)}
                </text>
              </svg>
            </div>

            {/* 仪表盘 */}
            <div className="tw-grid tw-grid-cols-3 tw-gap-2 tw-flex-shrink-0">
              <div className="tw-bg-slate-800/80 tw-border tw-border-slate-700 tw-p-3 tw-rounded-xl">
                <div className="tw-text-xs tw-text-slate-400 tw-mb-1">算法想输出</div>
                <div className="tw-text-base tw-font-mono tw-font-bold tw-text-slate-100">
                  {wPhys.calcForce.toFixed(1)}
                </div>
              </div>
              <div className="tw-bg-sky-950/40 tw-border tw-border-sky-900 tw-p-3 tw-rounded-xl">
                <div className="tw-text-xs tw-text-sky-300 tw-mb-1">电机实际给出</div>
                <div className="tw-text-base tw-font-mono tw-font-bold tw-text-sky-100">
                  {wPhys.actualForce.toFixed(1)}
                </div>
              </div>
              <div
                className={`tw-p-3 tw-rounded-xl tw-border ${
                  showWindupGlow
                    ? 'tw-bg-rose-950/60 tw-border-rose-800'
                    : 'tw-bg-slate-800/80 tw-border-slate-700'
                }`}
              >
                <div className={`tw-text-xs tw-mb-1 ${showWindupGlow ? 'tw-text-rose-300' : 'tw-text-slate-400'}`}>
                  积分账单 K<sub>i</sub>·∫e
                </div>
                <div
                  className={`tw-text-base tw-font-mono tw-font-bold ${
                    showWindupGlow ? 'tw-text-rose-200' : 'tw-text-slate-100'
                  }`}
                >
                  {windupITerm.toFixed(1)}
                </div>
              </div>
            </div>

            {/* 滑块 */}
            <div className="tw-grid tw-grid-cols-2 tw-gap-2 tw-flex-shrink-0">
              <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                  <label className="tw-text-sm tw-font-bold tw-text-rose-300">K<sub>p</sub> · 比例</label>
                  <span className="tw-text-sm tw-font-mono tw-text-rose-200 tw-bg-rose-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-rose-800">{windupKp.toFixed(1)}</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="10"
                  step="0.1"
                  value={windupKp}
                  onChange={(e) => setWindupKp(Number(e.target.value))}
                  className="tw-w-full tw-h-2.5 tw-rounded-lg tw-accent-rose-500 tw-cursor-pointer"
                />
              </div>

              <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                  <label className="tw-text-sm tw-font-bold tw-text-emerald-300">
                    K<sub>i</sub> · 积分 <span className="tw-text-xs tw-font-normal tw-text-slate-500">(0 = 安全 PD)</span>
                  </label>
                  <span className="tw-text-sm tw-font-mono tw-text-emerald-200 tw-bg-emerald-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-emerald-800">{windupKi.toFixed(2)}</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="5"
                  step="0.05"
                  value={windupKi}
                  onChange={(e) => setWindupKi(Number(e.target.value))}
                  className="tw-w-full tw-h-2.5 tw-rounded-lg tw-accent-emerald-500 tw-cursor-pointer"
                />
              </div>

              <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                  <label className="tw-text-sm tw-font-bold tw-text-purple-300">K<sub>d</sub> · 微分</label>
                  <span className="tw-text-sm tw-font-mono tw-text-purple-200 tw-bg-purple-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-purple-800">{windupKd.toFixed(1)}</span>
                </div>
                <input
                  type="range"
                  min="0"
                  max="30"
                  step="0.5"
                  value={windupKd}
                  onChange={(e) => setWindupKd(Number(e.target.value))}
                  className="tw-w-full tw-h-2.5 tw-rounded-lg tw-accent-purple-500 tw-cursor-pointer"
                />
              </div>

              <div className="tw-bg-slate-900 tw-p-3 tw-rounded-xl tw-border tw-border-slate-700">
                <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                  <label className="tw-text-sm tw-font-bold tw-text-amber-300">电机力矩上限</label>
                  <span className="tw-text-sm tw-font-mono tw-text-amber-200 tw-bg-amber-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-amber-800">{windupMaxF}</span>
                </div>
                <input
                  type="range"
                  min="10"
                  max="200"
                  step="10"
                  value={windupMaxF}
                  onChange={(e) => setWindupMaxF(Number(e.target.value))}
                  className="tw-w-full tw-h-2.5 tw-rounded-lg tw-accent-amber-500 tw-cursor-pointer"
                />
              </div>
            </div>

            {/* 按钮 */}
            <div className="tw-grid tw-grid-cols-2 tw-gap-2 tw-flex-shrink-0">
              <button
                type="button"
                onClick={() => setWallActive((v) => !v)}
                className={`tw-px-3 tw-py-3 tw-rounded-xl tw-text-base tw-font-bold tw-text-white tw-transition ${
                  wallActive
                    ? 'tw-bg-rose-600 hover:tw-bg-rose-500'
                    : 'tw-bg-emerald-600 hover:tw-bg-emerald-500'
                }`}
              >
                {wallActive ? '移除障碍物 (引发过冲)' : '重新放上障碍物'}
              </button>
              <button
                type="button"
                onClick={resetWindup}
                className="tw-px-3 tw-py-3 tw-rounded-xl tw-text-base tw-font-bold tw-text-white tw-bg-slate-600 hover:tw-bg-slate-500 tw-transition"
              >
                重置系统
              </button>
            </div>
          </div>
        ) : (
          /* PID 合并页:左列 仿真+曲线,右列 调参+公式 */
          <div className="tw-grid tw-grid-cols-1 lg:tw-grid-cols-2 tw-gap-4 tw-flex-1 tw-min-h-0">
            {/* 左列:仿真画布 + 历史曲线 */}
            <div className="tw-flex tw-flex-col tw-gap-3 tw-min-w-0 tw-min-h-0">
              <div className="tw-relative tw-flex-1 tw-min-h-0">
                <div
                  className="tw-absolute tw-inset-0 tw-pointer-events-none tw-opacity-[0.08] tw-rounded-xl"
                  style={{
                    backgroundImage: `linear-gradient(#64748b 1px, transparent 1px), linear-gradient(90deg, #64748b 1px, transparent 1px)`,
                    backgroundSize: '40px 40px',
                  }}
                />
                <svg
                  ref={svgRef}
                  viewBox={`0 0 ${SIM_W} ${SIM_H}`}
                  preserveAspectRatio="xMidYMid meet"
                  onPointerDown={handleSimPointerDown}
                  className="tw-w-full tw-h-full tw-bg-slate-800 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm tw-cursor-crosshair tw-touch-none">

                  {/* HUD 背景 */}
                  <rect x="14" y="14" width="200" height="82" rx="8" fill="rgba(15,23,42,0.85)" stroke="#475569" />
                  <text x="28" y="46" fill="#ef4444" fontSize="22" fontFamily="monospace" fontWeight="700">🎯 目标: {targetHeight}</text>
                  <text x="28" y="80" fill="#059669" fontSize="22" fontFamily="monospace" fontWeight="700">🚁 当前: {currentHeight}</text>

                  {/* 目标线 */}
                  <line x1="0" y1={targetY} x2={SIM_W} y2={targetY} stroke="#ef4444" strokeWidth="2" strokeDasharray="6,5" />
                  <text x={SIM_W - 10} y={targetY - 8} fill="#ef4444" fontSize="18" fontWeight="700" textAnchor="end">目标线</text>

                  {/* 无人机 */}
                  <g transform={`translate(${SIM_W / 2}, ${phys.currentY})`}>
                    <rect x="-26" y="-13" width="52" height="26" rx="6" fill="#10b981" />
                    <line x1="-38" y1="-13" x2="-14" y2="-13" stroke="#64748b" strokeWidth="3" />
                    <line x1="14" y1="-13" x2="38" y2="-13" stroke="#64748b" strokeWidth="3" />
                    <ellipse cx="-32" cy="-16" rx={18 - propPhase * 0.5} ry="2.5" fill="rgba(15,23,42,0.3)" />
                    <ellipse cx="32" cy="-16" rx={18 - ((propPhase + 5) % 10) * 0.5} ry="2.5" fill="rgba(15,23,42,0.3)" />
                    {thrust > 5 && (
                      <polygon
                        points={`-12,13 0,${13 + Math.min(thrust * 0.5, 50)} 12,13`}
                        fill="orange"
                        opacity="0.85"
                      />
                    )}
                  </g>
                </svg>
              </div>

              <div className="tw-flex-shrink-0">
                <div className="tw-text-sm tw-font-bold tw-text-slate-400 tw-mb-1.5 tw-flex tw-items-center">
                  <Activity size={14} className="tw-mr-1.5" /> 高度响应曲线(最近 {MAX_HISTORY} 帧)
                </div>
                <svg
                  viewBox={`0 0 ${GRAPH_W} ${GRAPH_H}`}
                  preserveAspectRatio="none"
                  className="tw-w-full tw-bg-slate-900 tw-rounded-xl tw-border tw-border-slate-700 tw-shadow-sm"
                  style={{ height: '140px' }}>
                  <line
                    x1="0"
                    y1={(targetY / SIM_H) * GRAPH_H}
                    x2={GRAPH_W}
                    y2={(targetY / SIM_H) * GRAPH_H}
                    stroke="#ef4444"
                    strokeWidth="1.5"
                    strokeDasharray="4,4"
                    opacity="0.6"
                  />
                  {historyRef.current.length > 1 && (
                    <polyline
                      points={historyPoints}
                      fill="none"
                      stroke="#0ea5e9"
                      strokeWidth="2"
                      strokeLinejoin="round"
                      strokeLinecap="round"
                    />
                  )}
                </svg>
              </div>
            </div>

            {/* 右列:② 调参 + ③ 公式 */}
            <div className="tw-flex tw-flex-col tw-gap-4 tw-min-w-0 tw-min-h-0">
              <div className="tw-flex-shrink-0">
                <div className="tw-text-sm tw-font-bold tw-text-slate-300 tw-mb-3 tw-flex tw-items-center">
                  <Sliders size={14} className="tw-mr-1.5 tw-text-sky-400" /> ② 动手调参
                </div>

                <div className="tw-grid tw-grid-cols-2 tw-gap-2 tw-mb-4">
                  {Object.entries(PRESETS).map(([key, p]) => (
                    <button
                      key={key}
                      type="button"
                      onClick={() => applyPreset(key)}
                      className="tw-text-sm tw-bg-slate-800 tw-border tw-border-slate-600 tw-px-3 tw-py-2 tw-rounded-lg hover:tw-border-sky-400 hover:tw-text-sky-300 hover:tw-bg-sky-950/60 tw-transition-all tw-font-medium tw-text-slate-300 tw-shadow-sm tw-text-left tw-truncate">
                      {p.name}
                    </button>
                  ))}
                </div>

                <div className="tw-space-y-3">
                  <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                    <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                      <label className="tw-text-sm tw-font-bold tw-text-rose-300">P · 比例 K<sub>p</sub></label>
                      <span className="tw-text-sm tw-font-mono tw-text-rose-200 tw-bg-rose-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-rose-800">{kp.toFixed(1)}</span>
                    </div>
                    <input
                      type="range"
                      min="0"
                      max="10"
                      step="0.1"
                      value={kp}
                      onChange={(e) => setKp(Number(e.target.value))}
                      className="tw-w-full tw-h-2.5 tw-bg-rose-900 tw-rounded-lg tw-accent-rose-500 tw-cursor-pointer"
                    />
                  </div>
                  <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                    <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                      <label className="tw-text-sm tw-font-bold tw-text-emerald-300">I · 积分 K<sub>i</sub></label>
                      <span className="tw-text-sm tw-font-mono tw-text-emerald-200 tw-bg-emerald-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-emerald-800">{ki.toFixed(2)}</span>
                    </div>
                    <input
                      type="range"
                      min="0"
                      max="2"
                      step="0.01"
                      value={ki}
                      onChange={(e) => setKi(Number(e.target.value))}
                      className="tw-w-full tw-h-2.5 tw-bg-emerald-900 tw-rounded-lg tw-accent-emerald-500 tw-cursor-pointer"
                    />
                  </div>
                  <div className="tw-bg-slate-900 tw-p-4 tw-rounded-xl tw-border tw-border-slate-700">
                    <div className="tw-flex tw-justify-between tw-items-center tw-mb-2">
                      <label className="tw-text-sm tw-font-bold tw-text-purple-300">D · 微分 K<sub>d</sub></label>
                      <span className="tw-text-sm tw-font-mono tw-text-purple-200 tw-bg-purple-950/60 tw-px-2 tw-py-0.5 tw-rounded tw-border tw-border-purple-800">{kd.toFixed(1)}</span>
                    </div>
                    <input
                      type="range"
                      min="0"
                      max="10"
                      step="0.1"
                      value={kd}
                      onChange={(e) => setKd(Number(e.target.value))}
                      className="tw-w-full tw-h-2.5 tw-bg-purple-900 tw-rounded-lg tw-accent-purple-500 tw-cursor-pointer"
                    />
                  </div>
                </div>
              </div>

              <div className="tw-flex-shrink-0">
                <div className="tw-text-sm tw-font-bold tw-text-slate-300 tw-mb-3 tw-flex tw-items-center">
                  <Calculator size={14} className="tw-mr-1.5 tw-text-sky-400" /> ③ 实时代入公式
                </div>
                <div className="tw-bg-slate-800 tw-rounded-xl tw-p-4 tw-font-mono tw-text-sm tw-shadow-inner tw-border tw-border-slate-700">
                  <div className="tw-text-sky-400 tw-mb-3 tw-border-b tw-border-slate-700 tw-pb-3 tw-leading-relaxed">
                    u(t) =
                    <span className="tw-text-rose-400"> K<sub>p</sub>·e</span> +
                    <span className="tw-text-emerald-300"> K<sub>i</sub>·∫e dτ</span> +
                    <span className="tw-text-purple-300"> K<sub>d</sub>·de/dt</span>
                  </div>
                  <div className="tw-space-y-1.5 tw-text-slate-400 tw-leading-relaxed">
                    <p>
                      e = {phys.currentY.toFixed(0)} − {targetY.toFixed(0)} = <span className="tw-text-white">{error.toFixed(1)}</span>
                    </p>
                    <p>
                      P = <span className="tw-text-rose-400">{kp.toFixed(1)}</span> · {error.toFixed(1)} = <span className="tw-text-rose-400">{pTerm.toFixed(1)}</span>
                    </p>
                    <p>
                      I = <span className="tw-text-emerald-300">{ki.toFixed(2)}</span> · {phys.integral.toFixed(1)} = <span className="tw-text-emerald-300">{iTerm.toFixed(1)}</span>
                    </p>
                    <p>
                      D = <span className="tw-text-purple-300">{kd.toFixed(1)}</span> · {((error - phys.lastError) / 0.1).toFixed(1)} = <span className="tw-text-purple-300">{dTerm.toFixed(1)}</span>
                    </p>
                  </div>
                  <div className="tw-mt-3 tw-p-3 tw-bg-slate-700 tw-rounded-lg tw-flex tw-items-center">
                    <Crosshair size={18} className="tw-text-sky-400 tw-mr-2" />
                    <span className="tw-text-white tw-font-semibold tw-text-base">
                      u = <span className="tw-text-sky-400">{output.toFixed(1)}</span> (合力)
                    </span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        )}

      </div>
    </div>
  );
};

export default App;
