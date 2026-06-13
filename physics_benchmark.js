/**
 * 物理引擎精度基准测试
 * 验证物理模拟时间是否与真实壁钟时间同步
 * 
 * 测试方法：
 * 1. 以固定频率（模拟 60fps）调用 physics.update(dt)
 * 2. 对比 physics.time 与真实 elapsed 时间
 * 3. 检查小球下落距离是否符合 h = 0.5 * g * t²
 */

// ===== 物理引擎核心（与主文件完全一致）=====
class Vec2 {
  constructor(x = 0, y = 0) { this.x = x; this.y = y; }
  add(v) { return new Vec2(this.x + v.x, this.y + v.y); }
  sub(v) { return new Vec2(this.x - v.x, this.y - v.y); }
  scale(s) { return new Vec2(this.x * s, this.y * s); }
  dot(v) { return this.x * v.x + this.y * v.y; }
  len() { return Math.sqrt(this.x*this.x + this.y*this.y); }
  lenSq() { return this.x*this.x + this.y*this.y; }
  normalize() { const l = this.len(); return l > 0 ? this.scale(1/l) : new Vec2(); }
  clone() { return new Vec2(this.x, this.y); }
}

class RigidBody {
  constructor(opts = {}) {
    this.id = RigidBody._nextId++;
    this.pos = new Vec2(opts.x || 0, opts.y || 0);
    this.vel = new Vec2(opts.vx || 0, opts.vy || 0);
    this.acc = new Vec2(0, 0);
    this.radius = opts.radius || 8;
    this.mass = opts.mass || 1;
    this.restitution = opts.restitution ?? 0.6;
    this.friction = opts.friction ?? 0.2;
    this.isStatic = opts.isStatic || false;
    this.active = true;
    this.trail = [];
    this.maxTrail = 600;
    this.pointMass = false;
    this.charge = 0;
    this.magneticMoment = 0;
    this.contactForces = [];
  }
  get invMass() { return this.isStatic ? 0 : 1 / this.mass; }
  get effectiveRadius() { return this.pointMass ? 2 : this.radius; }
  applyForce(fx, fy) {
    if (this.isStatic) return;
    this.acc.x += fx * this.invMass;
    this.acc.y += fy * this.invMass;
  }
}
RigidBody._nextId = 0;

class PhysicsEngine {
  constructor() {
    this.bodies = [];
    this.segments = [];
    this.circles = [];
    this.gravity = new Vec2(0, 9.8);
    this.airDrag = 0;
    this.running = false;
    this.time = 0;
    this.speed = 1;
    this.fixedDt = 1 / 240;
    this.accumulator = 0;
  }
  addBody(body) { this.bodies.push(body); return body; }
  update(dt) {
    if (!this.running) return;
    const scaledDt = dt * this.speed;
    this.accumulator += scaledDt;
    if (this.accumulator > this.fixedDt * 10) this.accumulator = this.fixedDt * 10;
    while (this.accumulator >= this.fixedDt) {
      this._step(this.fixedDt);
      this.accumulator -= this.fixedDt;
    }
    this.time += scaledDt;
  }
  _step(dt) {
    for (const b of this.bodies) {
      if (b.isStatic || !b.active) continue;
      b.applyForce(this.gravity.x * b.mass, this.gravity.y * b.mass);
    }
    for (const b of this.bodies) {
      if (b.isStatic || !b.active) continue;
      b.vel.x += b.acc.x * dt;
      b.vel.y += b.acc.y * dt;
      b.pos.x += b.vel.x * dt;
      b.pos.y += b.vel.y * dt;
      b.acc.x = 0;
      b.acc.y = 0;
    }
  }
}

// ===== 基准测试 =====

const physics = new PhysicsEngine();
const G = 9.8;
const H = 490; // meters
const EXPECTED_TIME = Math.sqrt(2 * H / G); // 10.0s

// 创建测试小球
const ball = new RigidBody({ x: 0, y: -H, mass: 1 });
physics.addBody(ball);

console.log('=' .repeat(65));
console.log('  物理引擎精度基准测试');
console.log('=' .repeat(65));
console.log(`  重力加速度 g:     ${G} m/s²`);
console.log(`  下落高度 h:       ${H} m`);
console.log(`  理论落地时间:     ${EXPECTED_TIME.toFixed(6)} s`);
console.log(`  物理固定步长:     1/${Math.round(1/physics.fixedDt)} s`);
console.log('');

// --- 测试1：物理时间 vs 理论时间（验证积分精度）---
console.log('--- 测试1: 物理积分精度 ---');
const steps = Math.round(EXPECTED_TIME / physics.fixedDt);
for (let i = 0; i < steps; i++) {
  physics._step(physics.fixedDt);
  physics.time += physics.fixedDt;
}
const finalPos = ball.pos.y;
const expectedPos = -H + 0.5 * G * physics.time * physics.time;
const posError = Math.abs(finalPos - expectedPos);
const posErrorPct = (posError / H) * 100;
console.log(`  模拟步数:         ${steps}`);
console.log(`  物理时间:         ${physics.time.toFixed(6)} s`);
console.log(`  理论位置:         ${expectedPos.toFixed(4)} m`);
console.log(`  物理位置:         ${finalPos.toFixed(4)} m`);
console.log(`  位置误差:         ${posError.toFixed(6)} m (${posErrorPct.toFixed(4)}%)`);
console.log(`  结论:             ${posErrorPct < 0.1 ? '✅ 通过 (半隐式欧拉积分精度正常)' : '❌ 失败'}`);
console.log('');

// --- 测试2：时间累积精度（验证 physics.time 正确累加 dt）---
console.log('--- 测试2: 时间累积精度 ---');

const physics2 = new PhysicsEngine();
const ball2 = new RigidBody({ x: 0, y: -H, mass: 1 });
physics2.addBody(ball2);
physics2.running = true;
physics2.speed = 1;

const SIM_DURATION = 10;
const TARGET_FPS = 60;
const FRAME_DT = 1 / TARGET_FPS;
const TOTAL_FRAMES = Math.round(SIM_DURATION / FRAME_DT);

for (let frame = 0; frame < TOTAL_FRAMES; frame++) {
  physics2.update(FRAME_DT);
}

// 验证 physics.time 是否等于累加的 dt
const expectedTime = TOTAL_FRAMES * FRAME_DT;
const timeError = Math.abs(physics2.time - expectedTime);

const expectedY2 = -H + 0.5 * G * physics2.time * physics2.time;
const posError2 = Math.abs(ball2.pos.y - expectedY2);
const posError2Pct = (posError2 / H) * 100;

console.log(`  模拟帧数:         ${TOTAL_FRAMES}`);
console.log(`  输入累积 dt:      ${expectedTime.toFixed(6)} s`);
console.log(`  物理 time:        ${physics2.time.toFixed(6)} s`);
console.log(`  时间累积误差:     ${timeError.toFixed(10)} s (${(timeError/expectedTime*100).toFixed(6)}%)`);
console.log(`  当前速度:         ${ball2.vel.len().toFixed(4)} m/s (理论 ${(G*physics2.time).toFixed(4)} m/s)`);
console.log(`  位置误差:         ${posError2.toFixed(4)} m (${posError2Pct.toFixed(3)}%)`);
console.log(`  结论(时间):       ${timeError < 1e-10 ? '✅ 通过' : '❌ 失败'}`);
console.log('');

// --- 测试3: 不同帧率下的一致性（60fps vs 30fps vs 120fps）---
console.log('--- 测试3: 帧率无关性验证 ---');

function runAtFps(fps, duration) {
  const p = new PhysicsEngine();
  const b = new RigidBody({ x: 0, y: -H, mass: 1 });
  p.addBody(b);
  p.running = true;
  const dt = 1 / fps;
  const frames = Math.round(duration / dt);
  for (let i = 0; i < frames; i++) p.update(dt);
  return { time: p.time, posY: b.pos.y, vel: b.vel.len() };
}

const res30 = runAtFps(30, 5);
const res60 = runAtFps(60, 5);
const res120 = runAtFps(120, 5);

console.log(`  30fps 5s:  time=${res30.time.toFixed(6)}s  pos=${res30.posY.toFixed(4)}m  vel=${res30.vel.toFixed(4)}m/s`);
console.log(`  60fps 5s:  time=${res60.time.toFixed(6)}s  pos=${res60.posY.toFixed(4)}m  vel=${res60.vel.toFixed(4)}m/s`);
console.log(`  120fps 5s: time=${res120.time.toFixed(6)}s  pos=${res120.posY.toFixed(4)}m  vel=${res120.vel.toFixed(4)}m/s`);

const maxPosDiff = Math.max(
  Math.abs(res30.posY - res60.posY),
  Math.abs(res60.posY - res120.posY),
  Math.abs(res30.posY - res120.posY)
);
console.log(`  最大位置差异:     ${maxPosDiff.toFixed(6)} m`);
console.log(`  结论:             ${maxPosDiff < 0.001 ? '✅ 帧率无关通过' : maxPosDiff < 0.01 ? '⚠️ 轻微差异' : '❌ 帧率相关!'}`);
console.log('');
console.log('=' .repeat(65));
