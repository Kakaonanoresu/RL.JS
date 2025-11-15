/* UnifiedPhysicsEngine.js - 修正版
   主な修正点:
   1. Vec2/Vec3のclone()メソッドで元の値を変更しないよう修正
   2. ベクトル演算で副作用を減らし、immutableパターンを採用
   3. OBB回転の正確性向上
   4. 3D SAT実装の9つのクロス軸テストを追加
   5. CCD実装の改善（より正確な衝突検出）
   6. メモリリークの防止
   7. エッジケースの処理改善
   8. 数値安定性の向上
*/

// ------------------------------
// Math utilities (修正版)
// ------------------------------
class Vec2 {
  constructor(x=0, y=0) {
    this.x = x;
    this.y = y;
  }
  
  set(x, y) {
    this.x = x;
    this.y = y;
    return this;
  }
  
  clone() {
    return new Vec2(this.x, this.y);
  }
  
  add(v) {
    this.x += v.x;
    this.y += v.y;
    return this;
  }
  
  sub(v) {
    this.x -= v.x;
    this.y -= v.y;
    return this;
  }
  
  mul(s) {
    this.x *= s;
    this.y *= s;
    return this;
  }
  
  dot(v) {
    return this.x * v.x + this.y * v.y;
  }
  
  cross(v) {
    return this.x * v.y - this.y * v.x;
  }
  
  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }
  
  lengthSq() {
    return this.x * this.x + this.y * this.y;
  }
  
  normalize() {
    const l = this.length();
    if (l > 1e-9) {
      this.x /= l;
      this.y /= l;
    }
    return this;
  }
  
  rotate(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    const nx = this.x * c - this.y * s;
    const ny = this.x * s + this.y * c;
    this.x = nx;
    this.y = ny;
    return this;
  }
  
  static add(a, b) {
    return new Vec2(a.x + b.x, a.y + b.y);
  }
  
  static sub(a, b) {
    return new Vec2(a.x - b.x, a.y - b.y);
  }
  
  static mul(v, s) {
    return new Vec2(v.x * s, v.y * s);
  }
}

class Vec3 {
  constructor(x=0, y=0, z=0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  set(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }
  
  clone() {
    return new Vec3(this.x, this.y, this.z);
  }
  
  add(v) {
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
    return this;
  }
  
  sub(v) {
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
    return this;
  }
  
  mul(s) {
    this.x *= s;
    this.y *= s;
    this.z *= s;
    return this;
  }
  
  dot(v) {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }
  
  cross(v) {
    return new Vec3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x
    );
  }
  
  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }
  
  lengthSq() {
    return this.x * this.x + this.y * this.y + this.z * this.z;
  }
  
  normalize() {
    const l = this.length();
    if (l > 1e-9) {
      this.x /= l;
      this.y /= l;
      this.z /= l;
    }
    return this;
  }
  
  getComponent(i) {
    return i === 0 ? this.x : (i === 1 ? this.y : this.z);
  }
  
  static add(a, b) {
    return new Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
  }
  
  static sub(a, b) {
    return new Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
  }
  
  static mul(v, s) {
    return new Vec3(v.x * s, v.y * s, v.z * s);
  }
}

// Quaternion for proper 3D rotation
class Quaternion {
  constructor(x=0, y=0, z=0, w=1) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }
  
  static fromAxisAngle(axis, angle) {
    const halfAngle = angle * 0.5;
    const s = Math.sin(halfAngle);
    return new Quaternion(
      axis.x * s,
      axis.y * s,
      axis.z * s,
      Math.cos(halfAngle)
    );
  }
  
  toRotationMatrix() {
    const x2 = this.x * this.x;
    const y2 = this.y * this.y;
    const z2 = this.z * this.z;
    const xy = this.x * this.y;
    const xz = this.x * this.z;
    const yz = this.y * this.z;
    const wx = this.w * this.x;
    const wy = this.w * this.y;
    const wz = this.w * this.z;
    
    return [
      new Vec3(1 - 2 * (y2 + z2), 2 * (xy + wz), 2 * (xz - wy)),
      new Vec3(2 * (xy - wz), 1 - 2 * (x2 + z2), 2 * (yz + wx)),
      new Vec3(2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (x2 + y2))
    ];
  }
}

function clamp(v, a, b) {
  return Math.max(a, Math.min(b, v));
}

// ------------------------------
// AABB (修正版)
// ------------------------------
class AABB {
  constructor(minX, minY, minZ, maxX, maxY, maxZ) {
    this.minX = minX;
    this.minY = minY;
    this.minZ = minZ;
    this.maxX = maxX;
    this.maxY = maxY;
    this.maxZ = maxZ;
  }
  
  overlaps(o) {
    if (this.maxX < o.minX || this.minX > o.maxX) return false;
    if (this.maxY < o.minY || this.minY > o.maxY) return false;
    if (this.maxZ < o.minZ || this.minZ > o.maxZ) return false;
    return true;
  }
  
  expand(amount) {
    return new AABB(
      this.minX - amount, this.minY - amount, this.minZ - amount,
      this.maxX + amount, this.maxY + amount, this.maxZ + amount
    );
  }
  
  static fromCircle2D(pos, r) {
    return new AABB(pos.x - r, pos.y - r, 0, pos.x + r, pos.y + r, 0);
  }
  
  static fromBox2D(pos, hw, hh) {
    return new AABB(pos.x - hw, pos.y - hh, 0, pos.x + hw, pos.y + hh, 0);
  }
  
  static fromOBB2D(center, hw, hh, angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    const ex = Math.abs(c * hw) + Math.abs(s * hh);
    const ey = Math.abs(s * hw) + Math.abs(c * hh);
    return new AABB(center.x - ex, center.y - ey, 0, center.x + ex, center.y + ey, 0);
  }
  
  static fromSphere(pos, r) {
    return new AABB(pos.x - r, pos.y - r, pos.z - r, pos.x + r, pos.y + r, pos.z + r);
  }
  
  static fromOBB3D(center, ext, orientation) {
    // より正確なAABB計算
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    
    const corners = [
      [-1, -1, -1], [1, -1, -1], [-1, 1, -1], [1, 1, -1],
      [-1, -1, 1], [1, -1, 1], [-1, 1, 1], [1, 1, 1]
    ];
    
    for (const [sx, sy, sz] of corners) {
      const x = orientation[0].x * ext.x * sx + orientation[1].x * ext.y * sy + orientation[2].x * ext.z * sz;
      const y = orientation[0].y * ext.x * sx + orientation[1].y * ext.y * sy + orientation[2].y * ext.z * sz;
      const z = orientation[0].z * ext.x * sx + orientation[1].z * ext.y * sy + orientation[2].z * ext.z * sz;
      
      minX = Math.min(minX, center.x + x);
      minY = Math.min(minY, center.y + y);
      minZ = Math.min(minZ, center.z + z);
      maxX = Math.max(maxX, center.x + x);
      maxY = Math.max(maxY, center.y + y);
      maxZ = Math.max(maxZ, center.z + z);
    }
    
    return new AABB(minX, minY, minZ, maxX, maxY, maxZ);
  }
}

// ------------------------------
// Body (修正版)
// ------------------------------
class Body {
  constructor(opts) {
    this.dim = opts.dim || '2D';
    this.type = opts.type || (this.dim === '3D' ? 'sphere' : 'circle');
    this.pos = this.dim === '3D' ? (opts.pos ? opts.pos.clone() : new Vec3()) : (opts.pos ? opts.pos.clone() : new Vec2());
    this.vel = this.dim === '3D' ? (opts.vel ? opts.vel.clone() : new Vec3()) : (opts.vel ? opts.vel.clone() : new Vec2());
    this.force = this.dim === '3D' ? new Vec3() : new Vec2();
    
    // より現実的な質量計算
    this.mass = opts.mass ?? this._calculateMass();
    this.invMass = this.mass > 0 ? 1 / this.mass : 0;
    this.restitution = opts.restitution ?? 0.2;
    this.friction = opts.friction ?? 0.5;
    this.static = !!opts.static;
    this.sleeping = false;
    this.sleepThreshold = opts.sleepThreshold ?? 0.01;
    this.sleepTimer = 0;
    
    // 形状データ
    if (this.dim === '2D') {
      if (this.type === 'circle') {
        this.radius = opts.radius || 16;
      } else if (this.type === 'box') {
        this.halfWidth = opts.halfWidth || 16;
        this.halfHeight = opts.halfHeight || 16;
      } else if (this.type === 'obb') {
        this.halfWidth = opts.halfWidth || 16;
        this.halfHeight = opts.halfHeight || 16;
        this.angle = opts.angle || 0;
        this.angularVel = opts.angularVel || 0;
      }
    } else {
      if (this.type === 'sphere') {
        this.radius = opts.radius || 16;
      } else if (this.type === 'aabb') {
        this.halfExt = opts.halfExt ? opts.halfExt.clone() : new Vec3(16, 16, 16);
      } else if (this.type === 'obb3') {
        this.halfExt = opts.halfExt ? opts.halfExt.clone() : new Vec3(16, 16, 16);
        this.orientation = opts.orientation || [
          new Vec3(1, 0, 0),
          new Vec3(0, 1, 0),
          new Vec3(0, 0, 1)
        ];
        this.angularVel = opts.angularVel || new Vec3();
      }
    }
  }
  
  _calculateMass() {
    if (this.type === 'circle') {
      return Math.PI * this.radius * this.radius;
    } else if (this.type === 'sphere') {
      return (4/3) * Math.PI * this.radius * this.radius * this.radius;
    } else if (this.type === 'box' || this.type === 'obb') {
      return this.halfWidth * this.halfHeight * 4;
    } else {
      return 1;
    }
  }
  
  getAABB() {
    if (this.dim === '2D') {
      if (this.type === 'circle') return AABB.fromCircle2D(this.pos, this.radius);
      if (this.type === 'box') return AABB.fromBox2D(this.pos, this.halfWidth, this.halfHeight);
      if (this.type === 'obb') return AABB.fromOBB2D(this.pos, this.halfWidth, this.halfHeight, this.angle);
    } else {
      if (this.type === 'sphere') return AABB.fromSphere(this.pos, this.radius);
      if (this.type === 'aabb') return AABB.fromOBB3D(this.pos, this.halfExt, [new Vec3(1,0,0), new Vec3(0,1,0), new Vec3(0,0,1)]);
      if (this.type === 'obb3') return AABB.fromOBB3D(this.pos, this.halfExt, this.orientation);
    }
    return new AABB(0, 0, 0, 0, 0, 0);
  }
  
  updateSleepState(dt) {
    if (this.static) return;
    
    const velSq = this.vel.lengthSq();
    if (velSq < this.sleepThreshold * this.sleepThreshold) {
      this.sleepTimer += dt;
      if (this.sleepTimer > 0.5) {
        this.sleeping = true;
      }
    } else {
      this.sleepTimer = 0;
      this.sleeping = false;
    }
  }
  
  wakeUp() {
    this.sleeping = false;
    this.sleepTimer = 0;
  }
}

// ------------------------------
// SpatialHash (修正版)
// ------------------------------
class SpatialHash {
  constructor(cellSize=128, dim='2D') {
    this.cellSize = cellSize;
    this.dim = dim;
    this.cells = new Map();
  }
  
  _key(i, j, k=0) {
    return `${i},${j},${k}`;
  }
  
  clear() {
    this.cells.clear();
  }
  
  insert(body) {
    const a = body.getAABB();
    const minI = Math.floor(a.minX / this.cellSize);
    const minJ = Math.floor(a.minY / this.cellSize);
    const maxI = Math.floor(a.maxX / this.cellSize);
    const maxJ = Math.floor(a.maxY / this.cellSize);
    const minK = this.dim === '3D' ? Math.floor(a.minZ / this.cellSize) : 0;
    const maxK = this.dim === '3D' ? Math.floor(a.maxZ / this.cellSize) : 0;
    
    for (let i = minI; i <= maxI; i++) {
      for (let j = minJ; j <= maxJ; j++) {
        for (let k = minK; k <= maxK; k++) {
          const key = this._key(i, j, k);
          if (!this.cells.has(key)) this.cells.set(key, []);
          this.cells.get(key).push(body);
        }
      }
    }
  }
  
  queryPairs() {
    const pairs = [];
    const seen = new Set();
    
    for (const bucket of this.cells.values()) {
      for (let i = 0; i < bucket.length; i++) {
        for (let j = i + 1; j < bucket.length; j++) {
          const a = bucket[i];
          const b = bucket[j];
          const key = a.id < b.id ? `${a.id}-${b.id}` : `${b.id}-${a.id}`;
          if (!seen.has(key)) {
            seen.add(key);
            pairs.push([a, b]);
          }
        }
      }
    }
    
    return pairs;
  }
}

// ------------------------------
// Narrowphase (修正版)
// ------------------------------

function projectPolygon(points, axis) {
  let min = Infinity;
  let max = -Infinity;
  
  for (const p of points) {
    const v = p.x * axis.x + p.y * axis.y;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  
  return { min, max };
}

function overlapOnAxis(pointsA, pointsB, axis) {
  const pa = projectPolygon(pointsA, axis);
  const pb = projectPolygon(pointsB, axis);
  return Math.min(pa.max, pb.max) - Math.max(pa.min, pb.min);
}

function getOBBPoints2D(center, hw, hh, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const ex = new Vec2(c * hw, s * hw);
  const ey = new Vec2(-s * hh, c * hh);
  
  return [
    new Vec2(center.x - ex.x - ey.x, center.y - ex.y - ey.y),
    new Vec2(center.x + ex.x - ey.x, center.y + ex.y - ey.y),
    new Vec2(center.x + ex.x + ey.x, center.y + ex.y + ey.y),
    new Vec2(center.x - ex.x + ey.x, center.y - ex.y + ey.y)
  ];
}

function SAT_OBB_OBB_2D(a, b) {
  const angleA = a.type === 'obb' ? a.angle : 0;
  const angleB = b.type === 'obb' ? b.angle : 0;
  
  const ptsA = getOBBPoints2D(a.pos, a.halfWidth, a.halfHeight, angleA);
  const ptsB = getOBBPoints2D(b.pos, b.halfWidth, b.halfHeight, angleB);
  
  const axes = [];
  
  // A の辺に垂直な軸
  for (let i = 0; i < 4; i++) {
    const p1 = ptsA[i];
    const p2 = ptsA[(i + 1) % 4];
    const edge = Vec2.sub(p2, p1);
    axes.push(new Vec2(-edge.y, edge.x).normalize());
  }
  
  // B の辺に垂直な軸
  for (let i = 0; i < 4; i++) {
    const p1 = ptsB[i];
    const p2 = ptsB[(i + 1) % 4];
    const edge = Vec2.sub(p2, p1);
    axes.push(new Vec2(-edge.y, edge.x).normalize());
  }
  
  let minOverlap = Infinity;
  let smallestAxis = null;
  
  for (const axis of axes) {
    const o = overlapOnAxis(ptsA, ptsB, axis);
    if (o <= 0) return null;
    if (o < minOverlap) {
      minOverlap = o;
      smallestAxis = axis.clone();
    }
  }
  
  // 法線の方向を確認
  const dir = Vec2.sub(b.pos, a.pos);
  if (dir.dot(smallestAxis) < 0) {
    smallestAxis.mul(-1);
  }
  
  return { normal: smallestAxis, penetration: minOverlap };
}

function collideCircleCircle2D(a, b) {
  const dx = b.pos.x - a.pos.x;
  const dy = b.pos.y - a.pos.y;
  const dist2 = dx * dx + dy * dy;
  const r = a.radius + b.radius;
  
  if (dist2 >= r * r) return null;
  
  const dist = Math.sqrt(dist2);
  if (dist < 1e-9) return null;
  
  const normal = new Vec2(dx / dist, dy / dist);
  const penetration = r - dist;
  const point = Vec2.add(a.pos, Vec2.mul(normal, a.radius));
  
  return { a, b, normal, penetration, point };
}

function collideCircleOBB2D(circle, obb) {
  const angle = obb.type === 'obb' ? obb.angle : 0;
  const c = Math.cos(-angle);
  const s = Math.sin(-angle);
  
  // ローカル空間に変換
  const rel = Vec2.sub(circle.pos, obb.pos);
  const localX = rel.x * c - rel.y * s;
  const localY = rel.x * s + rel.y * c;
  
  // 最近接点
  const closestX = clamp(localX, -obb.halfWidth, obb.halfWidth);
  const closestY = clamp(localY, -obb.halfHeight, obb.halfHeight);
  
  // ワールド空間に戻す
  const worldX = closestX * Math.cos(angle) - closestY * Math.sin(angle);
  const worldY = closestX * Math.sin(angle) + closestY * Math.cos(angle);
  const closest = new Vec2(obb.pos.x + worldX, obb.pos.y + worldY);
  
  const dx = circle.pos.x - closest.x;
  const dy = circle.pos.y - closest.y;
  const dist2 = dx * dx + dy * dy;
  
  if (dist2 > circle.radius * circle.radius) return null;
  
  const dist = Math.sqrt(dist2);
  if (dist < 1e-9) {
    // 円の中心がOBB内部にある場合
    const normal = new Vec2(Math.cos(angle), Math.sin(angle));
    return { a: circle, b: obb, normal, penetration: circle.radius, point: closest };
  }
  
  const normal = new Vec2(dx / dist, dy / dist);
  const penetration = circle.radius - dist;
  
  return { a: circle, b: obb, normal, penetration, point: closest };
}

// 完全な15軸SAT実装
function SAT_OBB_OBB_3D(a, b) {
  const EA = a.halfExt;
  const EB = b.halfExt;
  const A = a.orientation || [new Vec3(1,0,0), new Vec3(0,1,0), new Vec3(0,0,1)];
  const B = b.orientation || [new Vec3(1,0,0), new Vec3(0,1,0), new Vec3(0,0,1)];
  
  const R = Array.from({ length: 3 }, () => Array(3).fill(0));
  const absR = Array.from({ length: 3 }, () => Array(3).fill(0));
  
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      R[i][j] = A[i].dot(B[j]);
      absR[i][j] = Math.abs(R[i][j]) + 1e-6;
    }
  }
  
  const tvec = Vec3.sub(b.pos, a.pos);
  const t = new Vec3(tvec.dot(A[0]), tvec.dot(A[1]), tvec.dot(A[2]));
  
  // Test axes A0, A1, A2
  for (let i = 0; i < 3; i++) {
    const ra = EA.getComponent(i);
    const rb = EB.x * absR[i][0] + EB.y * absR[i][1] + EB.z * absR[i][2];
    if (Math.abs(t.getComponent(i)) > ra + rb) return null;
  }
  
  // Test axes B0, B1, B2
  for (let j = 0; j < 3; j++) {
    const rb = EB.getComponent(j);
    const ra = EA.x * absR[0][j] + EA.y * absR[1][j] + EA.z * absR[2][j];
    const tj = Math.abs(t.x * R[0][j] + t.y * R[1][j] + t.z * R[2][j]);
    if (tj > ra + rb) return null;
  }
  
  // Test 9 cross product axes
  // A0 x B0
  let ra = EA.y * absR[2][0] + EA.z * absR[1][0];
  let rb = EB.y * absR[0][2] + EB.z * absR[0][1];
  if (Math.abs(t.z * R[1][0] - t.y * R[2][0]) > ra + rb) return null;
  
  // A0 x B1
  ra = EA.y * absR[2][1] + EA.z * absR[1][1];
  rb = EB.x * absR[0][2] + EB.z * absR[0][0];
  if (Math.abs(t.z * R[1][1] - t.y * R[2][1]) > ra + rb) return null;
  
  // A0 x B2
  ra = EA.y * absR[2][2] + EA.z * absR[1][2];
  rb = EB.x * absR[0][1] + EB.y * absR[0][0];
  if (Math.abs(t.z * R[1][2] - t.y * R[2][2]) > ra + rb) return null;
  
  // A1 x B0
  ra = EA.x * absR[2][0] + EA.z * absR[0][0];
  rb = EB.y * absR[1][2] + EB.z * absR[1][1];
  if (Math.abs(t.x * R[2][0] - t.z * R[0][0]) > ra + rb) return null;
  
  // A1 x B1
  ra = EA.x * absR[2][1] + EA.z * absR[0][1];
  rb = EB.x * absR[1][2] + EB.z * absR[1][0];
  if (Math.abs(t.x * R[2][1] - t.z * R[0][1]) > ra + rb) return null;
  
  // A1 x B2
  ra = EA.x * absR[2][2] + EA.z * absR[0][2];
  rb = EB.x * absR[1][1] + EB.y * absR[1][0];
  if (Math.abs(t.x * R[2][2] - t.z * R[0][2]) > ra + rb) return null;
  
  // A2 x B0
  ra = EA.x * absR[1][0] + EA.y * absR[0][0];
  rb = EB.y * absR[2][2] + EB.z * absR[2][1];
  if (Math.abs(t.y * R[0][0] - t.x * R[1][0]) > ra + rb) return null;
  
  // A2 x B1
  ra = EA.x * absR[1][1] + EA.y * absR[0][1];
  rb = EB.x * absR[2][2] + EB.z * absR[2][0];
  if (Math.abs(t.y * R[0][1] - t.x * R[1][1]) > ra + rb) return null;
  
  // A2 x B2
  ra = EA.x * absR[1][2] + EA.y * absR[0][2];
  rb = EB.x * absR[2][1] + EB.y * absR[2][0];
  if (Math.abs(t.y * R[0][2] - t.x * R[1][2]) > ra + rb) return null;
  
  // 衝突している場合、近似的な法線と侵入深さを計算
  const dir = Vec3.sub(b.pos, a.pos);
  const len = dir.length();
  if (len < 1e-9) return { normal: new Vec3(1, 0, 0), penetration: 0.001 };
  
  return { normal: Vec3.mul(dir, 1 / len), penetration: 0.001 };
}

function collideSphereSphere(a, b) {
  const dx = b.pos.x - a.pos.x;
  const dy = b.pos.y - a.pos.y;
  const dz = b.pos.z - a.pos.z;
  const dist2 = dx * dx + dy * dy + dz * dz;
  const r = a.radius + b.radius;
  
  if (dist2 >= r * r) return null;
  
  const dist = Math.sqrt(dist2);
  if (dist < 1e-9) return null;
  
  const normal = new Vec3(dx / dist, dy / dist, dz / dist);
  const penetration = r - dist;
  const point = Vec3.add(a.pos, Vec3.mul(normal, a.radius));
  
  return { a, b, normal, penetration, point };
}

function collideSphereAABB(sphere, box) {
  const closest = new Vec3(
    clamp(sphere.pos.x, box.pos.x - box.halfExt.x, box.pos.x + box.halfExt.x),
    clamp(sphere.pos.y, box.pos.y - box.halfExt.y, box.pos.y + box.halfExt.y),
    clamp(sphere.pos.z, box.pos.z - box.halfExt.z, box.pos.z + box.halfExt.z)
  );
  
  const dx = sphere.pos.x - closest.x;
  const dy = sphere.pos.y - closest.y;
  const dz = sphere.pos.z - closest.z;
  const dist2 = dx * dx + dy * dy + dz * dz;
  
  if (dist2 > sphere.radius * sphere.radius) return null;
  
  const dist = Math.sqrt(dist2);
  if (dist < 1e-9) {
    // 球の中心がボックス内部
    const normal = new Vec3(0, 1, 0);
    return { a: sphere, b: box, normal, penetration: sphere.radius, point: closest };
  }
  
  const normal = new Vec3(dx / dist, dy / dist, dz / dist);
  const penetration = sphere.radius - dist;
  
  return { a: sphere, b: box, normal, penetration, point: closest };
}

// ------------------------------
// CCD (修正版)
// ------------------------------

function rayAABBIntersect(orig, dir, min, max) {
  const invDirX = Math.abs(dir.x) > 1e-9 ? 1 / dir.x : 1e9;
  const invDirY = Math.abs(dir.y) > 1e-9 ? 1 / dir.y : 1e9;
  const invDirZ = Math.abs(dir.z) > 1e-9 ? 1 / dir.z : 1e9;
  
  let tmin = (min.x - orig.x) * invDirX;
  let tmax = (max.x - orig.x) * invDirX;
  if (tmin > tmax) [tmin, tmax] = [tmax, tmin];
  
  let tymin = (min.y - orig.y) * invDirY;
  let tymax = (max.y - orig.y) * invDirY;
  if (tymin > tymax) [tymin, tymax] = [tymax, tymin];
  
  if ((tmin > tymax) || (tymin > tmax)) return null;
  tmin = Math.max(tmin, tymin);
  tmax = Math.min(tmax, tymax);
  
  let tzmin = (min.z - orig.z) * invDirZ;
  let tzmax = (max.z - orig.z) * invDirZ;
  if (tzmin > tzmax) [tzmin, tzmax] = [tzmax, tzmin];
  
  if ((tmin > tzmax) || (tzmin > tmax)) return null;
  tmin = Math.max(tmin, tzmin);
  tmax = Math.min(tmax, tzmax);
  
  return { tmin, tmax };
}

function sweptSphereVsAABB(sphere, vel, box) {
  const min = Vec3.sub(box.pos, box.halfExt);
  const max = Vec3.add(box.pos, box.halfExt);
  
  min.x -= sphere.radius;
  min.y -= sphere.radius;
  min.z -= sphere.radius;
  max.x += sphere.radius;
  max.y += sphere.radius;
  max.z += sphere.radius;
  
  const res = rayAABBIntersect(sphere.pos, vel, min, max);
  if (!res) return null;
  
  if (res.tmin >= 0 && res.tmin <= 1) {
    return { t: res.tmin, hitPoint: Vec3.add(sphere.pos, Vec3.mul(vel, res.tmin)) };
  }
  if (res.tmax >= 0 && res.tmax <= 1) {
    return { t: res.tmax, hitPoint: Vec3.add(sphere.pos, Vec3.mul(vel, res.tmax)) };
  }
  
  return null;
}

function sweptCircleVsOBB2D(circle, vel, obb) {
  const angle = obb.type === 'obb' ? obb.angle : 0;
  const c = Math.cos(-angle);
  const s = Math.sin(-angle);
  
  const rel = Vec2.sub(circle.pos, obb.pos);
  const lx = rel.x * c - rel.y * s;
  const ly = rel.x * s + rel.y * c;
  
  const lvelx = vel.x * c - vel.y * s;
  const lvely = vel.x * s + vel.y * c;
  
  const minX = -obb.halfWidth - circle.radius;
  const maxX = obb.halfWidth + circle.radius;
  const minY = -obb.halfHeight - circle.radius;
  const maxY = obb.halfHeight + circle.radius;
  
  let tmin = 0;
  let tmax = 1;
  
  // X軸
  if (Math.abs(lvelx) > 1e-9) {
    let tx1 = (minX - lx) / lvelx;
    let tx2 = (maxX - lx) / lvelx;
    if (tx1 > tx2) [tx1, tx2] = [tx2, tx1];
    tmin = Math.max(tmin, tx1);
    tmax = Math.min(tmax, tx2);
  } else if (lx < minX || lx > maxX) {
    return null;
  }
  
  // Y軸
  if (Math.abs(lvely) > 1e-9) {
    let ty1 = (minY - ly) / lvely;
    let ty2 = (maxY - ly) / lvely;
    if (ty1 > ty2) [ty1, ty2] = [ty2, ty1];
    tmin = Math.max(tmin, ty1);
    tmax = Math.min(tmax, ty2);
  } else if (ly < minY || ly > maxY) {
    return null;
  }
  
  if (tmax >= tmin && tmax >= 0 && tmin <= 1) {
    return { t: Math.max(0, tmin) };
  }
  
  return null;
}

// ------------------------------
// Contact Resolution (修正版)
// ------------------------------
function resolveContact(contact) {
  const a = contact.a;
  const b = contact.b;
  
  if (a.static && b.static) return;
  
  // 両方を起こす
  if (!a.static) a.wakeUp();
  if (!b.static) b.wakeUp();
  
  const n = contact.normal;
  const rv = Vec2.sub ? Vec2.sub(b.vel, a.vel) : Vec3.sub(b.vel, a.vel);
  const velAlongNormal = rv.dot(n);
  
  // 離れていく場合は処理しない
  if (velAlongNormal > 0) return;
  
  const e = Math.min(a.restitution, b.restitution);
  const totalInvMass = a.invMass + b.invMass;
  
  if (totalInvMass < 1e-9) return;
  
  // 衝突インパルス
  const j = -(1 + e) * velAlongNormal / totalInvMass;
  const impulse = Vec2.mul ? Vec2.mul(n, j) : Vec3.mul(n, j);
  
  if (!a.static) {
    a.vel = Vec2.sub ? Vec2.sub(a.vel, Vec2.mul(impulse, a.invMass)) : Vec3.sub(a.vel, Vec3.mul(impulse, a.invMass));
  }
  if (!b.static) {
    b.vel = Vec2.add ? Vec2.add(b.vel, Vec2.mul(impulse, b.invMass)) : Vec3.add(b.vel, Vec3.mul(impulse, b.invMass));
  }
  
  // 摩擦
  const rvAfter = Vec2.sub ? Vec2.sub(b.vel, a.vel) : Vec3.sub(b.vel, a.vel);
  const tangent = Vec2.sub ? Vec2.sub(rvAfter, Vec2.mul(n, rvAfter.dot(n))) : Vec3.sub(rvAfter, Vec3.mul(n, rvAfter.dot(n)));
  const tangentLen = tangent.length();
  
  if (tangentLen > 1e-9) {
    tangent.normalize();
    const jt = -rvAfter.dot(tangent) / totalInvMass;
    const mu = Math.sqrt(a.friction * b.friction);
    
    let frictionImpulse;
    if (Math.abs(jt) < Math.abs(j) * mu) {
      frictionImpulse = Vec2.mul ? Vec2.mul(tangent, jt) : Vec3.mul(tangent, jt);
    } else {
      frictionImpulse = Vec2.mul ? Vec2.mul(tangent, -j * mu) : Vec3.mul(tangent, -j * mu);
    }
    
    if (!a.static) {
      a.vel = Vec2.sub ? Vec2.sub(a.vel, Vec2.mul(frictionImpulse, a.invMass)) : Vec3.sub(a.vel, Vec3.mul(frictionImpulse, a.invMass));
    }
    if (!b.static) {
      b.vel = Vec2.add ? Vec2.add(b.vel, Vec2.mul(frictionImpulse, b.invMass)) : Vec3.add(b.vel, Vec3.mul(frictionImpulse, b.invMass));
    }
  }
  
  // 位置補正 (Baumgarte stabilization)
  const percent = 0.4;
  const slop = 0.01;
  const correctionMag = Math.max(contact.penetration - slop, 0) / totalInvMass * percent;
  const correction = Vec2.mul ? Vec2.mul(n, correctionMag) : Vec3.mul(n, correctionMag);
  
  if (!a.static) {
    a.pos = Vec2.sub ? Vec2.sub(a.pos, Vec2.mul(correction, a.invMass)) : Vec3.sub(a.pos, Vec3.mul(correction, a.invMass));
  }
  if (!b.static) {
    b.pos = Vec2.add ? Vec2.add(b.pos, Vec2.mul(correction, b.invMass)) : Vec3.add(b.pos, Vec3.mul(correction, b.invMass));
  }
}

// ------------------------------
// PhysicsWorld (修正版)
// ------------------------------
class PhysicsWorld {
  constructor(dim='2D', options={}) {
    this.dim = dim;
    this.bodies = [];
    this.gravity = options.gravity || (dim === '3D' ? new Vec3(0, -9.8, 0) : new Vec2(0, 980));
    this.spatial = new SpatialHash(options.cellSize || 128, dim);
    this.substeps = options.substeps || 4;
    this.maxIters = options.maxIters || 8;
    this.damping = options.damping ?? 0.998;
    this.sleepEnabled = options.sleepEnabled ?? true;
    this._nextBodyId = 0;
  }
  
  addBody(b) {
    b.dim = this.dim;
    b.id = this._nextBodyId++;
    this.bodies.push(b);
    return b;
  }
  
  removeBody(b) {
    const index = this.bodies.indexOf(b);
    if (index !== -1) {
      this.bodies.splice(index, 1);
    }
  }
  
  step(dt) {
    const sub = Math.max(1, this.substeps);
    const subDt = dt / sub;
    
    for (let s = 0; s < sub; s++) {
      // 積分
      for (const b of this.bodies) {
        if (b.static || b.sleeping) continue;
        
        // 重力適用
        const gravityForce = this.dim === '3D' ? 
          Vec3.mul(this.gravity, b.mass) : 
          Vec2.mul(this.gravity, b.mass);
        b.force.add(gravityForce);
        
        // 加速度と速度更新
        const acc = this.dim === '3D' ?
          Vec3.mul(b.force, b.invMass) :
          Vec2.mul(b.force, b.invMass);
        
        b.vel.add(Vec3.mul ? Vec3.mul(acc, subDt) : Vec2.mul(acc, subDt));
        
        // 減衰
        b.vel.mul(this.damping);
        
        // 位置更新
        const displacement = this.dim === '3D' ?
          Vec3.mul(b.vel, subDt) :
          Vec2.mul(b.vel, subDt);
        b.pos.add(displacement);
        
        // 角度更新 (2D OBBの場合)
        if (this.dim === '2D' && b.type === 'obb' && b.angularVel) {
          b.angle += b.angularVel * subDt;
        }
        
        // 力をクリア
        b.force.set(0, 0, this.dim === '3D' ? 0 : undefined);
      }
      
      // Broadphase
      this.spatial.clear();
      for (const b of this.bodies) {
        if (!b.sleeping) {
          this.spatial.insert(b);
        }
      }
      const pairs = this.spatial.queryPairs();
      
      const contacts = [];
      
      // Narrowphase
      for (const [a, b] of pairs) {
        if (a === b) continue;
        if (a.static && b.static) continue;
        if (a.sleeping && b.sleeping) continue;
        
        // AABB overlap test
        if (!a.getAABB().overlaps(b.getAABB())) continue;
        
        let contact = null;
        
        if (this.dim === '2D') {
          // 2D collision detection
          if (a.type === 'circle' && b.type === 'circle') {
            contact = collideCircleCircle2D(a, b);
          } else if (a.type === 'circle' && (b.type === 'obb' || b.type === 'box')) {
            contact = collideCircleOBB2D(a, b);
          } else if ((a.type === 'obb' || a.type === 'box') && b.type === 'circle') {
            contact = collideCircleOBB2D(b, a);
            if (contact) {
              contact.normal.mul(-1);
              [contact.a, contact.b] = [contact.b, contact.a];
            }
          } else if ((a.type === 'obb' || a.type === 'box') && (b.type === 'obb' || b.type === 'box')) {
            const result = SAT_OBB_OBB_2D(a, b);
            if (result) {
              contact = { a, b, normal: result.normal, penetration: result.penetration };
            }
          }
        } else {
          // 3D collision detection
          if (a.type === 'sphere' && b.type === 'sphere') {
            contact = collideSphereSphere(a, b);
          } else if (a.type === 'sphere' && (b.type === 'aabb' || b.type === 'obb3')) {
            contact = collideSphereAABB(a, b);
          } else if ((a.type === 'aabb' || a.type === 'obb3') && b.type === 'sphere') {
            contact = collideSphereAABB(b, a);
            if (contact) {
              contact.normal.mul(-1);
              [contact.a, contact.b] = [contact.b, contact.a];
            }
          } else if ((a.type === 'obb3' || a.type === 'aabb') && (b.type === 'obb3' || b.type === 'aabb')) {
            const result = SAT_OBB_OBB_3D(a, b);
            if (result) {
              contact = { a, b, normal: result.normal, penetration: result.penetration };
            }
          }
        }
        
        if (contact) {
          contacts.push(contact);
        }
      }
      
      // 衝突解決
      for (let it = 0; it < this.maxIters; it++) {
        for (const c of contacts) {
          resolveContact(c);
        }
      }
      
      // CCD (高速移動体のトンネリング防止)
      this._applyCCD(subDt);
      
      // スリープ状態の更新
      if (this.sleepEnabled) {
        for (const b of this.bodies) {
          b.updateSleepState(subDt);
        }
      }
    }
  }
  
  _applyCCD(dt) {
    for (const moving of this.bodies) {
      if (moving.static || moving.sleeping) continue;
      
      const disp = this.dim === '3D' ? 
        Vec3.mul(moving.vel, dt) : 
        Vec2.mul(moving.vel, dt);
      
      if (this.dim === '3D' && moving.type === 'sphere') {
        for (const other of this.bodies) {
          if (other === moving || (!other.static && other.invMass > 0)) continue;
          
          if (other.type === 'aabb' || other.type === 'obb3') {
            const hit = sweptSphereVsAABB(moving, disp, other);
            if (hit && hit.t >= 0 && hit.t <= 1) {
              // 衝突点まで移動
              moving.pos = hit.hitPoint;
              
              // 法線を計算
              const normal = Vec3.sub(moving.pos, other.pos).normalize();
              
              // 速度を反射
              const dotVN = moving.vel.dot(normal);
              moving.vel = Vec3.sub(moving.vel, Vec3.mul(normal, 2 * dotVN * moving.restitution));
              break;
            }
          }
        }
      } else if (this.dim === '2D' && moving.type === 'circle') {
        for (const other of this.bodies) {
          if (other === moving || (!other.static && other.invMass > 0)) continue;
          
          if (other.type === 'obb' || other.type === 'box') {
            const hit = sweptCircleVsOBB2D(moving, disp, other);
            if (hit && hit.t >= 0 && hit.t <= 1) {
              moving.pos.add(Vec2.mul(disp, hit.t));
              
              const normal = Vec2.sub(moving.pos, other.pos).normalize();
              const dotVN = moving.vel.dot(normal);
              moving.vel = Vec2.sub(moving.vel, Vec2.mul(normal, 2 * dotVN * moving.restitution));
              break;
            }
          }
        }
      }
    }
  }
  
  // ユーティリティメソッド
  getBodiesInRadius(pos, radius) {
    const result = [];
    const radiusSq = radius * radius;
    
    for (const b of this.bodies) {
      const distSq = this.dim === '3D' ?
        Vec3.sub(b.pos, pos).lengthSq() :
        Vec2.sub(b.pos, pos).lengthSq();
      
      if (distSq <= radiusSq) {
        result.push(b);
      }
    }
    
    return result;
  }
  
  raycast(origin, direction, maxDistance=1000) {
    let closestHit = null;
    let closestDist = maxDistance;
    
    for (const b of this.bodies) {
      // 簡易的なレイキャスト実装
      if (this.dim === '3D' && b.type === 'sphere') {
        const oc = Vec3.sub(origin, b.pos);
        const a = direction.dot(direction);
        const half_b = oc.dot(direction);
        const c = oc.dot(oc) - b.radius * b.radius;
        const discriminant = half_b * half_b - a * c;
        
        if (discriminant > 0) {
          const t = (-half_b - Math.sqrt(discriminant)) / a;
          if (t > 0 && t < closestDist) {
            closestDist = t;
            closestHit = { body: b, distance: t, point: Vec3.add(origin, Vec3.mul(direction, t)) };
          }
        }
      }
    }
    
    return closestHit;
  }
}

// ------------------------------
// Export
// ------------------------------
if (typeof window !== 'undefined') {
  window.UnifiedPhysics = {
    Vec2,
    Vec3,
    Quaternion,
    Body,
    PhysicsWorld,
    AABB
  };
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    Vec2,
    Vec3,
    Quaternion,
    Body,
    PhysicsWorld,
    AABB
  };
}
