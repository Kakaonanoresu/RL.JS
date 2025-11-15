/* UnifiedPhysicsEngine.js - 完全リビルド版
   
   完全に見直した2D/3D物理エンジン
   - 正確な衝突検出と応答
   - 安定した積分
   - 適切な力の適用
   - デバッグ済みのSAT実装
*/

// ================================
// ベクトル演算 (Immutable)
// ================================
class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  clone() {
    return new Vec2(this.x, this.y);
  }

  add(v) {
    return new Vec2(this.x + v.x, this.y + v.y);
  }

  sub(v) {
    return new Vec2(this.x - v.x, this.y - v.y);
  }

  mul(s) {
    return new Vec2(this.x * s, this.y * s);
  }

  dot(v) {
    return this.x * v.x + this.y * v.y;
  }

  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }

  lengthSq() {
    return this.x * this.x + this.y * this.y;
  }

  normalize() {
    const len = this.length();
    if (len < 1e-8) return new Vec2(0, 0);
    return this.mul(1 / len);
  }

  perpendicular() {
    return new Vec2(-this.y, this.x);
  }

  rotate(angle) {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return new Vec2(
      this.x * c - this.y * s,
      this.x * s + this.y * c
    );
  }
}

class Vec3 {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  clone() {
    return new Vec3(this.x, this.y, this.z);
  }

  add(v) {
    return new Vec3(this.x + v.x, this.y + v.y, this.z + v.z);
  }

  sub(v) {
    return new Vec3(this.x - v.x, this.y - v.y, this.z - v.z);
  }

  mul(s) {
    return new Vec3(this.x * s, this.y * s, this.z * s);
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
    const len = this.length();
    if (len < 1e-8) return new Vec3(0, 0, 0);
    return this.mul(1 / len);
  }
}

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

// ================================
// AABB (Axis-Aligned Bounding Box)
// ================================
class AABB {
  constructor(min, max) {
    this.min = min;
    this.max = max;
  }

  overlaps(other) {
    const a = this;
    const b = other;
    
    if (a.min instanceof Vec2) {
      return !(a.max.x < b.min.x || a.min.x > b.max.x ||
               a.max.y < b.min.y || a.min.y > b.max.y);
    } else {
      return !(a.max.x < b.min.x || a.min.x > b.max.x ||
               a.max.y < b.min.y || a.min.y > b.max.y ||
               a.max.z < b.min.z || a.min.z > b.max.z);
    }
  }

  expand(amount) {
    if (this.min instanceof Vec2) {
      return new AABB(
        this.min.sub(new Vec2(amount, amount)),
        this.max.add(new Vec2(amount, amount))
      );
    } else {
      return new AABB(
        this.min.sub(new Vec3(amount, amount, amount)),
        this.max.add(new Vec3(amount, amount, amount))
      );
    }
  }
}

// ================================
// Body (剛体)
// ================================
class Body {
  constructor(options = {}) {
    this.id = Body._nextId++;
    this.dim = options.dim || '2D';
    this.type = options.type || 'circle';
    
    // 位置と速度
    if (this.dim === '2D') {
      this.pos = options.pos ? options.pos.clone() : new Vec2();
      this.vel = options.vel ? options.vel.clone() : new Vec2();
      this.acc = new Vec2();
    } else {
      this.pos = options.pos ? options.pos.clone() : new Vec3();
      this.vel = options.vel ? options.vel.clone() : new Vec3();
      this.acc = new Vec3();
    }

    // 質量
    this.mass = options.mass !== undefined ? options.mass : 1;
    this.invMass = this.mass > 0 ? 1 / this.mass : 0;
    this.static = options.static || false;
    if (this.static) this.invMass = 0;

    // 材質
    this.restitution = options.restitution !== undefined ? options.restitution : 0.5;
    this.friction = options.friction !== undefined ? options.friction : 0.5;

    // 形状
    if (this.dim === '2D') {
      if (this.type === 'circle') {
        this.radius = options.radius || 20;
      } else if (this.type === 'box' || this.type === 'obb') {
        this.width = options.width || 40;
        this.height = options.height || 40;
        this.angle = options.angle || 0;
        this.angularVel = 0;
      }
    } else {
      if (this.type === 'sphere') {
        this.radius = options.radius || 20;
      } else if (this.type === 'box') {
        this.width = options.width || 40;
        this.height = options.height || 40;
        this.depth = options.depth || 40;
      }
    }

    // スリープ
    this.sleeping = false;
    this.sleepTime = 0;
  }

  getAABB() {
    if (this.dim === '2D') {
      if (this.type === 'circle') {
        return new AABB(
          this.pos.sub(new Vec2(this.radius, this.radius)),
          this.pos.add(new Vec2(this.radius, this.radius))
        );
      } else {
        // OBBの場合は保守的なAABBを返す
        const hw = this.width / 2;
        const hh = this.height / 2;
        const r = Math.sqrt(hw * hw + hh * hh);
        return new AABB(
          this.pos.sub(new Vec2(r, r)),
          this.pos.add(new Vec2(r, r))
        );
      }
    } else {
      if (this.type === 'sphere') {
        const r = new Vec3(this.radius, this.radius, this.radius);
        return new AABB(this.pos.sub(r), this.pos.add(r));
      } else {
        const hw = this.width / 2;
        const hh = this.height / 2;
        const hd = this.depth / 2;
        return new AABB(
          this.pos.sub(new Vec3(hw, hh, hd)),
          this.pos.add(new Vec3(hw, hh, hd))
        );
      }
    }
  }

  applyForce(force) {
    if (this.static) return;
    this.acc = this.acc.add(force.mul(this.invMass));
  }

  integrate(dt) {
    if (this.static) return;

    // Verlet integration
    this.vel = this.vel.add(this.acc.mul(dt));
    this.pos = this.pos.add(this.vel.mul(dt));

    // 角度の更新 (2D OBBの場合)
    if (this.dim === '2D' && (this.type === 'box' || this.type === 'obb')) {
      this.angle += this.angularVel * dt;
    }

    // 加速度をリセット
    if (this.dim === '2D') {
      this.acc = new Vec2();
    } else {
      this.acc = new Vec3();
    }
  }
}

Body._nextId = 0;

// ================================
// Spatial Hash (Broadphase)
// ================================
class SpatialHash {
  constructor(cellSize = 100) {
    this.cellSize = cellSize;
    this.cells = new Map();
  }

  clear() {
    this.cells.clear();
  }

  _hash(x, y, z = 0) {
    const ix = Math.floor(x / this.cellSize);
    const iy = Math.floor(y / this.cellSize);
    const iz = Math.floor(z / this.cellSize);
    return `${ix},${iy},${iz}`;
  }

  insert(body) {
    const aabb = body.getAABB();
    const minHash = this._hash(aabb.min.x, aabb.min.y, aabb.min.z || 0);
    const maxHash = this._hash(aabb.max.x, aabb.max.y, aabb.max.z || 0);

    const [minX, minY, minZ] = minHash.split(',').map(Number);
    const [maxX, maxY, maxZ] = maxHash.split(',').map(Number);

    for (let x = minX; x <= maxX; x++) {
      for (let y = minY; y <= maxY; y++) {
        for (let z = minZ; z <= maxZ; z++) {
          const key = `${x},${y},${z}`;
          if (!this.cells.has(key)) {
            this.cells.set(key, []);
          }
          this.cells.get(key).push(body);
        }
      }
    }
  }

  getPotentialPairs() {
    const pairs = [];
    const tested = new Set();

    for (const bodies of this.cells.values()) {
      for (let i = 0; i < bodies.length; i++) {
        for (let j = i + 1; j < bodies.length; j++) {
          const a = bodies[i];
          const b = bodies[j];
          const key = a.id < b.id ? `${a.id}-${b.id}` : `${b.id}-${a.id}`;
          
          if (!tested.has(key)) {
            tested.add(key);
            pairs.push([a, b]);
          }
        }
      }
    }

    return pairs;
  }
}

// ================================
// Collision Detection (Narrowphase)
// ================================

// 2D: Circle vs Circle
function collideCircles(a, b) {
  const delta = b.pos.sub(a.pos);
  const distSq = delta.lengthSq();
  const radiusSum = a.radius + b.radius;

  if (distSq >= radiusSum * radiusSum) {
    return null;
  }

  const dist = Math.sqrt(distSq);
  const normal = dist > 1e-8 ? delta.mul(1 / dist) : new Vec2(1, 0);
  const depth = radiusSum - dist;

  return {
    a, b,
    normal,
    depth,
    point: a.pos.add(normal.mul(a.radius))
  };
}

// 2D: Circle vs OBB
function collideCircleBox(circle, box) {
  // ボックスのローカル空間に円を変換
  const delta = circle.pos.sub(box.pos);
  const localPos = delta.rotate(-box.angle);

  // 最近接点を計算
  const hw = box.width / 2;
  const hh = box.height / 2;
  const closestLocal = new Vec2(
    clamp(localPos.x, -hw, hw),
    clamp(localPos.y, -hh, hh)
  );

  // ワールド空間に戻す
  const closestWorld = closestLocal.rotate(box.angle).add(box.pos);
  const delta2 = circle.pos.sub(closestWorld);
  const distSq = delta2.lengthSq();

  if (distSq >= circle.radius * circle.radius) {
    return null;
  }

  const dist = Math.sqrt(distSq);
  const normal = dist > 1e-8 ? delta2.mul(1 / dist) : new Vec2(0, -1);
  const depth = circle.radius - dist;

  return {
    a: circle,
    b: box,
    normal,
    depth,
    point: closestWorld
  };
}

// 2D: OBB vs OBB (SAT)
function collideBoxes(a, b) {
  const axes = [];
  
  // Aの軸
  axes.push(new Vec2(Math.cos(a.angle), Math.sin(a.angle)));
  axes.push(new Vec2(-Math.sin(a.angle), Math.cos(a.angle)));
  
  // Bの軸
  axes.push(new Vec2(Math.cos(b.angle), Math.sin(b.angle)));
  axes.push(new Vec2(-Math.sin(b.angle), Math.cos(b.angle)));

  let minOverlap = Infinity;
  let minAxis = null;

  for (const axis of axes) {
    const [minA, maxA] = projectBox(a, axis);
    const [minB, maxB] = projectBox(b, axis);

    const overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
    
    if (overlap < 0) {
      return null; // 分離軸が見つかった
    }

    if (overlap < minOverlap) {
      minOverlap = overlap;
      minAxis = axis;
    }
  }

  // 法線の方向を調整
  const delta = b.pos.sub(a.pos);
  if (delta.dot(minAxis) < 0) {
    minAxis = minAxis.mul(-1);
  }

  return {
    a, b,
    normal: minAxis,
    depth: minOverlap,
    point: a.pos.add(minAxis.mul(minOverlap / 2))
  };
}

function projectBox(box, axis) {
  const hw = box.width / 2;
  const hh = box.height / 2;
  
  const corners = [
    new Vec2(-hw, -hh),
    new Vec2(hw, -hh),
    new Vec2(hw, hh),
    new Vec2(-hw, hh)
  ];

  let min = Infinity;
  let max = -Infinity;

  for (const corner of corners) {
    const worldCorner = corner.rotate(box.angle).add(box.pos);
    const projection = worldCorner.dot(axis);
    min = Math.min(min, projection);
    max = Math.max(max, projection);
  }

  return [min, max];
}

// 3D: Sphere vs Sphere
function collideSpheres(a, b) {
  const delta = b.pos.sub(a.pos);
  const distSq = delta.lengthSq();
  const radiusSum = a.radius + b.radius;

  if (distSq >= radiusSum * radiusSum) {
    return null;
  }

  const dist = Math.sqrt(distSq);
  const normal = dist > 1e-8 ? delta.mul(1 / dist) : new Vec3(0, 1, 0);
  const depth = radiusSum - dist;

  return {
    a, b,
    normal,
    depth,
    point: a.pos.add(normal.mul(a.radius))
  };
}

// 3D: Sphere vs Box
function collideSphereBox(sphere, box) {
  const hw = box.width / 2;
  const hh = box.height / 2;
  const hd = box.depth / 2;

  const closest = new Vec3(
    clamp(sphere.pos.x, box.pos.x - hw, box.pos.x + hw),
    clamp(sphere.pos.y, box.pos.y - hh, box.pos.y + hh),
    clamp(sphere.pos.z, box.pos.z - hd, box.pos.z + hd)
  );

  const delta = sphere.pos.sub(closest);
  const distSq = delta.lengthSq();

  if (distSq >= sphere.radius * sphere.radius) {
    return null;
  }

  const dist = Math.sqrt(distSq);
  const normal = dist > 1e-8 ? delta.mul(1 / dist) : new Vec3(0, 1, 0);
  const depth = sphere.radius - dist;

  return {
    a: sphere,
    b: box,
    normal,
    depth,
    point: closest
  };
}

// 3D: Box vs Box (SAT - 簡易版)
function collideBoxes3D(a, b) {
  const aabb1 = a.getAABB();
  const aabb2 = b.getAABB();

  if (!aabb1.overlaps(aabb2)) {
    return null;
  }

  // 各軸での侵入深さを計算
  const overlapX = Math.min(aabb1.max.x, aabb2.max.x) - Math.max(aabb1.min.x, aabb2.min.x);
  const overlapY = Math.min(aabb1.max.y, aabb2.max.y) - Math.max(aabb1.min.y, aabb2.min.y);
  const overlapZ = Math.min(aabb1.max.z, aabb2.max.z) - Math.max(aabb1.min.z, aabb2.min.z);

  // 最小侵入軸を選択
  let normal, depth;
  if (overlapX < overlapY && overlapX < overlapZ) {
    depth = overlapX;
    normal = a.pos.x < b.pos.x ? new Vec3(-1, 0, 0) : new Vec3(1, 0, 0);
  } else if (overlapY < overlapZ) {
    depth = overlapY;
    normal = a.pos.y < b.pos.y ? new Vec3(0, -1, 0) : new Vec3(0, 1, 0);
  } else {
    depth = overlapZ;
    normal = a.pos.z < b.pos.z ? new Vec3(0, 0, -1) : new Vec3(0, 0, 1);
  }

  return {
    a, b,
    normal,
    depth,
    point: a.pos.add(b.pos).mul(0.5)
  };
}

// ================================
// Collision Resolution
// ================================
function resolveCollision(contact) {
  const { a, b, normal, depth } = contact;

  if (a.static && b.static) return;

  const totalInvMass = a.invMass + b.invMass;
  if (totalInvMass < 1e-8) return;

  // 位置補正
  const correction = normal.mul(depth / totalInvMass * 0.8); // 80%補正
  if (!a.static) a.pos = a.pos.sub(correction.mul(a.invMass));
  if (!b.static) b.pos = b.pos.add(correction.mul(b.invMass));

  // 相対速度
  const relVel = b.vel.sub(a.vel);
  const velAlongNormal = relVel.dot(normal);

  // 離れていく場合は何もしない
  if (velAlongNormal > 0) return;

  // 反発係数
  const e = Math.min(a.restitution, b.restitution);

  // インパルスの大きさ
  const j = -(1 + e) * velAlongNormal / totalInvMass;
  const impulse = normal.mul(j);

  // 速度を更新
  if (!a.static) a.vel = a.vel.sub(impulse.mul(a.invMass));
  if (!b.static) b.vel = b.vel.add(impulse.mul(b.invMass));

  // 摩擦
  const tangent = relVel.sub(normal.mul(velAlongNormal)).normalize();
  const jt = -relVel.dot(tangent) / totalInvMass;
  const mu = Math.sqrt(a.friction * b.friction);

  let frictionImpulse;
  if (Math.abs(jt) < j * mu) {
    frictionImpulse = tangent.mul(jt);
  } else {
    frictionImpulse = tangent.mul(-j * mu);
  }

  if (!a.static) a.vel = a.vel.sub(frictionImpulse.mul(a.invMass));
  if (!b.static) b.vel = b.vel.add(frictionImpulse.mul(b.invMass));
}

// ================================
// Physics World
// ================================
class PhysicsWorld {
  constructor(dim = '2D', options = {}) {
    this.dim = dim;
    this.bodies = [];
    
    // 重力
    if (dim === '2D') {
      this.gravity = options.gravity || new Vec2(0, 500);
    } else {
      this.gravity = options.gravity || new Vec3(0, -500, 0);
    }

    // 設定
    this.iterations = options.iterations || 4;
    this.damping = options.damping !== undefined ? options.damping : 0.999;
    
    // Broadphase
    this.spatialHash = new SpatialHash(100);
  }

  addBody(body) {
    body.dim = this.dim;
    this.bodies.push(body);
    return body;
  }

  removeBody(body) {
    const index = this.bodies.indexOf(body);
    if (index !== -1) {
      this.bodies.splice(index, 1);
    }
  }

  step(dt) {
    // 力を適用
    for (const body of this.bodies) {
      if (!body.static) {
        body.applyForce(this.gravity.mul(body.mass));
      }
    }

    // 積分
    for (const body of this.bodies) {
      body.integrate(dt);
      // 減衰
      body.vel = body.vel.mul(this.damping);
    }

    // 衝突検出と解決
    for (let iter = 0; iter < this.iterations; iter++) {
      // Broadphase
      this.spatialHash.clear();
      for (const body of this.bodies) {
        this.spatialHash.insert(body);
      }

      const pairs = this.spatialHash.getPotentialPairs();

      // Narrowphase
      for (const [a, b] of pairs) {
        if (a.static && b.static) continue;

        let contact = null;

        if (this.dim === '2D') {
          // 2D衝突検出
          if (a.type === 'circle' && b.type === 'circle') {
            contact = collideCircles(a, b);
          } else if (a.type === 'circle' && (b.type === 'box' || b.type === 'obb')) {
            contact = collideCircleBox(a, b);
          } else if ((a.type === 'box' || a.type === 'obb') && b.type === 'circle') {
            contact = collideCircleBox(b, a);
            if (contact) {
              contact.normal = contact.normal.mul(-1);
              [contact.a, contact.b] = [contact.b, contact.a];
            }
          } else if ((a.type === 'box' || a.type === 'obb') && 
                     (b.type === 'box' || b.type === 'obb')) {
            contact = collideBoxes(a, b);
          }
        } else {
          // 3D衝突検出
          if (a.type === 'sphere' && b.type === 'sphere') {
            contact = collideSpheres(a, b);
          } else if (a.type === 'sphere' && b.type === 'box') {
            contact = collideSphereBox(a, b);
          } else if (a.type === 'box' && b.type === 'sphere') {
            contact = collideSphereBox(b, a);
            if (contact) {
              contact.normal = contact.normal.mul(-1);
              [contact.a, contact.b] = [contact.b, contact.a];
            }
          } else if (a.type === 'box' && b.type === 'box') {
            contact = collideBoxes3D(a, b);
          }
        }

        if (contact) {
          resolveCollision(contact);
        }
      }
    }
  }
}

// ================================
// Export
// ================================
if (typeof window !== 'undefined') {
  window.UnifiedPhysics = {
    Vec2,
    Vec3,
    Body,
    PhysicsWorld,
    AABB
  };
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    Vec2,
    Vec3,
    Body,
    PhysicsWorld,
    AABB
  };
}
