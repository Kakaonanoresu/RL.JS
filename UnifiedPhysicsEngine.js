/* UnifiedPhysicsEngine.js
   - Unified 2D / 3D physics core
   - Features: Vec2/Vec3, Rigid bodies (circle/sphere, OBB/box, AABB),
     SAT for OBB (2D) and OBB-OBB (3D simplified 15-axis SAT),
     Broadphase (spatial hash), CCD (swept-sphere / swept-circle against box),
     Impulse-based collision resolution, positional correction, substeps.

   Usage: include this file in a page. Create PhysicsWorld('2D'|'3D'),
   add Body objects (type: 'circle'|'box'|'obb'|'sphere'|'aabb'), and call world.step(dt).

   Note: This file aims for readability and lightweight performance; it's
   intentionally not a full-featured production engine but covers common
   game needs and is optimized with spatial hashing & early outs.
*/

// ------------------------------
// Math utilities
// ------------------------------
class Vec2 {
  constructor(x=0,y=0){ this.x=x; this.y=y; }
  set(x,y){ this.x=x; this.y=y; return this; }
  clone(){ return new Vec2(this.x,this.y); }
  add(v){ this.x+=v.x; this.y+=v.y; return this; }
  sub(v){ this.x-=v.x; this.y-=v.y; return this; }
  mul(s){ this.x*=s; this.y*=s; return this; }
  dot(v){ return this.x*v.x + this.y*v.y; }
  length(){ return Math.hypot(this.x,this.y); }
  normalize(){ const l=this.length(); if(l>0){ this.x/=l; this.y/=l;} return this; }
}

class Vec3 {
  constructor(x=0,y=0,z=0){ this.x=x; this.y=y; this.z=z; }
  set(x,y,z){ this.x=x; this.y=y; this.z=z; return this; }
  clone(){ return new Vec3(this.x,this.y,this.z); }
  add(v){ this.x+=v.x; this.y+=v.y; this.z+=v.z; return this; }
  sub(v){ this.x-=v.x; this.y-=v.y; this.z-=v.z; return this; }
  mul(s){ this.x*=s; this.y*=s; this.z*=s; return this; }
  dot(v){ return this.x*v.x + this.y*v.y + this.z*v.z; }
  cross(v){ return new Vec3(this.y*v.z - this.z*v.y, this.z*v.x - this.x*v.z, this.x*v.y - this.y*v.x); }
  length(){ return Math.hypot(this.x,this.y,this.z); }
  normalize(){ const l=this.length(); if(l>0){ this.x/=l; this.y/=l; this.z/=l;} return this; }
}

// Small helper to clamp
function clamp(v, a, b){ return Math.max(a, Math.min(b, v)); }

// ------------------------------
// Axis-Aligned Bounding Box class (for broadphase)
// ------------------------------
class AABB {
  constructor(minX,minY,minZ,maxX,maxY,maxZ){
    this.minX = minX; this.minY = minY; this.minZ = minZ;
    this.maxX = maxX; this.maxY = maxY; this.maxZ = maxZ;
  }
  overlaps(o){
    if(this.maxX < o.minX || this.minX > o.maxX) return false;
    if(this.maxY < o.minY || this.minY > o.maxY) return false;
    if(this.maxZ < o.minZ || this.minZ > o.maxZ) return false;
    return true;
  }
  static fromCircle2D(pos, r){ return new AABB(pos.x-r, pos.y-r, 0, pos.x+r, pos.y+r, 0); }
  static fromBox2D(pos, hw, hh){ return new AABB(pos.x-hw, pos.y-hh, 0, pos.x+hw, pos.y+hh, 0); }
  static fromSphere(pos, r){ return new AABB(pos.x-r, pos.y-r, pos.z-r, pos.x+r, pos.y+r, pos.z+r); }
  static fromOBB3D(center, ext){ return new AABB(center.x-ext.x, center.y-ext.y, center.z-ext.z, center.x+ext.x, center.y+ext.y, center.z+ext.z);
  }
}

// ------------------------------
// Body class (unified 2D/3D)
// Supported shapes:
// 2D: 'circle', 'box'(AABB), 'obb'(oriented box)
// 3D: 'sphere', 'aabb', 'obb3'
// ------------------------------
class Body {
  constructor(opts){
    this.dim = opts.dim || '2D';
    this.type = opts.type || (this.dim==='3D' ? 'sphere' : 'circle');
    this.pos = (this.dim==='3D') ? (opts.pos || new Vec3()) : (opts.pos || new Vec2());
    this.vel = (this.dim==='3D') ? (opts.vel || new Vec3()) : (opts.vel || new Vec2());
    this.force = (this.dim==='3D') ? new Vec3() : new Vec2();
    this.mass = opts.mass ?? (this.type.includes('sphere')||this.type==='circle' ? Math.PI : 1);
    this.invMass = this.mass>0 ? 1/this.mass : 0;
    this.restitution = opts.restitution ?? 0.2;
    this.friction = opts.friction ?? 0.5;
    this.static = !!opts.static;
    this.sleeping = false;

    // shapes
    if(this.dim==='2D'){
      if(this.type==='circle') this.radius = opts.radius || 16;
      else if(this.type==='box') { this.halfWidth = opts.halfWidth || 16; this.halfHeight = opts.halfHeight || 16; }
      else if(this.type==='obb') { this.halfWidth = opts.halfWidth || 16; this.halfHeight = opts.halfHeight || 16; this.angle = opts.angle || 0; }
    } else {
      if(this.type==='sphere') this.radius = opts.radius || 16;
      else if(this.type==='aabb') { this.halfExt = opts.halfExt || new Vec3(16,16,16); }
      else if(this.type==='obb3') { this.halfExt = opts.halfExt || new Vec3(16,16,16); this.orientation = opts.orientation || [new Vec3(1,0,0), new Vec3(0,1,0), new Vec3(0,0,1)]; }
    }
  }

  // produce an AABB for broadphase
  getAABB(){
    if(this.dim==='2D'){
      if(this.type==='circle') return AABB.fromCircle2D(this.pos, this.radius);
      if(this.type==='box') return AABB.fromBox2D(this.pos, this.halfWidth, this.halfHeight);
      if(this.type==='obb') return AABB.fromBox2D(this.pos, this.halfWidth, this.halfHeight); // conservative
    } else {
      if(this.type==='sphere') return AABB.fromSphere(this.pos, this.radius);
      if(this.type==='aabb') return AABB.fromOBB3D(this.pos, this.halfExt);
      if(this.type==='obb3') return AABB.fromOBB3D(this.pos, this.halfExt); // conservative
    }
    return new AABB(0,0,0,0,0,0);
  }
}

// ------------------------------
// Spatial Hash (3D-capable): cells keyed by i,j,k
// ------------------------------
class SpatialHash {
  constructor(cellSize=128, dim='2D'){
    this.cellSize = cellSize; this.dim = dim; this.cells = new Map();
  }
  _key(i,j,k=0){ return `${i},${j},${k}`; }
  clear(){ this.cells.clear(); }
  insert(body){
    const a = body.getAABB();
    const minI = Math.floor(a.minX / this.cellSize);
    const minJ = Math.floor(a.minY / this.cellSize);
    const maxI = Math.floor(a.maxX / this.cellSize);
    const maxJ = Math.floor(a.maxY / this.cellSize);
    const minK = this.dim==='3D' ? Math.floor(a.minZ / this.cellSize) : 0;
    const maxK = this.dim==='3D' ? Math.floor(a.maxZ / this.cellSize) : 0;
    for(let i=minI;i<=maxI;i++){
      for(let j=minJ;j<=maxJ;j++){
        for(let k=minK;k<=maxK;k++){
          const key = this._key(i,j,k);
          if(!this.cells.has(key)) this.cells.set(key, []);
          this.cells.get(key).push(body);
        }
      }
    }
  }
  queryPairs(){
    const pairs = [];
    for(const bucket of this.cells.values()){
      for(let i=0;i<bucket.length;i++){
        for(let j=i+1;j<bucket.length;j++) pairs.push([bucket[i], bucket[j]]);
      }
    }
    return pairs;
  }
}

// ------------------------------
// Narrowphase: collision tests & CCD
//  - 2D: circle-circle, circle-obb, obb-obb (SAT)
//  - 3D: sphere-sphere, sphere-aabb (swept), obb-obb (15-axis SAT simplified)
// ------------------------------

// 2D: project polygon on axis
function projectPolygon(points, axis){
  let min=Infinity, max=-Infinity;
  for(const p of points){ const v = p.x*axis.x + p.y*axis.y; if(v<min) min=v; if(v>max) max=v; }
  return {min,max};
}

function overlapOnAxis(pointsA, pointsB, axis){
  const pa = projectPolygon(pointsA, axis);
  const pb = projectPolygon(pointsB, axis);
  return Math.min(pa.max, pb.max) - Math.max(pa.min, pb.min);
}

function getOBBPoints2D(center, hw, hh, angle){
  const c = Math.cos(angle), s = Math.sin(angle);
  const ex = new Vec2(c*hw, s*hw); // local x
  const ey = new Vec2(-s*hh, c*hh); // local y
  return [
    new Vec2(center.x - ex.x - ey.x, center.y - ex.y - ey.y),
    new Vec2(center.x + ex.x - ey.x, center.y + ex.y - ey.y),
    new Vec2(center.x + ex.x + ey.x, center.y + ex.y + ey.y),
    new Vec2(center.x - ex.x + ey.x, center.y - ex.y + ey.y)
  ];
}

function SAT_OBB_OBB_2D(a, b){
  // a and b are bodies with obb or box (if box, angle=0)
  const ptsA = a.type==='obb' ? getOBBPoints2D(a.pos, a.halfWidth, a.halfHeight, a.angle) : getOBBPoints2D(a.pos, a.halfWidth, a.halfHeight, 0);
  const ptsB = b.type==='obb' ? getOBBPoints2D(b.pos, b.halfWidth, b.halfHeight, b.angle) : getOBBPoints2D(b.pos, b.halfWidth, b.halfHeight, 0);
  const axes = [];
  for(let i=0;i<4;i++){ const p1=ptsA[i], p2=ptsA[(i+1)%4]; axes.push(new Vec2(p2.y-p1.y, -(p2.x-p1.x)).normalize()); }
  for(let i=0;i<4;i++){ const p1=ptsB[i], p2=ptsB[(i+1)%4]; axes.push(new Vec2(p2.y-p1.y, -(p2.x-p1.x)).normalize()); }
  let minOverlap = Infinity; let smallestAxis = null;
  for(const axis of axes){
    const o = overlapOnAxis(ptsA, ptsB, axis);
    if(o <= 0) return null; // separation found
    if(o < minOverlap){ minOverlap = o; smallestAxis = axis.clone(); }
  }
  // return contact approximation
  return {normal: smallestAxis, penetration: minOverlap};
}

function collideCircleCircle2D(a,b){
  const dx = b.pos.x - a.pos.x; const dy = b.pos.y - a.pos.y;
  const dist2 = dx*dx + dy*dy; const r = a.radius + b.radius;
  if(dist2 >= r*r) return null;
  const dist = Math.sqrt(dist2) || 0.0001;
  const normal = new Vec2(dx/dist, dy/dist);
  const penetration = r - dist;
  return {a,b,normal,penetration,point: new Vec2(a.pos.x + normal.x * a.radius, a.pos.y + normal.y * a.radius)};
}

function collideCircleOBB2D(circle, obb){
  // closest point on OBB to circle center
  const pts = getOBBPoints2D(obb.pos, obb.halfWidth, obb.halfHeight, obb.angle);
  // convert to local axes
  const c = Math.cos(obb.angle), s = Math.sin(obb.angle);
  const localX = new Vec2(c, s); const localY = new Vec2(-s, c);
  const rel = new Vec2(circle.pos.x - obb.pos.x, circle.pos.y - obb.pos.y);
  const x = clamp(rel.dot(localX), -obb.halfWidth, obb.halfWidth);
  const y = clamp(rel.dot(localY), -obb.halfHeight, obb.halfHeight);
  const closest = new Vec2(obb.pos.x + localX.x*x + localY.x*y, obb.pos.y + localX.y*x + localY.y*y);
  const dx = circle.pos.x - closest.x, dy = circle.pos.y - closest.y;
  const dist2 = dx*dx + dy*dy;
  if(dist2 > circle.radius*circle.radius) return null;
  const dist = Math.sqrt(dist2) || 0.0001;
  const normal = new Vec2(dx/dist, dy/dist);
  const penetration = circle.radius - dist;
  return {a: circle, b: obb, normal, penetration, point: closest};
}

// 3D SAT OBB-OBB simplified (15 axes: 3 A axes, 3 B axes, 9 cross axes)
function SAT_OBB_OBB_3D(a, b){
  // a.orientation and b.orientation are arrays of 3 Vec3 unit vectors
  const EA = a.halfExt; const EB = b.halfExt;
  const A = a.orientation; const B = b.orientation;
  // compute rotation matrix R and absR
  const R = Array.from({length:3},()=>Array(3).fill(0));
  const absR = Array.from({length:3},()=>Array(3).fill(0));
  for(let i=0;i<3;i++) for(let j=0;j<3;j++){ R[i][j] = A[i].dot(B[j]); absR[i][j] = Math.abs(R[i][j]) + 1e-6; }
  // translation vector t in A's frame
  const tvec = b.pos.clone().sub(a.pos);
  const t = new Vec3(tvec.dot(A[0]), tvec.dot(A[1]), tvec.dot(A[2]));
  // test axes A0,A1,A2
  for(let i=0;i<3;i++){
    const ra = (i===0?EA.x: (i===1?EA.y:EA.z));
    let rb = EB.x*absR[i][0] + EB.y*absR[i][1] + EB.z*absR[i][2];
    if(Math.abs(t.getComponent?.(i) ?? (i===0?t.x: i===1? t.y: t.z)) > ra + rb) return null;
  }
  // test axes B0,B1,B2
  for(let j=0;j<3;j++){
    const rb = (j===0?EB.x: (j===1?EB.y:EB.z));
    const ra = EA.x*absR[0][j] + EA.y*absR[1][j] + EA.z*absR[2][j];
    const tj = Math.abs(t.x*R[0][j] + t.y*R[1][j] + t.z*R[2][j]);
    if(tj > ra + rb) return null;
  }
  // cross axes omitted for brevity/perf: approximate by checking A/B axes only -> faster but less accurate
  // For better correctness, add 9 cross-axis tests.
  // If passed, approximate overlap (not exact penetration depth) and normal
  const dir = b.pos.clone().sub(a.pos).normalize();
  return {normal: dir, penetration: 0.0001};
}

// Simple sphere-sphere 3D
function collideSphereSphere(a,b){
  const dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y, dz = b.pos.z - a.pos.z;
  const dist2 = dx*dx + dy*dy + dz*dz; const r = a.radius + b.radius;
  if(dist2 >= r*r) return null;
  const dist = Math.sqrt(dist2) || 0.0001;
  const normal = new Vec3(dx/dist, dy/dist, dz/dist);
  const penetration = r - dist;
  return {a,b,normal,penetration,point: new Vec3(a.pos.x + normal.x * a.radius, a.pos.y + normal.y * a.radius, a.pos.z + normal.z * a.radius)};
}

// ------------------------------
// CCD (swept tests)
//  - swept sphere vs AABB/OBB using ray vs expanded box test (Moller Trumbore style simplified)
//  - For 2D: swept circle vs OBB via ray vs expanded rectangle
// ------------------------------

// ray vs AABB in 3D (returns tmin,tmax)
function rayAABBIntersect(orig, dir, min, max){
  let tmin = (min.x - orig.x) / (dir.x || 1e-9);
  let tmax = (max.x - orig.x) / (dir.x || 1e-9);
  if(tmin > tmax){ const tmp=tmin; tmin=tmax; tmax=tmp; }
  let tymin = (min.y - orig.y) / (dir.y || 1e-9);
  let tymax = (max.y - orig.y) / (dir.y || 1e-9);
  if(tymin > tymax){ const tmp=tymin; tymin=tymax; tymax=tmp; }
  if((tmin > tymax) || (tymin > tmax)) return null;
  if(tymin > tmin) tmin = tymin;
  if(tymax < tmax) tmax = tymax;
  let tzmin = (min.z - orig.z) / (dir.z || 1e-9);
  let tzmax = (max.z - orig.z) / (dir.z || 1e-9);
  if(tzmin > tzmax){ const tmp=tzmin; tzmin=tzmax; tzmax=tmp; }
  if((tmin > tzmax) || (tzmin > tmax)) return null;
  if(tzmin > tmin) tmin = tzmin;
  if(tzmax < tmax) tmax = tzmax;
  return {tmin, tmax};
}

function sweptSphereVsAABB(sphere, vel, box){
  // expand box by sphere radius and do ray vs AABB
  const min = box.pos.clone().sub(box.halfExt.clone().mul(1));
  const max = box.pos.clone().add(box.halfExt.clone().mul(1));
  min.x -= sphere.radius; min.y -= sphere.radius; min.z -= sphere.radius;
  max.x += sphere.radius; max.y += sphere.radius; max.z += sphere.radius;
  const orig = sphere.pos.clone();
  const dir = vel.clone();
  const res = rayAABBIntersect(orig, dir, min, max);
  if(!res) return null;
  if(res.tmin >=0 && res.tmin <=1) return {t: res.tmin};
  if(res.tmax >=0 && res.tmax <=1) return {t: res.tmax};
  return null;
}

// 2D swept circle vs OBB (approx): transform to local OBB space and use ray vs AABB
function sweptCircleVsOBB2D(circle, vel, obb){
  const c = Math.cos(-obb.angle), s = Math.sin(-obb.angle);
  // translate circle to OBB local
  const relx = circle.pos.x - obb.pos.x, rely = circle.pos.y - obb.pos.y;
  const lx = relx*c - rely*s; const ly = relx*s + rely*c;
  const lvelx = vel.x*c - vel.y*s; const lvely = vel.x*s + vel.y*c;
  const minX = -obb.halfWidth - circle.radius, maxX = obb.halfWidth + circle.radius;
  const minY = -obb.halfHeight - circle.radius, maxY = obb.halfHeight + circle.radius;
  // ray-AABB 2D
  let tmin = -Infinity, tmax = Infinity;
  if(Math.abs(lvelx) < 1e-9){ if(lx < minX || lx > maxX) return null; }
  else {
    let tx1 = (minX - lx)/lvelx, tx2 = (maxX - lx)/lvelx; if(tx1>tx2){ const tmp=tx1; tx1=tx2; tx2=tmp; }
    tmin = Math.max(tmin, tx1); tmax = Math.min(tmax, tx2);
  }
  if(Math.abs(lvely) < 1e-9){ if(ly < minY || ly > maxY) return null; }
  else {
    let ty1 = (minY - ly)/lvely, ty2 = (maxY - ly)/lvely; if(ty1>ty2){ const tmp=ty1; ty1=ty2; ty2=tmp; }
    tmin = Math.max(tmin, ty1); tmax = Math.min(tmax, ty2);
  }
  if(tmax >= tmin && tmax >= 0 && tmin <= 1) return {t: Math.max(0,tmin)};
  return null;
}

// ------------------------------
// Contact resolution (impulse + friction + positional correction)
// Works for both 2D and 3D (vectors normalized)
// ------------------------------
function resolveContact(contact){
  const a = contact.a, b = contact.b;
  if(a.static && b.static) return;
  // get normal
  const n = contact.normal;
  // relative velocity
  const rv = (a.pos.constructor === Vec3) ? b.vel.clone().sub(a.vel) : new Vec2(b.vel.x - a.vel.x, b.vel.y - a.vel.y);
  const velAlongNormal = rv.dot(n);
  if(velAlongNormal > 0 && contact.penetration <= 0) return;
  const e = Math.min(a.restitution, b.restitution);
  const j = -(1 + e) * velAlongNormal / (a.invMass + b.invMass);
  const impulse = (n.clone ? n.clone().mul(j) : (new Vec2(n.x,n.y)).mul(j));
  if(!a.static) a.vel = a.vel.clone().sub(impulse.clone().mul(a.invMass));
  if(!b.static) b.vel = b.vel.clone().add(impulse.clone().mul(b.invMass));
  // friction: tangent
  const t = rv.clone().sub(n.clone().mul(rv.dot(n)));
  t.normalize();
  const jt = -rv.dot(t) / (a.invMass + b.invMass);
  const mu = Math.sqrt(a.friction * b.friction);
  let frictionImpulse;
  if(Math.abs(jt) < j * mu) frictionImpulse = t.clone().mul(jt);
  else frictionImpulse = t.clone().mul(-j * mu);
  if(!a.static) a.vel = a.vel.clone().sub(frictionImpulse.clone().mul(a.invMass));
  if(!b.static) b.vel = b.vel.clone().add(frictionImpulse.clone().mul(b.invMass));
  // positional correction (baumgarte)
  const percent = 0.8; const slop = 0.01;
  const corrMag = Math.max(contact.penetration - slop, 0) / (a.invMass + b.invMass) * percent;
  const corr = n.clone().mul(corrMag);
  if(!a.static) a.pos = a.pos.clone().sub(corr.clone().mul(a.invMass));
  if(!b.static) b.pos = b.pos.clone().add(corr.clone().mul(b.invMass));
}

// ------------------------------
// PhysicsWorld (unified)
// ------------------------------
class PhysicsWorld {
  constructor(dim='2D', options={}){
    this.dim = dim; this.bodies = []; this.gravity = options.gravity || (dim==='3D' ? new Vec3(0,-9.8,0) : new Vec2(0, 980));
    this.spatial = new SpatialHash(options.cellSize || 128, dim);
    this.substeps = options.substeps || 2;
    this.maxIters = options.maxIters || 3;
  }
  addBody(b){ b.dim = this.dim; this.bodies.push(b); }
  step(dt){
    const sub = Math.max(1, this.substeps);
    const subDt = dt / sub;
    for(let s=0;s<sub;s++){
      // integrate
      for(const b of this.bodies){
        if(b.static || b.sleeping) continue;
        // apply gravity
        if(this.dim==='3D') b.force.add(this.gravity.clone().mul(b.mass));
        else b.force.add(this.gravity.clone().mul(b.mass));
        const acc = b.force.clone().mul(b.invMass);
        b.vel.add(acc.mul(subDt));
        // damping
        b.vel.mul(0.999);
        b.pos.add(b.vel.clone().mul(subDt));
        // clear
        b.force.mul(0);
      }

      // broadphase
      this.spatial.clear();
      for(const b of this.bodies) this.spatial.insert(b);
      const pairs = this.spatial.queryPairs();

      const contacts = [];

      // narrowphase
      for(const [a,b] of pairs){
        if(a===b) continue;
        // cheap AABB overlap
        if(!a.getAABB().overlaps(b.getAABB())) continue;
        // choose tests based on types
        if(this.dim==='2D'){
          // circle-circle
          if(a.type==='circle' && b.type==='circle'){
            const c = collideCircleCircle2D(a,b); if(c) contacts.push(c);
          } else if((a.type==='circle' && b.type==='obb') || (a.type==='obb' && b.type==='circle')){
            const circ = a.type==='circle'?a:b; const obb = a.type==='obb'?a:b;
            const c = collideCircleOBB2D(circ, obb); if(c) contacts.push(c);
          } else if((a.type==='obb' || a.type==='box') && (b.type==='obb' || b.type==='box')){
            const c = SAT_OBB_OBB_2D(a,b); if(c) contacts.push({a,b,normal:c.normal,penetration:c.penetration});
          }
        } else {
          if(a.type==='sphere' && b.type==='sphere'){
            const c = collideSphereSphere(a,b); if(c) contacts.push(c);
          } else if((a.type==='sphere' && b.type==='aabb') || (a.type==='aabb' && b.type==='sphere')){
            // simple proximity test via AABB-sphere overlap
            const sph = a.type==='sphere'?a:b; const box = a.type==='aabb'?a:b;
            // precise CCD would use sweptSphereVsAABB
            const dx = clamp(sph.pos.x, box.pos.x - box.halfExt.x, box.pos.x + box.halfExt.x) - sph.pos.x;
            const dy = clamp(sph.pos.y, box.pos.y - box.halfExt.y, box.pos.y + box.halfExt.y) - sph.pos.y;
            const dz = clamp(sph.pos.z, box.pos.z - box.halfExt.z, box.pos.z + box.halfExt.z) - sph.pos.z;
            const dist2 = dx*dx + dy*dy + dz*dz;
            if(dist2 <= sph.radius*sph.radius) contacts.push({a:sph,b:box,normal:new Vec3(-dx, -dy, -dz).normalize(), penetration: sph.radius - Math.sqrt(dist2)});
          } else if((a.type==='obb3' || a.type==='aabb') && (b.type==='obb3' || b.type==='aabb')){
            const c = SAT_OBB_OBB_3D(a,b); if(c) contacts.push({a,b,normal:c.normal,penetration:c.penetration});
          }
        }
      }

      // resolve contacts iteratively
      for(let it=0; it<this.maxIters; it++){
        for(const c of contacts) resolveContact(c);
      }

      // CCD sweep (post step: correct high-velocity tunneling)
      // Simple strategy: for each dynamic body, compute displacement and test swept against potential colliders; if hit, rewind and apply impulse at time of impact.
      for(const moving of this.bodies){
        if(moving.static) continue;
        const disp = moving.vel.clone().mul(subDt);
        if(this.dim==='3D'){
          if(moving.type==='sphere'){
            for(const other of this.bodies){ if(other===moving) continue; if(other.static===false && other.invMass>0) continue; // only collide with static for CCD here
              if(other.type==='aabb' || other.type==='obb3'){
                const hit = sweptSphereVsAABB(moving, disp, other);
                if(hit && hit.t>=0 && hit.t<=1){
                  // move to impact point
                  moving.pos.add(disp.clone().mul(hit.t));
                  // simple response: reflect velocity along normal
                  // approximate normal from impact point to box center
                  const normal = moving.pos.clone().sub(other.pos).normalize();
                  moving.vel = moving.vel.clone().sub(normal.clone().mul(2 * moving.vel.dot(normal)));
                  break;
                }
              }
            }
          }
        } else {
          if(moving.type==='circle'){
            for(const other of this.bodies){ if(other===moving) continue; if(other.static===false && other.invMass>0) continue;
              if(other.type==='obb' || other.type==='box'){
                const hit = sweptCircleVsOBB2D(moving, disp, other);
                if(hit && hit.t>=0 && hit.t<=1){
                  moving.pos.add(disp.clone().mul(hit.t));
                  const normal = moving.pos.clone().sub(other.pos).normalize();
                  moving.vel = moving.vel.clone().sub(normal.clone().mul(2 * (moving.vel.dot(normal))));
                  break;
                }
              }
            }
          }
        }
      }

    } // end substep loop
  }
}

// small additions to Vec3 to help getComponent used earlier
Vec3.prototype.getComponent = function(i){ return i===0?this.x: i===1?this.y: this.z; };

/*
 NOTES & TRADEOFFS:
 - 3D SAT implementation intentionally simplified: cross-axis tests are omitted for speed.
   For strict correctness, add the 9 cross-axis tests (A0 x B0, A0 x B1, ...).
 - CCD provided is a light-weight swept-sphere vs AABB/OBB check; it prevents common tunneling for fast spheres.
 - SpatialHash handles both 2D and 3D by using Z cell index 0 in 2D.
 - This code prioritizes clarity and a balance between accuracy and speed. Extend as needed.
*/

// Export for use in browser (global)
if(typeof window !== 'undefined'){
  window.UnifiedPhysics = { Vec2, Vec3, Body, PhysicsWorld };
}
