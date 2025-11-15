<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Unified Physics Engine Demo 2D & 3D</title>
<style>
  body { margin:0; overflow:hidden; display:flex; }
  canvas { flex:1; display:block; }
</style>
</head>
<body>
<canvas id="canvas2D"></canvas>
<canvas id="canvas3D"></canvas>
<script src="UnifiedPhysicsEngine.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js"></script>

<script>
// ----------------------------
// 2D Setup
// ----------------------------
const canvas2D = document.getElementById('canvas2D');
canvas2D.width = window.innerWidth/2;
canvas2D.height = window.innerHeight;
const ctx2D = canvas2D.getContext('2d');
const world2D = new UnifiedPhysics.PhysicsWorld('2D', {gravity: new UnifiedPhysics.Vec2(0,500)});

// Floor for 2D
const floor2D = new UnifiedPhysics.Body({type:'box', pos:new UnifiedPhysics.Vec2(canvas2D.width/2, canvas2D.height-10), halfWidth:canvas2D.width/2, halfHeight:10, static:true});
world2D.addBody(floor2D);

// Function to add 2D props
function add2DProp(){
  const types = ['circle','obb'];
  const type = types[Math.floor(Math.random()*types.length)];
  if(type==='circle'){
    const circle = new UnifiedPhysics.Body({type:'circle', pos:new UnifiedPhysics.Vec2(Math.random()*canvas2D.width,50), radius:15+Math.random()*15, mass:1});
    world2D.addBody(circle);
  } else {
    const obb = new UnifiedPhysics.Body({type:'obb', pos:new UnifiedPhysics.Vec2(Math.random()*canvas2D.width,50), halfWidth:15+Math.random()*15, halfHeight:15+Math.random()*15, angle:Math.random()*Math.PI});
    world2D.addBody(obb);
  }
}

canvas2D.addEventListener('click', add2DProp);

// ----------------------------
// 3D Setup
// ----------------------------
const canvas3D = document.getElementById('canvas3D');
canvas3D.width = window.innerWidth/2;
canvas3D.height = window.innerHeight;
const renderer = new THREE.WebGLRenderer({canvas: canvas3D});
renderer.setSize(canvas3D.width, canvas3D.height);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);
const camera = new THREE.PerspectiveCamera(60, canvas3D.width/canvas3D.height, 0.1, 1000);
camera.position.set(0,50,150);
const light = new THREE.DirectionalLight(0xffffff,1);
light.position.set(50,100,50);
scene.add(light);

const world3D = new UnifiedPhysics.PhysicsWorld('3D', {gravity: new UnifiedPhysics.Vec3(0,-9.8,0)});

// Floor for 3D
const floor3D = new UnifiedPhysics.Body({type:'aabb', pos:new UnifiedPhysics.Vec3(0,-10,0), halfExt:new UnifiedPhysics.Vec3(100,5,100), static:true});
world3D.addBody(floor3D);
const floorMesh = new THREE.Mesh(new THREE.BoxGeometry(200,10,200), new THREE.MeshPhongMaterial({color:0x555555}));
floorMesh.position.copy(floor3D.pos);
scene.add(floorMesh);

// Function to add 3D props
const meshes3D = [];
function add3DProp(){
  const types = ['sphere','obb3'];
  const type = types[Math.floor(Math.random()*types.length)];
  let body, mesh;
  if(type==='sphere'){
    const r = 5+Math.random()*10;
    body = new UnifiedPhysics.Body({type:'sphere', pos:new UnifiedPhysics.Vec3((Math.random()-0.5)*100,50,(Math.random()-0.5)*100), radius:r, mass:1});
    mesh = new THREE.Mesh(new THREE.SphereGeometry(r,16,16), new THREE.MeshPhongMaterial({color:0xff5500}));
  } else {
    const ext = new UnifiedPhysics.Vec3(5+Math.random()*10,5+Math.random()*10,5+Math.random()*10);
    body = new UnifiedPhysics.Body({type:'obb3', pos:new UnifiedPhysics.Vec3((Math.random()-0.5)*100,50,(Math.random()-0.5)*100), halfExt:ext});
    mesh = new THREE.Mesh(new THREE.BoxGeometry(ext.x*2,ext.y*2,ext.z*2), new THREE.MeshPhongMaterial({color:0x00aaff}));
  }
  world3D.addBody(body);
  mesh.userData.body = body;
  meshes3D.push(mesh);
  scene.add(mesh);
}

canvas3D.addEventListener('click', add3DProp);

// ----------------------------
// Render loop
// ----------------------------
function animate(){
  requestAnimationFrame(animate);

  // step physics
  world2D.step(1/60);
  world3D.step(1/60);

  // draw 2D
  ctx2D.clearRect(0,0,canvas2D.width,canvas2D.height);
  for(const b of world2D.bodies){
    ctx2D.save();
    ctx2D.translate(b.pos.x, b.pos.y);
    if(b.type==='circle'){
      ctx2D.beginPath(); ctx2D.arc(0,0,b.radius,0,Math.PI*2); ctx2D.fillStyle='orange'; ctx2D.fill();
    } else if(b.type==='box' || b.type==='obb'){
      ctx2D.rotate(b.angle||0);
      ctx2D.fillStyle='skyblue'; ctx2D.fillRect(-b.halfWidth,-b.halfHeight,b.halfWidth*2,b.halfHeight*2);
    }
    ctx2D.restore();
  }

  // draw 3D
  for(const mesh of meshes3D){
    const b = mesh.userData.body;
    mesh.position.copy(b.pos);
    if(b.type==='obb3'){
      mesh.rotation.set(0,0,0); // orientation simplification
    }
  }
  renderer.render(scene, camera);
}
animate();
</script>
</body>
</html>
