const MaxPointN = 1<<16;

function prepareWASM(instance) {
    const type2class = {uint8_t: Uint8Array, int: Int32Array, uint32_t: Uint32Array, uint64_t: BigUint64Array};
    const prefix = '_len_';
    const exports = instance.exports;
    const main = {};
    const arrays = {}
    for (const key in exports) {
        if (!key.startsWith(prefix)) {
            if (!key.startsWith('_')) {
                main[key] = exports[key];
            }
            continue;
        }
        const [name, type] = key.slice(prefix.length).split('__');
        Object.defineProperty(main, name, {
            enumerable: true,
            get() {
                if (!(name in arrays) || arrays[name].buffer != exports.memory.buffer) { 
                    const ofs = exports['_get_'+name]();
                    const len = exports[key]();
                    if (!ofs) {
                        return null;
                    }
                    arrays[name] = new (type2class[type])(exports.memory.buffer, ofs, len);
                }
                return arrays[name];
            }
        });
    }
    return main;
}

function accumPoints(tree) {
    const centers = new Float32Array(tree.nodes.length * 4);
    const mass = new Float32Array(tree.nodes.length);
    for (let ni=tree.nodes.length-1; ni>=0; --ni) {
        const node = tree.nodes[ni];
        centers[ni*4+3] = tree.extent / (1<<node.level);
        mass[ni] = node.end - node.start;
        if (node.next == ni+1) { // leaf
            for (let i=node.start; i<node.end; ++i) {
                for (let c=0; c<3; ++c) {
                    centers[ni*4+c] += tree.points[i*3+c];
                }
            }
        } 
        if (node.parentIdx == ni) continue
        for (let c=0; c<3; ++c) {
            centers[node.parentIdx*4+c] += centers[ni*4+c];
        }
    }
    for (let i=0; i<tree.nodes.length; ++i) {
        const m = mass[i];
        centers[i*4] /= m;
        centers[i*4+1] /= m;
        centers[i*4+2] /= m;
    }
    return {centers, mass};
}

function calcForces(points, tree, centers, maxDist) {
    const maxW = tree.extent;
    const theta2 = 0.81; // Barnes-Hut theta squared
    const maxDist2 = maxDist*maxDist;

    const nodeN = tree.nodes.length;
    const forces = new Float32Array(points.length);
    const start = new Int32Array(nodeN);
    const end = new Int32Array(nodeN);
    const next = new Int32Array(nodeN);
    for (let i=0; i<nodeN; ++i) {
        const node = tree.nodes[i];
        start[i] = node.start;
        end[i] = node.end;
        next[i] = node.next;
    }
    let p;
    for (let pointI = 0; pointI < points.length/3; ++pointI) {
        const [x,y,z] = points.subarray(pointI*3, pointI*3+3)
        for (let nodeI=0; nodeI<nodeN;) {
            //const node = tree.nodes[nodeI];
            p=nodeI*4;
            const dx=centers[p++]-x, dy=centers[p++]-y, dz=centers[p++]-z, w=centers[p];
            const l2 = dx*dx + dy*dy + dz*dz;
            const tooFar = maxDist2 < l2;
            if (w*w < theta2*l2) {
                // Far enough, treat as single body
                if (!tooFar) {
                    const mass = end[nodeI]-start[nodeI];
                    const c = mass / (1.0+l2);
                    p = pointI*3;
                    forces[p++] += c*dx; forces[p++] += c*dy; forces[p] += c*dz;
                }
                nodeI = next[nodeI];
            } else {
                if (next[nodeI]  == nodeI+1 && !tooFar) { // leaf
                    for (let i=start[nodeI]*3; i<end[nodeI]*3; i+=3) {
                        const dx=points[i]-x, dy=points[i+1]-y, dz=points[i+2]-z;
                        const l2 = dx*dx + dy*dy + dz*dz;
                        const c = 1.0 / (1.0+l2);
                        p = pointI*3;
                        forces[p++] += c*dx; forces[p++] += c*dy; forces[p] += c*dz;
                    }
                }
                ++nodeI;
            }
        }
    }
    return {forces, indices:tree.indices, tree, centers};
}

function forceLink({distance=20, iterations=3, strength=0.4}={}) {
    function force(layout) {
        const {nodes, pos, vel} = layout;
        
        for (let k=0; k<iterations; ++k) {
            for (let i=0; i<nodes.length; ++i) {
                for (const j of nodes[i]) {
                    if (i > j) continue;
                    let x = pos[j*3]   + vel[j*3]   - pos[i*3]   - vel[i*3];
                    let y = pos[j*3+1] + vel[j*3+1] - pos[i*3+1] - vel[i*3+1];
                    let z = pos[j*3+2] + vel[j*3+2] - pos[i*3+2] - vel[i*3+2];
                    let l = Math.max(Math.hypot(x, y, z), 1.0);
                    l = (l - distance) / l * strength;
                    x *= l, y *= l, z *= l;
                    vel[j*3] -= x; vel[j*3+1] -= y; vel[j*3+2] -= z;
                    vel[i*3] += x; vel[i*3+1] += y; vel[i*3+2] += z;
                }
            }
        }
    }
    return force;
}

class GraphLayout {
    constructor(nodes, wasm, dim=2) {
        this.nodes = nodes;
        this.wasm = prepareWASM(wasm);
        this.dim = dim;
        this.velocityDecay = 0.1;
        this.pointN = 0;
        this.pos = new Float32Array(MaxPointN*3);
        this.vel = new Float32Array(MaxPointN*3);
        this.linkForce = forceLink();

        this.chargeStrength = -3.0;
        this.chargeMaxDist = 2000;
    }

    chargeForce() {
        const {pointN} = this;
        const pos = this.pos.subarray(0, pointN*3);
        const tree = this.tree = new Octree(pos);
        const points = tree.points; // use points sorted by Octree
        const {centers} = accumPoints(tree);

        const {forces, indices} = calcForces(points, tree, centers, this.chargeMaxDist);
        const {vel} = this;
        const strength = this.chargeStrength;
        for (let i=0; i<indices.length; ++i) {
            const k = indices[i];
            vel[k*3]   += strength*forces[i*3];
            vel[k*3+1] += strength*forces[i*3+1];
            vel[k*3+2] += strength*forces[i*3+2];
        }
    }

    tick(step_n=1) {
        const {pos, vel} = this;
        const f = 1.0-this.velocityDecay;
        const rnd = ()=>(Math.random()-0.5)*10.0;
        // init new points
        for (let i=this.pointN; i<this.nodes.length; ++i) {
            let nn = 0;
            let x=0, y=0, z=0;
            for (const j of this.nodes[i]) {
                if (j>=this.pointN) continue; // skip new points
                let p=j*3; nn++;
                x+=this.pos[p++]; y+=this.pos[p++]; z+=this.pos[p];
            }
            if (nn>0) {
                x/=nn; y/=nn; z/=nn;
            } else {
                x=rnd(); y=rnd(); z=this.dim==3 ? rnd() : 0.0;
            }
            let p=i*3; this.pos[p++]=x; this.pos[p++]=y; this.pos[p]=z;
        }
        this.pointN = this.nodes.length;

        for (let k=0; k<step_n; ++k) {
            this.linkForce(this);
            this.chargeForce();
            for (let i=0; i<this.pointN*3; ++i) {
                pos[i] += vel[i];
                vel[i] *= f;
            }
        }
    }
}