const MaxPointN = 1<<16;

function prepareWASM(instance) {
    const type2class = {uint8_t: Uint8Array, int: Int32Array,
        uint32_t: Uint32Array, uint64_t: BigUint64Array, float: Float32Array};
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

class GraphLayout {
    constructor(wasm, dim=2) {
        this.wasm = prepareWASM(wasm);
        this.dim = dim;
        this.velocityDecay = 0.1;
        this.pointN = 0;
        this.pos = this.wasm.points;
        this.vel = this.wasm.vel;

        this.chargeStrength = -3.0;
        this.chargeMaxDist = 2000;

        this.linkIterN = 2;
        this.linkDistance = 25;
        this.linkStrength = 0.5;

        // Expose octree buffers for visualization
        this.nodeCount = 0;
        this.nodeCenter = this.wasm.node_center;
        this.nodeMin = this.wasm.node_min;
        this.nodeMax = this.wasm.node_max;
        this.nodeMass = this.wasm.node_mass;
        this.nodeExtent = this.wasm.node_extent;
        this.nodeParent = this.wasm.node_parent;
    }

    linkForce() {
        this.wasm.linkForce(this.linkN, this.linkStrength, this.linkDistance);
    }

    chargeForce() {
        const {pointN} = this;
        
        // Use WASM for octree construction
        this.nodeCount = this.wasm.buildOctree(pointN, 16, 10);
        this.treeExtent = this.wasm.get_tree_extent();
        this.treeCenter = this.wasm.tree_center; // From BUFFER(tree_center, ...)

        this.wasm.accumPoints(this.nodeCount, this.treeExtent);
        this.wasm.calcMultibodyForceDual(pointN, this.nodeCount, this.chargeMaxDist);
        this.wasm.applyChargeForces(pointN, this.chargeStrength);
    }

    updateData(newNodeCount, links, newNodesParents) {
        this.linkN = links.length / 2;
        this.wasm.links.set(links);

        // init new points
        const {dim} = this;
        const rnd = ()=>(Math.random()-0.5);
        if (this.pointN < newNodeCount) {
            for (let i=this.pointN; i<newNodeCount; ++i) {
                let nn = 0;
                let x=rnd(), y=rnd(), z=dim==3 ? rnd() : 0.0;
                let hintArray = (newNodesParents && newNodesParents[i]) ? newNodesParents[i] : [];
                for (const j of hintArray) {
                    if (j>=this.pointN) continue; // skip new points
                    let p=j*3; nn++;
                    x+=this.pos[p]; y+=this.pos[p+1]; z+=this.pos[p+2];
                }
                if (nn>1) {x/=nn; y/=nn; z/=nn;}
                let p=i*3; this.pos[p]=x; this.pos[p+1]=y; this.pos[p+2]=z;
            }
            this.pointN = newNodeCount;
        }
    }

    tick(step_n=1) {
        const f = 1.0-this.velocityDecay;
        for (let k=0; k<step_n; ++k) {
            for (let i=0; i<this.linkIterN; ++i) this.linkForce();
            this.chargeForce();
            this.wasm.updateNodes(this.pointN, f);
        }
    }
}