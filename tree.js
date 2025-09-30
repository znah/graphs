
"use strict";

function dilate3(x) {
    x &= 0x3ff; // Mask to 10 bits
    x = (x | (x << 16)) & 0x30000ff;
    x = (x | (x << 8))  & 0x300f00f;
    x = (x | (x << 4))  & 0x30c30c3;
    x = (x | (x << 2))  & 0x9249249;
    return x;
}

class Octree {
    /**
     * @param {Float32Array} points - Flat array of [x0, y0, z0, x1, y1, z1, ...]
     * @param {number} [leafSize=16] - Max points in a leaf node range
     */    
    constructor(points, leafSize=16) {
        this.points = points;
        this.leafSize = leafSize;
        this.pointN = points.length / 3;
        this.maxLevel = 10;

        this._createMortonCodes();
        this._sortPoints();
        this.nodes = [];
        this.root = this._buildNode(0, 0, this.pointN, this.bounds.lo, 0);
    }

    _createMortonCodes() {
        let lo = [ Infinity,  Infinity,  Infinity];
        let hi = [-Infinity, -Infinity, -Infinity];
        for (let i=0; i<this.points.length; ++i) {
            const v = this.points[i];
            const c = i % 3;
            lo[c] = Math.min(lo[c], v);
            hi[c] = Math.max(hi[c], v);
        }
        this.extent = Math.max(hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2]);
        const center = [(hi[0]+lo[0])/2, (hi[1]+lo[1])/2, (hi[2]+lo[2])/2]
        const h = this.extent/2;
        lo = [center[0]-h, center[1]-h, center[2]-h];
        hi = [center[0]+h, center[1]+h, center[2]+h];
        this.bounds = {lo, hi};
        const scale = 1023.0 / (this.extent+1e-8);
        const codes = new BigUint64Array(this.pointN);
        for (let i = 0; i < this.pointN; ++i) {
            codes[i] = BigInt(i);
            for (let c=0; c<3; ++c) {
                const v = this.points[i * 3 + c];
                const nv = (v - lo[c]) * scale;
                codes[i] |= BigInt(dilate3(nv | 0) << c)<<32n;
            }
        }
        codes.sort();
        this.codes = new Uint32Array(this.pointN);
        this.indices = new Uint32Array(this.pointN);
        for (let i = 0; i < this.pointN; ++i) {
            this.codes[i] = Number(codes[i] >> 32n);
            this.indices[i] = Number(codes[i] & 0xFFFFFFFFn);
        }
    }

    _sortPoints() {
        const newPoints = new Float32Array(this.pointN * 3);
        for (let i=0; i<this.pointN; ++i) {
            const j = this.indices[i];
            for (let c=0; c<3; ++c) {
                newPoints[i*3+c] = this.points[j*3+c];
            }
        }
        this.points = newPoints;
    }

    _buildNode(level, start, end, pos, parentIdx) {
        const node = {start, end, level, pos, parentIdx};
        const ownIdx = this.nodes.length;
        this.nodes.push(node);
        if (end - start <= this.leafSize || level == this.maxLevel) {
            node.next = this.nodes.length;
            return node;
        }
        const count = new Uint32Array(8);
        const shift = (this.maxLevel-level-1) * 3;
        for (let i=start; i<end; ++i) {
            const octant = (this.codes[i] >> shift) & 0x7;
            count[octant]++;
        }
        node.children = [];
        const half = this.extent / (1<<(level+1));
        for (let i=0; i<8; ++i) {
            if (count[i]) {
                const childPos = pos.map((p, c) => p + (((i >> c) & 1) * half));
                node.children[i] = this._buildNode(level+1, start, start+count[i], childPos, ownIdx);
            }
            start += count[i];
        }
        node.next = this.nodes.length;
        return node;
    }
};