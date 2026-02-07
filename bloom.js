"use strict";

class Bloom {
    static Tags = ['3d', 'novr'];
    constructor(glsl, gui) {
        this.glsl = glsl;
        this.bloom = 1.5; //2.0;
        this.gamma = 2.0; //2.2;
        if (gui) {
            gui.add(this, 'bloom', 0.0, 10.0, 0.01);
            gui.add(this, 'gamma', 0.1, 3.0, 0.01);
        }

        this.blurRadius = [ 3, 5, 7, 9, 11 ];
        this.blurKernel = this.blurRadius.map(r=>{
            const a = new Float32Array(12);
            a[0] = 1.0;
            let accum = a[0];
            for (let i=1; i<r; ++i) {
                a[i] = Math.exp(-5.0*i*i/r/r);
                accum += 2.0*a[i];
            }
            return a.map(x=>x/accum);
        });
    }

    compose(frame) {
        let [w, h] = frame.size;
        const glsl = this.glsl;
        let inputTex = glsl({T:frame.linear, 
            FP:`vec4 c = T(UV); FOut = (c-0.9)/0.1;`}, 
            {size: [w, h], filter:'linear', wrap:'edge', tag:'lum'});
        
        const lodN = this.blurRadius.length, lods = {};
        for (let lod=0; lod<lodN; ++lod, w/=2, h/=2) {
            for (const dir of [[1,0],[0,1]]) {
                lods['L'+lod] = inputTex = glsl({T:inputTex, dir, 
                    R:this.blurRadius[lod], kernel:this.blurKernel[lod], FP:`
                    uniform float kernel[12];
                    uniform int R;
                    void fragment() {
                        FOut = T(UV)*kernel[0];
                        vec2 dp = dir/vec2(ViewSize), p=dp;
                        for (int i=1; i<R; i+=1, p+=dp) {
                            FOut += kernel[i] * (T(UV+p) + T(UV-p));
                        }
                    }`}, {size:[w, h], story:2, format:'rgb11f', 
                        filter:'linear', wrap:'edge', tag:`lod${lod}`})[0];
            }
        }
        const {bloom, gamma} = this;
        glsl({...lods, T:frame, bloom, gamma, FP:`
            vec4 acc = T(UV) + bloom*(L0(UV) + L1(UV) + L2(UV) + L3(UV) + L4(UV));
            FOut = pow(acc, vec4(1./gamma));
        `})
    }
}
